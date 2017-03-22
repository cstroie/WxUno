/**
  WxSta - Weather Station

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxSta is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxSta is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxSta.  If not, see <http://www.gnu.org/licenses/>.


  WiFi connected weather station, reading the athmospheric sensor BME280 and
  the illuminance sensor TSL2561 and publishing the measured data along with
  various local telemetry.
*/

// The DEBUG and DEVEL flag
#define DEBUG
#define DEVEL

// The sensors are connected to I2C
#include <Wire.h>
#include <SparkFunBME280.h>

// Ethernet
#include <Ethernet.h>
#include <EthernetUdp.h>

// NTP
#include <TimeLib.h>

// Statistics
#undef RUNNING_MEDIAN_ALL
#include <RunningMedian.h>
#undef RUNNING_MEDIAN_ALL

// Device name
char NODENAME[] = "WxUno";
char VERSION[] = "0.1";

// Ethernet
// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 0, 177);

// NTP
const int TZ = 0;
const char *NTP_SERVER = "europe.pool.ntp.org";
const int  NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
unsigned int ntpLocalPort = 8888;
EthernetUDP UDP;

// APRS parameters
const char *APRS_SERVER = "cwop5.aprs.net";
const int  APRS_PORT = 14580;
const char *APRS_CALLSIGN = "FW0727";
const int  APRS_PASSCODE = -1;
const char *APRS_LOCATION = "4455.29N/02527.08E";
const int  ALTI = 282; // meters
float ALTI_CORR = pow((float)(1.0 - 2.25577e-5 * ALTI), (float)(-5.25588));
long ALTIF = (long)(ALTI * 3.28084);
const byte APRS_SNS_MAX = 5; // x SNS_DELAY
byte APRS_SNS_COUNT = 0;
unsigned int aprsSeq = 0;
EthernetClient APRS_Client;

// Statistics
RunningMedian rmTemp = RunningMedian(APRS_SNS_MAX);
RunningMedian rmHmdt = RunningMedian(APRS_SNS_MAX);
RunningMedian rmPres = RunningMedian(APRS_SNS_MAX);
RunningMedian rmVcc  = RunningMedian(APRS_SNS_MAX);
RunningMedian rmRSSI = RunningMedian(APRS_SNS_MAX);
RunningMedian rmHeap = RunningMedian(APRS_SNS_MAX);

// Sensors
const unsigned long SNS_DELAY = 60UL * 1000UL;
unsigned long SNS_NEXT = 0UL;
BME280 atmo;
bool atmo_ok = false;

void aprsSendHeader(char sep) {
  APRS_Client.print(APRS_CALLSIGN);
  APRS_Client.print(F(">APRS,TCPIP*:"));
  APRS_Client.print(sep);
}

/**
  Return time in APRS format: DDHHMMz
*/
char *aprsTime() {
  time_t moment = now();
  char buf[8];
  sprintf(buf, "%02d%02d%02dz", day(moment), hour(moment), minute(moment));
  return buf;
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxSta 0.2"
*/
void aprsAuthenticate() {
  APRS_Client.print(F("user "));
  APRS_Client.print(APRS_CALLSIGN);
  APRS_Client.print(F(" pass "));
  APRS_Client.print(APRS_PASSCODE);
  APRS_Client.print(F(" vers "));
  APRS_Client.print(NODENAME);
  APRS_Client.print(F(" "));
  APRS_Client.println(VERSION);
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxSta

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void aprsSendWeather(float temp, float hmdt, float pres, float lux) {
  // Temperature will be in Fahrenheit
  float fahr = temp * 9 / 5 + 32;
  aprsSendHeader('@');
  // Compose the APRS packet
  APRS_Client.print(aprsTime());
  APRS_Client.print(APRS_LOCATION);
  // Wind
  APRS_Client.print(F("_.../...g..."));
  // Temperature
  if (temp >= -273.15) {
    char buf[5];
    sprintf(buf, "t%03d", (int)fahr);
    APRS_Client.print(buf);
  }
  else {
    APRS_Client.print(F("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      APRS_Client.print(F("h00"));
    }
    else {
      char buf[5];
      sprintf(buf, "h%02d", (int)hmdt);
      APRS_Client.print(buf);
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    char buf[7];
    sprintf(buf, "b%05d", (int)(pres / 10));
    APRS_Client.print(buf);
  }
  // Illuminance, if valid
  if (lux >= 0) {
    char buf[5];
    sprintf(buf, "L%02d", (int)(lux * 0.0079));
    APRS_Client.print(buf);
  }
  // Comment (device name)
  APRS_Client.println(NODENAME);
}

/**
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param vcc voltage
  @param rssi wifi level
  @param heap free memory
  @param luxVis raw visible illuminance
  @param luxIrd raw infrared illuminance
  @bits digital inputs
*/
void aprsSendTelemetry(int vcc, int rssi, int heap, unsigned int luxVis, unsigned int luxIrd, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  aprsSeq += 1;
  if (aprsSeq > 999) aprsSeq = 0;
  // Send the setup, if the sequence number is 0
  if (aprsSeq == 260) aprsSendTelemetrySetup();
  // Compose the APRS packet
  aprsSendHeader('T');
  char buf[40];
  sprintf(buf, "#%03d,%03d,%03d,%03d,%03d,%03d,", aprsSeq, (int)((vcc - 2500) / 4), -rssi, (int)(heap / 200), luxVis, luxIrd);
  APRS_Client.print(buf);
  APRS_Client.println(F("00000000"));
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  char sep = ':';
  char padCallSign[10];
  sprintf(padCallSign, "%9s", APRS_CALLSIGN);
  // Parameter names
  aprsSendHeader(sep);
  APRS_Client.print(padCallSign);
  APRS_Client.println(F(":PARM.Vcc,RSSI,Heap,Temp,NA,B1,B2,B3,B4,B5,B6,B7,B8"));
  // Equations
  aprsSendHeader(sep);
  APRS_Client.print(padCallSign);
  APRS_Client.println(F(":EQNS.0,0.004,2.5,0,-1,0,0,200,0,0,1,0,0,1,0"));
  // Units
  aprsSendHeader(sep);
  APRS_Client.print(padCallSign);
  APRS_Client.println(F(":UNIT.V,dBm,Bytes,C,units,N/A,N/A,N/A,N/A,N/A,N/A,N/A,N/A"));
  // Bit sense and project name
  aprsSendHeader(sep);
  APRS_Client.print(padCallSign);
  APRS_Client.print(F(":BITS.11111111, "));
  APRS_Client.print(NODENAME);
  APRS_Client.print(F("/"));
  APRS_Client.println(VERSION);
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>13:06 Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message != "") {
    // Send the APRS packet
    aprsSendHeader('>');
    APRS_Client.println(message);
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment) {
  // Compose the APRS packet
  aprsSendHeader('!');
  APRS_Client.print(APRS_LOCATION);
  APRS_Client.print(F("/000/000/A="));
  char buf[7];
  sprintf(buf, "%06d", ALTIF);
  APRS_Client.print(buf);
  APRS_Client.println(comment);
}

time_t getNtpTime()
{
  while (UDP.parsePacket() > 0) ; // discard any previously received packets
  sendNTPpacket();
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = UDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println(F("NTP sync"));
      UDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + TZ * SECS_PER_HOUR;
    }
  }
  Serial.println(F("No NTP sync"));
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket()
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  UDP.beginPacket(NTP_SERVER, 123); //NTP requests are to port 123
  UDP.write(packetBuffer, NTP_PACKET_SIZE);
  UDP.endPacket();
}

double GetTemp(void)
{
  unsigned int wADC;
  double t;

  // The internal temperature has to be used
  // with the internal reference of 1.1V.
  // Channel 8 can not be selected with
  // the analogRead function yet.

  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  // enable the ADC

  delay(20);            // wait for voltages to become stable.

  ADCSRA |= _BV(ADSC);  // Start the ADC

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  wADC = ADCW;

  // The offset could be wrong. It is just an indication.
  t = (wADC - 335.2 ) / 1.06154;

  // The returned temperature is in degrees Celsius.
  return (t);
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(9600);
  Serial.println(NODENAME);

  // Start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    Ethernet.begin(mac, ip);
  }
  // Print the Ethernet shield's IP address:
  Serial.print(F("IP address: "));
  Serial.println(Ethernet.localIP());
  // Give the Ethernet shield a second to initialize:
  delay(1000);

  // Start the NTP sync
  UDP.begin(ntpLocalPort);
  setSyncProvider(getNtpTime);

  // BME280
  atmo.settings.commInterface = I2C_MODE;
  atmo.settings.I2CAddress = 0x77;
  atmo.settings.runMode = 3;
  atmo.settings.tStandby = 0;
  atmo.settings.filter = 0;
  atmo.settings.tempOverSample = 1;
  atmo.settings.pressOverSample = 1;
  atmo.settings.humidOverSample = 1;
  delay(10);
  if (atmo.begin() == 0x60) {
    atmo_ok = true;
    Serial.println(F("BME280 sensor detected."));
  }
  else {
    atmo_ok = false;
    Serial.println(F("BME280 sensor missing."));
  }

  // Initialize the random number generator and set the APRS telemetry start sequence
  if (timeStatus() != timeNotSet) randomSeed(now());
  aprsSeq = random(1000);

  // Start the sensor timer
  SNS_NEXT = millis() + SNS_DELAY;
}

void loop() {
  // Read the sensors and publish telemetry
  if (millis() >= SNS_NEXT) {
    // Check the DHCP lease
    Ethernet.maintain();
    // Check if we need to send the APRS data
    APRS_SNS_COUNT++;
    if (APRS_SNS_COUNT > APRS_SNS_MAX) APRS_SNS_COUNT = 1;

    // Read BME280
    float temp, pres, slvl, hmdt, dewp;
    if (atmo_ok) {
      // Get the weather parameters
      temp = atmo.readTempC();
      pres = atmo.readFloatPressure();
      slvl = pres * ALTI_CORR;
      hmdt = atmo.readFloatHumidity();
      dewp = 243.04 * (log(hmdt / 100.0) + ((17.625 * temp) / (243.04 + temp))) / (17.625 - log(hmdt / 100.0) - ((17.625 * temp) / (243.04 + temp)));
      // Running Median
      rmTemp.add(temp);
      rmHmdt.add(hmdt);
      rmPres.add(slvl);
    }

    rmTemp.add(GetTemp());

    // Various telemetry
    int rssi = -67;
    int heap = 234;
    int vcc  = 3204;
    // Running Median
    rmVcc.add(vcc);
    rmRSSI.add(rssi);
    rmHeap.add(heap);

    // APRS (after the first minute, then every APRS_SNS_MAX minutes)
    if (APRS_SNS_COUNT == 1) {
      if (APRS_Client.connect(APRS_SERVER, APRS_PORT)) {
        //Serial.print(F("Connected to "));
        //Serial.println(APRS_SERVER);
        aprsAuthenticate();
        //aprsSendPosition(" WxStaProbe");
        if (atmo_ok) {
          aprsSendWeather(rmTemp.getMedian(), rmHmdt.getMedian(), rmPres.getMedian(), -1);
        }
        aprsSendWeather(rmTemp.getMedian(), -1, -1, -1);
        aprsSendTelemetry(rmVcc.getMedian(), rmRSSI.getMedian(), rmHeap.getMedian(), 0, 0, 0);
        APRS_Client.stop();
      }
      else Serial.println(F("Connection failed"));
    }

    // Repeat sensor reading
    SNS_NEXT = SNS_NEXT + SNS_DELAY;
  }
}
