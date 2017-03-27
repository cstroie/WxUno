/**
  WxUno - Weather Station for Arduino UNO

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of Weather Station.

  WxUno is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by the Free
  Software Foundation, either version 3 of the License, or (at your option) any
  later version.

  WxUno is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  WxUno.  If not, see <http://www.gnu.org/licenses/>.


  WiFi connected weather station, reading the athmospheric sensor BME280 and
  the illuminance sensor TSL2561 and publishing the measured data along with
  various local telemetry.
*/

// The DEBUG and DEVEL flag
#define DEBUG
#define DEVEL

// The sensors are connected to I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// Ethernet
#include <Ethernet.h>
#include <EthernetUdp.h>

// NTP
#include <TimeLib.h>

// Device name
char *NODENAME = "WxUno";
char *VERSION = "1.0";
bool  PROBE = true;    // True if the station is being probed

// Ethernet
// assign a MAC address for the ethernet controller.
// fill in your address here:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// Set the static IP address to use if the DHCP fails to assign
IPAddress ip(192, 168, 0, 177);

// NTP
const int   TZ = 0;
const char *NTP_SERVER = "europe.pool.ntp.org";
const int   NTP_PACKET_SIZE = 48;
byte packetBuffer[NTP_PACKET_SIZE];
unsigned int ntpLocalPort = 8888;
EthernetUDP UDP;

// APRS parameters
const char *aprsServer = "cwop5.aprs.net";
const int   aprsPort = 14580;
const char *aprsCallSign = "FW0727";
const char *aprsPassCode = "-1";
const char *aprsLocation = "4455.29N/02527.08E_";
const char *aprsPath = ">APRS,TCPIP*:";
const int   altMeters = 83; // 282
const long  altFeet = (long)(altMeters * 3.28084);
float altCorr = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));
// Reports and measurements
const int   aprsRprtHour = 15;  // Number of APRS reports per hour
const int   aprsMsrmMax = 3;    // Number of measurements per report (keep even)
int         aprsMsrmCount = 0;  // Measurements counter
int         aprsTlmSeq = 0;     // Telemetry sequence mumber
// Telemetry bits
char        aprsTlmBits = B00000000;
// The APRS connection client
EthernetClient APRS_Client;

// Statistics (median filter for the last 3 values)
int rmTemp[4];
int rmPres[4];
int rmA0[4];
int rmA1[4];
int rmA2[4];
int rmMCU[4];

// Sensors
const unsigned long snsDelay = 3600000UL / (aprsRprtHour * aprsMsrmMax);
unsigned long snsNextTime = 0UL;  // The next time to read the sensors
Adafruit_BMP280 atmo;             // The athmospheric sensor
bool atmo_ok = false;             // The athmospheric sensor flag

/*
  Simple median filter
  2014-03-25: started by David Cary
*/
int mdnOut(int *buf) {
  if (buf[0] < 3) return buf[3];
  else {
    int the_max = max(max(buf[1], buf[2]), buf[3]);
    int the_min = min(min(buf[1], buf[2]), buf[3]);
    // unnecessarily clever code
    int the_median = the_max ^ the_min ^ buf[1] ^ buf[2] ^ buf[3];
    return the_median;
  }
}

void mdnIn(int *buf, int x) {
  if (buf[0] < 3) buf[0]++;
  buf[1] = buf[2];
  buf[2] = buf[3];
  buf[3] = x;
}

void aprsSend(const char *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  APRS_Client.print(pkt);
}

void aprsSend(const __FlashStringHelper *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  APRS_Client.print(pkt);
}

void aprsSendCRLF() {
#ifdef DEBUG
  Serial.print(F("\r\n"));
#endif
  APRS_Client.print(F("\r\n"));
}

void aprsSendHeader(const char *sep) {
  aprsSend(aprsCallSign);
  aprsSend(aprsPath);
  aprsSend(sep);
}

/**
  Return time in APRS format: DDHHMMz
*/
char *aprsTime() {
  time_t moment = now();
  char buf[8];
  sprintf_P(buf, PSTR("%02d%02d%02dz"), day(moment), hour(moment), minute(moment));
  return buf;
}

/**
  Send APRS authentication data
  user FW0690 pass -1 vers WxUno 0.2"
*/
void aprsAuthenticate() {
  aprsSend(F("user "));
  aprsSend(aprsCallSign);
  aprsSend(F(" pass "));
  aprsSend(aprsPassCode);
  aprsSend(F(" vers "));
  aprsSend(NODENAME);
  aprsSend(F(" "));
  aprsSend(VERSION);
  aprsSendCRLF();
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxUno

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void aprsSendWeather(int temp, int hmdt, int pres, int lux) {
  aprsSendHeader("@");
  // Compose the APRS packet
  aprsSend(aprsTime());
  aprsSend(aprsLocation);
  // Wind
  aprsSend(F(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    char buf[5];
    sprintf_P(buf, PSTR("t%03d"), temp);
    aprsSend(buf);
  }
  else {
    aprsSend(F("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      aprsSend(F("h00"));
    }
    else {
      char buf[5];
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      aprsSend(buf);
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    char buf[7];
    sprintf_P(buf, PSTR("b%05d"), pres);
    aprsSend(buf);
  }
  // Illuminance, if valid
  if (lux >= 0) {
    char buf[5];
    sprintf_P(buf, PSTR("L%02d"), (int)(lux * 0.0079));
    aprsSend(buf);
  }
  // Comment (device name)
  aprsSend(NODENAME);
  aprsSendCRLF();
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
void aprsSendTelemetry(int a0, int a1, int a2, int a3, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup on power up (first minutes) or if the sequence number is 0
  if ((aprsTlmSeq == 0) or (millis() < snsDelay + snsDelay)) aprsSendTelemetrySetup();
  // Compose the APRS packet
  aprsSendHeader("T");
  char buf[40];
  sprintf_P(buf, PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, (int)(a0 / 4), (int)(a1 / 4), (int)(a2 / 4), a3, temp);
  aprsSend(buf);
  char bbuf[10];
  itoa(bits, bbuf, 2);
  aprsSend(bbuf);
  aprsSendCRLF();
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  char padCallSign[10];
  sprintf_P(padCallSign, PSTR("%-9s"), aprsCallSign);
  // Parameter names
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":PARM.LDR,Thrm,Vcc,A3,Temp,PROBE,ATMO,LUX,SAT,BAT,B6,B7,B8"));
  aprsSendCRLF();
  // Equations
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":EQNS.0,4,0,0,4,0,0,4,0,0,1,0,0,1,0"));
  aprsSendCRLF();
  // Units
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":UNIT.lux,C,V,N/A,C,prb,on,on,sat,low,N/A,N/A,N/A"));
  aprsSendCRLF();
  // Bit sense and project name
  aprsSendHeader(":");
  aprsSend(padCallSign);
  aprsSend(F(":BITS.10011111, "));
  aprsSend(NODENAME);
  aprsSend(F("/"));
  aprsSend(VERSION);
  aprsSendCRLF();
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>13:06 Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    aprsSendHeader(">");
    aprsSend(message);
    aprsSendCRLF();
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment) {
  // Compose the APRS packet
  aprsSendHeader("!");
  aprsSend(aprsLocation);
  aprsSend(F("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  aprsSend(buf);
  aprsSend(comment);
  aprsSendCRLF();
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

int getMCUTemp() {
  unsigned int wADC;
  float temp;

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
  temp = (wADC - 335.2) / 1.06154;

  // The returned temperature is in degrees Celsius.
  return temp;
}

void setup() {
  // Init the serial com
  Serial.println();
  Serial.begin(9600);
  Serial.println(NODENAME);
  Serial.println(__DATE__);

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

  // BMP280
  if (atmo.begin(0x76)) {
    atmo_ok = true;
    Serial.println(F("BMP280 sensor detected."));
  }
  else {
    atmo_ok = false;
    Serial.println(F("BMP280 sensor missing."));
  }

  // Debug
  //Serial.print(F("Reports per hour: "));
  //Serial.println(aprsRprtHour);
  //Serial.print(F("Measurement per report: "));
  //Serial.println(aprsMsrmMax);
  //Serial.print(F("Measurements delay: "));
  //Serial.println(snsDelay);

  // Initialize the random number generator and set the APRS telemetry start sequence
  if (timeStatus() != timeNotSet) randomSeed(now());
  aprsTlmSeq = random(1000);

  // Start the sensor timer
  snsNextTime = millis() + snsDelay;
}

void loop() {
  // Read the sensors and publish telemetry
  if (millis() >= snsNextTime) {
    // Check the DHCP lease
    Ethernet.maintain();
    // Count to check if we need to send the APRS data
    if (++aprsMsrmCount > aprsMsrmMax) aprsMsrmCount = 1;
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;

    // Read BMP280
    float temp, pres;
    if (atmo_ok) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      temp = atmo.readTemperature();
      pres = atmo.readPressure();
      // Median Filter
      mdnIn(rmTemp, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      mdnIn(rmPres, (int)(pres * altCorr / 10.0));  // Store directly sea level in dPa
    }

    // Various telemetry
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);
    int a2 = analogRead(A2);
    if (a2 < 512) {
      // Set the bit 3 to show the battery is low
      aprsTlmBits |= B00001000;
    }
    // Median Filter
    mdnIn(rmA0, a0);
    mdnIn(rmA1, a1);
    mdnIn(rmA2, a2);
    mdnIn(rmMCU, getMCUTemp());

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 1) {
      if (APRS_Client.connect(aprsServer, aprsPort)) {
        Serial.print(F("Connected to "));
        Serial.println(aprsServer);
        aprsAuthenticate();
        //aprsSendPosition(" WxUnoProbe");
        if (atmo_ok) aprsSendWeather(mdnOut(rmTemp), -1, mdnOut(rmPres), -1);
        //aprsSendWeather(rmTemp.out(), -1, -1, -1);
        aprsSendTelemetry(mdnOut(rmA0), mdnOut(rmA1), mdnOut(rmA2), 0, mdnOut(rmMCU), aprsTlmBits);
        //aprsSendStatus("Fine weather");
        //aprsSendTelemetrySetup();
        APRS_Client.stop();
        Serial.print(F("Disconnected."));
      }
      else Serial.println(F("Connection failed"));
    }

    // Repeat sensor reading
    snsNextTime += snsDelay;
  }
}
