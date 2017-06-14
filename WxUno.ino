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


  Ethernet connected weather station, reading the temperature and athmospheric
  pressure sensor BMP280, as well as internal temperature, supply voltage,
  local illuminance, publishing the measured data to CWOP APRS.
*/

// The DEBUG and DEVEL flag
#define DEBUG
#define DEVEL

// Watchdog, sleep
#include <avr/wdt.h>
#include <avr/sleep.h>

// EEPROM and CRC32
#include <EEPROM.h>
#include <CRC32.h>

// The sensors are connected to I2C
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>

// Ethernet
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Device name and software version
const char NODENAME[] PROGMEM = "WxUno";
const char VERSION[]  PROGMEM = "3.1";
bool       PROBE              = true;                   // True if the station is being probed

// APRS parameters
const char  aprsServer[] PROGMEM  = "cwop5.aprs.net";   // CWOP APRS-IS server address to connect to
const int   aprsPort              = 14580;              // CWOP APRS-IS port
#ifdef DEVEL
const int   altMeters             = 83;                 // Altitude in Bucharest
#else
const int   altMeters             = 282;                // Altitude in Targoviste
#endif
const long  altFeet = (long)(altMeters * 3.28084);                                    // Altitude in feet
const float altCorr = pow((float)(1.0 - 2.25577e-5 * altMeters), (float)(-5.25578));  // Altitude correction for QNH

const char aprsCallSign[] PROGMEM = "FW0727";
const char aprsPassCode[] PROGMEM = "-1";
const char aprsPath[]     PROGMEM = ">APRS,TCPIP*:";
const char aprsLocation[] PROGMEM = "4455.29N/02527.08E_";
const char aprsTlmPARM[]  PROGMEM = ":PARM.Light,Soil,RSSI,Vcc,Tmp,PROBE,ATMO,LUX,SAT,BAT,TM,RB,B8";
const char aprsTlmEQNS[]  PROGMEM = ":EQNS.0,20,0,0,20,0,0,-1,0,0,0.004,4.5,0,1,-100";
const char aprsTlmUNIT[]  PROGMEM = ":UNIT.mV,mV,dBm,V,C,prb,on,on,sat,low,err,N/A,N/A";
const char aprsTlmBITS[]  PROGMEM = ":BITS.10011111, ";
const char eol[]          PROGMEM = "\r\n";

char       aprsPkt[100]           = "";     // The APRS packet buffer, largest packet is 82 for v2.1

// Time synchronization and keeping
const char    timeServer[] PROGMEM  = "utcnist.colorado.edu";  // Time server address to connect to (RFC868)
const int     timePort              = 37;                      // Time server port
unsigned long timeNextSync          = 0UL;                     // Next time to syncronize
unsigned long timeDelta             = 0UL;                     // Difference between real time and internal clock
bool          timeOk                = false;                   // Flag to know the time is accurate
const int     timeZone              = 0;                       // Time zone
const int     eeTime                = 0;                       // EEPROM address for storing last known good time

// Reports and measurements
const int aprsRprtHour   = 10; // Number of APRS reports per hour
const int aprsMsrmMax    = 3;  // Number of measurements per report (keep even)
int       aprsMsrmCount  = 0;  // Measurements counter
int       aprsTlmSeq     = 0;  // Telemetry sequence mumber

// Telemetry bits
char      aprsTlmBits    = B00000000;

// Ethernet
byte ethMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// Set the static IP address to use if the DHCP fails to assign
IPAddress   ethIP(10, 200, 4, 99);
IPAddress  ethDNS(10, 200, 4, 250);
IPAddress   ethGW(10, 200, 4, 250);
IPAddress  ethMSK(10, 200, 4, 255);
bool       ethDHCP = false;

// The APRS connection client
EthernetClient  ethClient;
unsigned long   linkLastTime = 0UL;             // Last connection time

// When ADC completed, take an interrupt
EMPTY_INTERRUPT(ADC_vect);

// Statistics (round median filter for the last 3 values)
enum      rMedIdx {MD_TEMP, MD_PRES, MD_RSSI, MD_SRAD, MD_VCC, MD_A0, MD_A1, MD_ALL};
int       rMed[MD_ALL][4];
const int eeRMed = 16; // EEPROM address for storing the round median array

// Sensors
const unsigned long snsReadTime = 30UL * 1000UL;                          // Total time to read sensors, repeatedly, for aprsMsrmMax times
const unsigned long snsDelayBfr = 3600000UL / aprsRprtHour - snsReadTime; // Delay before sensor readings
const unsigned long snsDelayBtw = snsReadTime / aprsMsrmMax;              // Delay between sensor readings
unsigned long       snsNextTime = 0UL;                                    // Next time to read the sensors
Adafruit_BMP280     atmo;                                                 // The athmospheric sensor
bool                atmo_ok = false;                                      // The athmospheric sensor presence flag
BH1750              light(0x23);                                          // The illuminance sensor
bool                light_ok = false;                                     // The illuminance sensor presence flag

// NTP
const char  ntpServer[] PROGMEM = "europe.pool.ntp.org";  // NTP server to connect to (RFC5905)
const int   ntpPort = 123;                                // NTP port
EthernetUDP ethNTP;                                       // NTP UDP client

/**
  Simple median filter: get the median
  2014-03-25: started by David Cary

  @param idx the index in round median array
  @return the median
*/
int rMedOut(int idx) {
  // Return the last value if the buffer is not full yet
  if (rMed[idx][0] < 3) return rMed[idx][3];
  else {
    // Get the maximum and the minimum
    int the_max = max(max(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    int the_min = min(min(rMed[idx][1], rMed[idx][2]), rMed[idx][3]);
    // Clever code: XOR the max and min, remaining the middle
    return the_max ^ the_min ^ rMed[idx][1] ^ rMed[idx][2] ^ rMed[idx][3];
  }
}

/**
  Simple median filter: add value to array

  @param idx the index in round median array
  @param x the value to add
*/
void rMedIn(int idx, int x) {
  // At index 0 there is the number of values stored
  if (rMed[idx][0] < 3) rMed[idx][0]++;
  // Shift one position
  rMed[idx][1] = rMed[idx][2];
  rMed[idx][2] = rMed[idx][3];
  rMed[idx][3] = x;
}

/**
  Write the time to EEPROM, along with CRC32: 8 bytes

  @param tm the time value to store
*/
void timeEEWrite(unsigned long utm) {
  // Compute CRC32 checksum
  CRC32 crc32;
  crc32.update(&utm, sizeof(utm));
  unsigned long crc = crc32.finalize();
  // Write the data
  EEPROM.put(eeTime, utm);
  EEPROM.put(eeTime + sizeof(utm), crc);
}

/**
  Read the time from EEPROM, along with CRC32 and verify
*/
unsigned long timeEERead() {
  unsigned long utm, eck;
  // Read the data
  EEPROM.get(eeTime, utm);
  EEPROM.get(eeTime + sizeof(utm), eck);
  // Compute CRC32 checksum
  CRC32 crc32;
  crc32.update(&utm, sizeof(utm));
  unsigned long crc = crc32.finalize();
  // Verify
  if (eck == crc) return utm;
  else            return 0UL;
}

/**
  Get current time as UNIX time (1970 epoch)

  @param sync flag to show whether network sync is to be performed
  @return current UNIX time
*/
unsigned long timeUNIX(bool sync = true) {
  // Check if we need to sync
  if (millis() >= timeNextSync and sync) {
    // Try to get the time from Internet
    unsigned long utm = ntpSync(); //timeSync();
    if (utm == 0) {
      // Time sync has failed, sync again over one minute
      timeNextSync += 1UL * 60 * 1000;
      timeOk = false;
      // Try to get old time from eeprom, if time delta is zero
      if (timeDelta == 0) {
        // Compute an approximate time delta, if time is valid
        utm = timeEERead();
        if (utm != 0) {
          timeDelta = utm - (millis() / 1000);
          Serial.print(F("Time sync error, using EEPROM: 0x"));
          Serial.println(utm, 16);
        }
        else Serial.println(F("Time sync error, invalid EEPROM"));
      }
    }
    else {
      // Compute the new time delta
      timeDelta = utm - (millis() / 1000);
      // Time sync has succeeded, sync again in 8 hours
      timeNextSync += 8UL * 60 * 60 * 1000;
      timeOk = true;
      // Store this known time
      timeEEWrite(utm);
      Serial.print(F("Network UNIX Time: 0x"));
      Serial.println(utm, 16);
    }
  }

  // Get current time based on uptime and time delta,
  // or just uptime for no time sync ever
  return (millis() / 1000) + timeDelta;
}

/**
  Connect to a time server using the RFC 868 time protocol

  @return UNIX time from server
*/
unsigned long timeSync() {
  union {
    uint32_t t = 0UL;
    uint8_t  b[4];
  } uxtm;

  // Try to connect
  int bytes = sizeof(uxtm.b);
  char timeServerBuf[strlen_P((char*)timeServer) + 1];
  strncpy_P(timeServerBuf, (char*)timeServer, sizeof(timeServerBuf));
  if (ethClient.connect(timeServerBuf, timePort)) {
    // Read network time during 5 seconds
    unsigned long timeout = millis() + 5000UL;
    while (millis() <= timeout and bytes != 0) {
      char b = ethClient.read();
      if (b != -1) uxtm.b[--bytes] = uint8_t(b);
    }
    ethClient.stop();
    // Keep the millis the connection worked
    linkLastTime = millis();
  }

  // Convert 1900 epoch to 1970 Unix time, if read data is valid
  if (!bytes) return (unsigned long)uxtm.t - 2208988800UL;
  else        return 0UL;
}

/**
  © Francesco Potortì 2013 - GPLv3 - Revision: 1.13

  Send an NTP packet and wait for the response, return the Unix time

  To lower the memory footprint, no buffers are allocated for sending
  and receiving the NTP packets.  Four bytes of memory are allocated
  for transmision, the rest is random garbage collected from the data
  memory segment, and the received packet is read one byte at a time.
  The Unix time is returned, that is, seconds from 1970-01-01T00:00.
*/
unsigned long ntpSync() {
  // Open socket on arbitrary port
  bool ntpOk = ethNTP.begin(12321);
  // NTP request header: Only the first four bytes of an outgoing
  // packet need to be set appropriately, the rest can be whatever.
  const long ntpFirstFourBytes = 0xEC0600E3;
  // Fail if UDP could not init a socket
  if (!ntpOk) return 0UL;
  // Clear received data from possible stray received packets
  ethNTP.flush();
  // Send an NTP request
  char ntpServerBuf[strlen_P((char*)ntpServer) + 1];
  strncpy_P(ntpServerBuf, (char*)ntpServer, sizeof(ntpServerBuf));
  if (!(ethNTP.beginPacket(ntpServerBuf, ntpPort) &&
        ethNTP.write((byte *)&ntpFirstFourBytes, 48) == 48 &&
        ethNTP.endPacket()))
    return 0UL;                         // sending request failed
  // Wait for response; check every pollIntv ms up to maxPoll times
  const int pollIntv = 150;             // poll every this many ms
  const byte maxPoll = 15;              // poll up to this many times
  int pktLen;                           // received packet length
  for (byte i = 0; i < maxPoll; i++) {
    if ((pktLen = ethNTP.parsePacket()) == 48) break;
    delay(pollIntv);
  }
  if (pktLen != 48) return 0UL;         // no correct packet received
  // Read and discard the first useless bytes (32 for speed, 40 for accuracy)
  for (byte i = 0; i < 40; ++i) ethNTP.read();
  // Read the integer part of sending time
  unsigned long ntpTime = ethNTP.read();  // NTP time
  for (byte i = 1; i < 4; i++)
    ntpTime = ntpTime << 8 | ethNTP.read();
  // Round to the nearest second if we want accuracy
  // The fractionary part is the next byte divided by 256: if it is
  // greater than 500ms we round to the next second; we also account
  // for an assumed network delay of 50ms, and (0.5-0.05)*256=115;
  // additionally, we account for how much we delayed reading the packet
  // since its arrival, which we assume on average to be pollIntv/2.
  ntpTime += (ethNTP.read() > 115 - pollIntv / 8);
  // Discard the rest of the packet
  ethNTP.flush();
  return ntpTime - 2208988800UL;          // convert to Unix time
}

/**
  Send an APRS packet and, eventuall, print it to serial line

  @param *pkt the packet to send
*/
void aprsSend(const char *pkt) {
#ifdef DEBUG
  Serial.print(pkt);
#endif
  ethClient.print(pkt);
}

/**
  Return time in zulu APRS format: HHMMSSh

  @param *buf the buffer to return the time to
  @param len the buffer length
*/
char aprsTime(char *buf, size_t len) {
  // Get the time, but do not open a connection to server
  unsigned long utm = timeUNIX(false);
  // Compute hour, minute and second
  int hh = (utm % 86400L) / 3600;
  int mm = (utm % 3600) / 60;
  int ss =  utm % 60;
  // Return the formatted time
  snprintf_P(buf, len, PSTR("%02d%02d%02dh"), hh, mm, ss);
}

/**
  Send APRS authentication data
  user FW0727 pass -1 vers WxUno 3.1"
*/
void aprsAuthenticate() {
  strcpy_P(aprsPkt, PSTR("user "));
  strcat_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, PSTR(" pass "));
  strcat_P(aprsPkt, aprsPassCode);
  strcat_P(aprsPkt, PSTR(" vers "));
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, PSTR(" "));
  strcat_P(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS weather data, then try to get the forecast
  FW0690>APRS,TCPIP*:@152457h4427.67N/02608.03E_.../...g...t044h86b10201L001WxUno
  #ifdef DEBUG
  Serial.print(pkt);
  #endif

  @param temp temperature
  @param hmdt humidity
  @param pres athmospheric pressure
  @param lux illuminance
*/
void aprsSendWeather(int temp, int hmdt, int pres, int lux) {
  char buf[8];
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("@"));
  aprsTime(buf, sizeof(buf));
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, aprsLocation);
  // Wind (unavailable)
  strcat_P(aprsPkt, PSTR(".../...g..."));
  // Temperature
  if (temp >= -460) { // 0K in F
    sprintf_P(buf, PSTR("t%03d"), temp);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  else {
    strcat_P(aprsPkt, PSTR("t..."));
  }
  // Humidity
  if (hmdt >= 0) {
    if (hmdt == 100) {
      strcat_P(aprsPkt, PSTR("h00"));
    }
    else {
      sprintf_P(buf, PSTR("h%02d"), hmdt);
      strncat(aprsPkt, buf, sizeof(buf));
    }
  }
  // Athmospheric pressure
  if (pres >= 0) {
    sprintf_P(buf, PSTR("b%05d"), pres);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Illuminance, if valid
  if (lux >= 0 and lux <= 999) {
    sprintf_P(buf, PSTR("L%03d"), lux);
    strncat(aprsPkt, buf, sizeof(buf));
  }
  // Comment (device name)
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry and, periodically, send the telemetry setup
  FW0690>APRS,TCPIP*:T#517,173,062,213,002,000,00000000

  @param a0 read analog A0
  @param a1 read analog A1
  @param rssi GSM RSSI level
  @param vcc voltage
  @param temp internal temperature
  @param bits digital inputs
*/
void aprsSendTelemetry(int a0, int a1, int rssi, int vcc, int temp, byte bits) {
  // Increment the telemetry sequence number, reset it if exceeds 999
  if (++aprsTlmSeq > 999) aprsTlmSeq = 0;
  // Send the telemetry setup if the sequence number is 0
  if (aprsTlmSeq == 0) aprsSendTelemetrySetup();
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("T"));
  char buf[40];
  snprintf_P(buf, sizeof(buf), PSTR("#%03d,%03d,%03d,%03d,%03d,%03d,"), aprsTlmSeq, a0, a1, rssi, vcc, temp);
  strncat(aprsPkt, buf, sizeof(buf));
  itoa(bits, buf, 2);
  strncat(aprsPkt, buf, sizeof(buf));
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS telemetry setup
*/
void aprsSendTelemetrySetup() {
  char padCallSign[10];
  strcpy_P(padCallSign, aprsCallSign);  // Workaround
  sprintf_P(padCallSign, PSTR("%-9s"), padCallSign);
  // Parameter names
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmPARM);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Equations
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmEQNS);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Units
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmUNIT);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
  // Bit sense and project name
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR(":"));
  strncat(aprsPkt, padCallSign, sizeof(padCallSign));
  strcat_P(aprsPkt, aprsTlmBITS);
  strcat_P(aprsPkt, NODENAME);
  strcat_P(aprsPkt, PSTR("/"));
  strcat_P(aprsPkt, VERSION);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Send APRS status
  FW0690>APRS,TCPIP*:>Fine weather

  @param message the status message to send
*/
void aprsSendStatus(const char *message) {
  // Send only if the message is not empty
  if (message[0] != '\0') {
    // Send the APRS packet
    strcpy_P(aprsPkt, aprsCallSign);
    strcat_P(aprsPkt, aprsPath);
    strcat_P(aprsPkt, PSTR(">"));
    strcat(aprsPkt, message);
    strcat_P(aprsPkt, eol);
    aprsSend(aprsPkt);
  }
}

/**
  Send APRS position and altitude
  FW0690>APRS,TCPIP*:!DDMM.hhN/DDDMM.hhW$comments

  @param comment the comment to append
*/
void aprsSendPosition(const char *comment = NULL) {
  // Compose the APRS packet
  strcpy_P(aprsPkt, aprsCallSign);
  strcat_P(aprsPkt, aprsPath);
  strcat_P(aprsPkt, PSTR("!"));
  strcat_P(aprsPkt, aprsLocation);
  strcat_P(aprsPkt, PSTR("/000/000/A="));
  char buf[7];
  sprintf_P(buf, PSTR("%06d"), altFeet);
  strncat(aprsPkt, buf, sizeof(buf));
  if (comment != NULL) strcat(aprsPkt, comment);
  strcat_P(aprsPkt, eol);
  aprsSend(aprsPkt);
}

/**
  Read the analog pin after a delay, while sleeping, using interrupt

  @param pin the analog pin
  @return raw analog read value
*/
int readAnalog(uint8_t pin) {
  // Allow for channel or pin numbers
  if (pin >= 14) pin -= 14;

  // Set the analog reference to DEFAULT, select the channel (low 4 bits).
  // This also sets ADLAR (left-adjust result) to 0 (the default).
  ADMUX = _BV(REFS0) | (pin & 0x07);
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // The returned reading
  return (int)(wADC);
}

/**
  Read the internal MCU temperature
  The internal temperature has to be used with the internal reference of 1.1V.
  Channel 8 can not be selected with the analogRead function yet.

  @return temperature in hundredth of degrees Celsius, *calibrated for my device*
*/
int readMCUTemp() {
  // Set the internal reference and mux.
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // The returned temperature is in hundreds degrees Celsius; calibrated
  return (int)(84.87 * wADC - 25840);
}

/*
  Read the power supply voltage, by measuring the internal 1V1 reference

  @return voltage in millivolts, *calibrated for my device*
*/
int readVcc() {
  // Set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRA |= _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2);  // prescaler of 128
  ADCSRA |= _BV(ADEN);  // enable the ADC
  ADCSRA |= _BV(ADIE);  // enable intterupt

  // Wait for voltage to settle
  delay(10);
  // Take an ADC reading in sleep mode
  noInterrupts();
  // Start conversion
  ADCSRA |= _BV(ADSC);
  set_sleep_mode(SLEEP_MODE_ADC);
  interrupts();

  // Awake again, reading should be done, but better make sure
  // maybe the timer interrupt fired
  while (bit_is_set(ADCSRA, ADSC));

  // Reading register "ADCW" takes care of how to read ADCL and ADCH.
  long wADC = ADCW;

  // Return Vcc in mV; 1125300 = 1.1 * 1024 * 1000
  // 1.1V calibration: 1.074
  return (int)(1099776UL / wADC);
}

/**
  Software reset the MCU
  (c) Mircea Diaconescu http://web-engineering.info/node/29
*/
void softReset(uint8_t prescaller) {
  Serial.print(F("Reboot"));
  // Start watchdog with the provided prescaller
  wdt_enable(prescaller);
  // Wait for the prescaller time to expire
  // without sending the reset signal by using
  // the wdt_reset() method
  while (true) {
    Serial.print(F("."));
    delay(1000);
  }
}

/**
  Check if the link failed for too long (3600 / aprsRprtHour) and reset
*/
void linkFailed() {
  if (millis() >= linkLastTime + 3600000UL / aprsRprtHour) {
    Serial.println(F("Connection failed for the last reports"));
    // If time is good, store it
    if (timeOk) timeEEWrite(timeUNIX(false));
    // Reset the MCU (in 8s)
    softReset(WDTO_8S);
  }
}

/**
  Print a character array from program memory
*/
void print_P(const char *str) {
  uint8_t val;
  do {
    val = pgm_read_byte(str++);
    if (val) Serial.write(val);
  } while (val);
}

/**
  Main Arduino setup function
*/
void setup() {
  // Init the serial com
  Serial.begin(9600);
  Serial.println();
  print_P(NODENAME);
  Serial.print(F(" "));
  print_P(VERSION);
  Serial.print(F(" "));
  Serial.println(__DATE__);

  // Start the Ethernet connection:
  if (ethDHCP = Ethernet.begin(ethMAC) == 0) {
    Serial.println(F("Failed to configure Ethernet using DHCP"));
    Ethernet.begin(ethMAC, ethIP, ethDNS, ethGW, ethMSK);
  }
  // Print the Ethernet shield's IP address:
  Serial.print(F("IP address: "));
  Serial.println(Ethernet.localIP());
  // Give the Ethernet shield a second to initialize:
  delay(1000);

  // Start time sync
  timeUNIX();

  // BMP280
  atmo_ok = atmo.begin(0x76);
  if (atmo_ok) Serial.println(F("BMP280 sensor detected"));
  else         Serial.println(F("BMP280 sensor missing"));

  // BH1750
  light.begin(BH1750_CONTINUOUS_HIGH_RES_MODE);
  uint16_t lux = light.readLightLevel();
  light_ok = true;

  // Hardware data
  int hwTemp = readMCUTemp();
  int hwVcc  = readVcc();
  Serial.print(F("Temp: "));
  Serial.println((float)hwTemp / 100, 2);
  Serial.print(F("Vcc : "));
  Serial.println((float)hwVcc / 1000, 3);

  // Initialize the random number generator and set the APRS telemetry start sequence
  randomSeed(hwTemp + timeUNIX(false) + hwVcc + millis());
  aprsTlmSeq = random(1000);
  Serial.print(F("TLM : "));
  Serial.println(aprsTlmSeq);

  // Start the sensor timer
  snsNextTime = millis();

  // Enable the watchdog
  wdt_enable(WDTO_8S);
}

/**
  Main Arduino loop
*/
void loop() {
  // Read the sensors
  if (millis() >= snsNextTime) {
    // Check the DHCP lease, if using DHCP
    if (ethDHCP) Ethernet.maintain();
    // Count to check if we need to send the APRS data
    if (++aprsMsrmCount >= aprsMsrmMax) {
      // Restart the counter
      aprsMsrmCount = 0;
      // Repeat sensor reading after the 'before' delay (long)
      snsNextTime += snsDelayBfr;
    }
    else {
      // Repeat sensor reading after the 'between' delay (short)
      snsNextTime += snsDelayBtw;
    }
    // Set the telemetry bit 7 if the station is being probed
    if (PROBE) aprsTlmBits = B10000000;

    // Check the time and set the telemetry bit 2 if time is not accurate
    unsigned long utm = timeUNIX();
    if (!timeOk) aprsTlmBits |= B00000100;

    // Set the telemetry bit 1 if the uptime is less than one day (recent reboot)
    if (millis() < 86400000UL) aprsTlmBits |= B00000010;

    // Read BMP280
    float temp, pres;
    if (atmo_ok) {
      // Set the bit 5 to show the sensor is present (reverse)
      aprsTlmBits |= B01000000;
      // Get the weather parameters
      temp = atmo.readTemperature();
      pres = atmo.readPressure();
      // Add to the round median filter
      rMedIn(MD_TEMP, (int)(temp * 9 / 5 + 32));      // Store directly integer Fahrenheit
      rMedIn(MD_PRES, (int)(pres * altCorr / 10.0));  // Store directly sea level in dPa
    }

    // Read BH1750, illuminance value in lux
    uint16_t lux = light.readLightLevel();
    // Calculate the solar radiation in W/m^2
    // FIXME this is in cW/m^2
    int solRad = (int)(lux * 0.79);
    // Set the bit 5 to show the sensor is present (reverse) and there is any light
    if (solRad > 0) aprsTlmBits |= B00100000;
    // Set the bit 4 to show the sensor is saturated
    if (solRad > 999) aprsTlmBits |= B00010000;
    // Add to round median filter
    rMedIn(MD_SRAD, solRad);

    // Read Vcc (mV) and add to the round median filter
    int vcc = readVcc();
    rMedIn(MD_VCC, vcc);
    if (vcc < 4750 or vcc > 5250) {
      // Set the bit 3 to show the USB voltage is wrong (5V +/- 5%)
      aprsTlmBits |= B00001000;
    }

    // Various analog telemetry
    int a0 = readAnalog(A0);
    int a1 = readAnalog(A1);

    // Add to round median filter, mV (a / 1024 * Vcc)
    rMedIn(MD_A0, (vcc * (unsigned long)a0) / 1024);
    rMedIn(MD_A1, (vcc * (unsigned long)a1) / 1024);

    // Upper part
    // 500 / R(kO); R = R0(1024 / x - 1)
    // Lower part
    // Vout=RawADC0*0.0048828125;
    // lux=(2500/Vout-500)/10;
    //int lux = 51150L / a0 - 50;

    // APRS (after the first 3600/(aprsMsrmMax*aprsRprtHour) seconds,
    //       then every 60/aprsRprtHour minutes)
    if (aprsMsrmCount == 0) {
      // Reset the watchdog
      wdt_reset();
      // Get RSSI (will get FALSE (0) if the modem is not working)
      // FIXME
      int rssi = 0;
      if (rssi) rMedIn(MD_RSSI, -rssi);
      // Connect to APRS server
      char aprsServerBuf[strlen_P((char*)aprsServer) + 1];
      strncpy_P(aprsServerBuf, (char*)aprsServer, sizeof(aprsServerBuf));
      if (ethClient.connect(aprsServerBuf, aprsPort)) {
        // Reset the watchdog
        wdt_reset();
        // Authentication
        aprsAuthenticate();
        // Send the position, altitude and comment in firsts minutes after boot
        if (millis() < snsDelayBfr) aprsSendPosition();
        // Send weather data if the athmospheric sensor is present
        if (atmo_ok) aprsSendWeather(rMedOut(MD_TEMP), -1, rMedOut(MD_PRES), rMedOut(MD_SRAD));
        // Send the telemetry
        aprsSendTelemetry(rMedOut(MD_A0) / 20,
                          rMedOut(MD_A1) / 20,
                          rMedOut(MD_RSSI),
                          (rMedOut(MD_VCC) - 4500) / 4,
                          readMCUTemp() / 100 + 100,
                          aprsTlmBits);
        //aprsSendStatus("Fine weather");
        // Close the connection
        ethClient.stop();
        // Keep the millis the connection worked
        linkLastTime = millis();
      }
      else linkFailed();
    }
  }

  // Reset the watchdog
  wdt_reset();
}
