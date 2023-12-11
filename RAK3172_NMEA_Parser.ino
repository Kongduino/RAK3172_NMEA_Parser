/*
  compiler.libraries.ldflags= -lsupc++ -lm -lc -lstdc++
*/

#undef max
#undef min
#include <string>
#include <vector>
#include <Wire.h>

#define GPS_Serial Serial1

using namespace std;

unsigned char MONGNSS[16] = {
  0xB5, 0x62, 0x0A, 0x28, 0x08, 0x00,
  0x01, 0x0F, 0x05, 0x05, 0x03, 0x00, 0x00, 0x00,
  0x57, 0x34
};

unsigned char CFGGNSS[44] = {
  /*
    0       U1  - msgVer  - Message version (=0 for this version)
    1       U1  - numTrkChHw  - Number of tracking channels available in hardware (read only)
    2       U1  - numTrkChUse - Number of tracking channels to use (<= numTrkChHw)
    3       U1  - numConfigBloc ks  - Number of configuration blocks following
    4 + 8*N U1  - gnssId  - GNSS identifier (see Satellite Numbering)
    5 + 8*N U1  - resTrkCh  - Number of reserved (minimum) tracking channels for this GNSS system
    6 + 8*N U1  - maxTrkCh  - Maximum number of tracking channels used for this GNSS system (>=resTrkChn)
    7 + 8*N U1  - reserved1 - Reserved
    8 + 8*N X4  - flags - bitfield of flags (see graphic below)
  */
  0xB5, 0x62, // 2/44. Header
  0x06, 0x3e, // 4/44. Class ID = CFG, Msg ID = UBX-CFG-GNSS
  0x24, 0x00, // 6/44. Payload Length = 36 bytes
  0x00, 0x16, 0x16, 0x04, // 10/44. msgVer=0, numTrkChHw=24, numTrkChUse=24, numConfigBlocks=4
  0x00, 0x08, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, // 18/44. gnssId=0 (GPS), resTrkCh=8, maxTrkCh=ff, reserved, ENABLE
  0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // 26/44. gnssId=1 (SBAS), resTrkCh=8, maxTrkCh=3, ENABLE
  0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, // 34/44. gnssId=6 (QZSS), resTrkCh=8, maxTrkCh=3, DISABLE
  0x06, 0x08, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, // 42/44. gnssId=6 (GLONASS), resTrkCh=8, maxTrkCh=ff, ENABLE
  0x00, 0x00 // 44/44. checksum
};

unsigned char MONVER[2] = {0x0A, 0x04};
unsigned char NAVX5[2] = {0x06, 0x23};
unsigned char CFGPRT[2] = {0x06, 0x00};
unsigned char CFGDAT[2] = {0x06, 0x06};
unsigned char CFGNMEA[2] = {0x06, 0x17};
unsigned char CFGRST[2] = {0x06, 0x04};

template class basic_string<char>; // https://github.com/esp8266/Arduino/issues/1136
// Required or the code won't compile!

#include <Arduino.h>
#include "Utilities.h"

uint8_t ix = 0;
vector<string> userStrings;
char UTC[7] = {0};
uint8_t SIV = 0;
float latitude = 0.0, longitude = 0.0;

float parseDegrees(const char *term) {
  float value = (float)(atof(term) / 100.0);
  uint16_t left = (uint16_t)value;
  value = (value - left) * 1.66666666666666;
  value += left;
  return value;
}

vector<string> parseNMEA(string nmea) {
  vector<string>result;
  if (nmea.at(0) != '$') {
    Serial.println("Not an NMEA sentence!");
    return result;
  }
  size_t lastFound = 0;
  size_t found = nmea.find(",", lastFound);
  while (found < nmea.size() && found != string::npos) {
    string token = nmea.substr(lastFound, found - lastFound);
    result.push_back(token);
    lastFound = found + 1;
    found = nmea.find(",", lastFound);
  }
  string token = nmea.substr(lastFound, found - lastFound);
  result.push_back(token);
  lastFound = found + 1;
  found = nmea.find(",", lastFound);
  return result;
}

void parseGPRMC(vector<string>result) {
  if (result.at(1) != "") {
    sprintf(buffer, "[%s] %s:%s:%s UTC", result.at(0).c_str(), result.at(1).substr(0, 2).c_str(), result.at(1).substr(2, 2).c_str(), result.at(1).substr(4, 2).c_str());
    Serial.println(buffer);
  }
  // if (result.at(2) == "V") Serial.println("Invalid fix!");
  // else Serial.println("Valid fix!");
  if (result.at(3) != "") {
    float newLatitude, newLongitude;
    newLatitude = parseDegrees(result.at(3).c_str());
    newLongitude = parseDegrees(result.at(5).c_str());
    float distance = haversine(latitude, longitude, newLatitude, newLongitude);
    if (distance > 20.0) {
      latitude = newLatitude;
      longitude = newLongitude;
      sprintf(buffer, "[%s] Coordinates: %3.8f %c, %3.8f %c\n", result.at(0).c_str(), latitude, result.at(4).c_str()[0], longitude, result.at(6).c_str()[0]);
      Serial.print(buffer);
    }
  }
}

void parseGPGGA(vector<string>result) {
  if (result.at(1) != "") {
    sprintf(buffer, "[%s] %s:%s:%s UTC", result.at(0).c_str(), result.at(1).substr(0, 2).c_str(), result.at(1).substr(2, 2).c_str(), result.at(1).substr(4, 2).c_str());
    Serial.println(buffer);
  }
  //  if (result.at(6) == "0") Serial.println("Invalid fix!");
  //  else Serial.println("Valid fix!");
  if (result.at(2) != "") {
    latitude = parseDegrees(result.at(2).c_str());
    longitude = parseDegrees(result.at(4).c_str());
    sprintf(buffer, "[%s] Coordinates: %3.8f %c, %3.8f %c\n", result.at(0).c_str(), latitude, result.at(3).c_str()[0], longitude, result.at(5).c_str()[0]);
    Serial.print(buffer);
  }
}

void parseGPZDA(vector<string>result) {
  if (result.at(1) != "") {
    sprintf(buffer, "[%s] Time: %s:%s:%s UTC", result.at(0).c_str(), result.at(1).substr(0, 2).c_str(), result.at(1).substr(2, 2).c_str(), result.at(1).substr(4, 2).c_str());
    Serial.println(buffer);
  }
  if (result.at(2) != "") {
    sprintf(buffer, "[%s] Date: %s/%s/%s UTC", result.at(0).c_str(), result.at(4).c_str(), result.at(3).c_str(), result.at(2));
    Serial.println(buffer);
  }
}

void parseGPGLL(vector<string>result) {
  if (result.at(1) != "") {
    latitude = parseDegrees(result.at(1).c_str());
    longitude = parseDegrees(result.at(3).c_str());
    sprintf(buffer, "[%s] Coordinates: %3.8f %c, %3.8f %c\n", result.at(0).c_str(), latitude, result.at(2).c_str()[0], longitude, result.at(4).c_str()[0]);
    Serial.print(buffer);
  }
}

void parseGPGSV(vector<string>result) {
  if (result.at(1) != "") {
    uint8_t newSIV = atoi(result.at(3).c_str());
    if (SIV != newSIV) {
      sprintf(buffer, "[%s] Message %s / %s. SIV: %s\n", result.at(0).c_str(), result.at(2).c_str(), result.at(1).c_str(), result.at(3).c_str());
      Serial.print(buffer);
      SIV = newSIV;
    }
  }
}

void parseGPTXT(vector<string>result) {
  //$GPTXT, 01, 01, 02, ANTSTATUS = INIT
  if (result.at(1) != "") {
    sprintf(buffer, " . Message %s / %s. Severity: %s\n . Message text: %s\n",
            result.at(2).c_str(), result.at(1).c_str(), result.at(3).c_str(), result.at(4).c_str()); // , result.at(5).c_str()
    Serial.print(buffer);
  }
}

void parseGPVTG(vector<string>result) {
  Serial.println("Track Made Good and Ground Speed.");
  if (result.at(1) != "") {
    sprintf(buffer, " . True track made good %s [%s].\n", result.at(1).c_str(), result.at(2).c_str());
    Serial.print(buffer);
  }
  if (result.at(3) != "") {
    sprintf(buffer, " . Magnetic track made good %s [%s].\n", result.at(3).c_str(), result.at(4).c_str());
    Serial.print(buffer);
  }
  if (result.at(5) != "") {
    sprintf(buffer, " . Speed: %s %s.\n", result.at(5).c_str(), result.at(6).c_str());
    Serial.print(buffer);
  }
  if (result.at(7) != "") {
    sprintf(buffer, " . Speed: %s %s.\n", result.at(7).c_str(), result.at(8).c_str());
    Serial.print(buffer);
  }
}

void parseGPGSA(vector<string>result) {
  // $GPGSA,A,3,15,29,23,,,,,,,,,,12.56,11.96,3.81
  Serial.println("GPS DOP and active satellites");
  if (result.at(1) == "A") Serial.println(" . Mode: Automatic");
  else if (result.at(1) == "M") Serial.println(" . Mode: Manual");
  else Serial.println(" . Mode: ???");
  if (result.at(2) == "1") {
    Serial.println(" . Fix not available.");
    return;
  } else if (result.at(2) == "2") Serial.println(" . Fix: 2D");
  else if (result.at(2) == "3") Serial.println(" . Fix: 3D");
  else {
    Serial.println(" . Fix: ???");
    return;
  }
  Serial.print(" . PDOP: "); Serial.println(result.at(result.size() - 3).c_str());
  Serial.print(" . HDOP: "); Serial.println(result.at(result.size() - 2).c_str());
  Serial.print(" . VDOP: "); Serial.println(result.at(result.size() - 1).c_str());
}

void setup() {
  Serial.begin(115200);
  initCommands();
  Serial.println("\nGPS Example (NMEA Parser)");
  Serial.println("Turning off port");
  pinMode(WB_IO2, OUTPUT); // SLOT_A SLOT_B
  digitalWrite(WB_IO2, 0);
  delay(100);
  Serial.println("Turning on port");
  digitalWrite(WB_IO2, 1);
  delay(100);
  GPS_Serial.begin(9600);
  delay(100);
  Serial.println("GPS_Serial ready!");
  delay(1000);
  SendCmd0B(MONVER[0], MONVER[1], (char*)"MON-VER");
}

bool waitForDollar = true;

bool isItB562() {
  char c = GPS_Serial.read();
  if (c == 0x62) {
    memset(B562, 0, 256);
    uint8_t posx = 0, i;
    B562[posx++] = GPS_Serial.read(); // Class
    B562[posx++] = GPS_Serial.read(); // ID
    B562[posx++] = GPS_Serial.read(); // Length L
    B562[posx++] = GPS_Serial.read(); // Length H
    uint16_t len = B562[posx - 2];
    sprintf(infoText, "B562 incoming: %2x %2x payload length: %d\n", B562[0], B562[1], len);
    if (len > 0)
      for (i = 0; i < len; i++) B562[posx++] = GPS_Serial.read();
    B562[posx++] = GPS_Serial.read(); // CK_A
    B562[posx++] = GPS_Serial.read(); // CK_B
    Serial.println("B562:");
    hexDump((unsigned char*)B562, posx);
    if (B562[0] == 0x05) handleACK();
    if (B562[0] == MONGNSS[0] && B562[1] == MONGNSS[1]) handleMONGNSS();
    if (B562[0] == CFGDAT[0] && B562[1] == CFGDAT[1]) F0x060x06();
    return true;
  }
  return false;
}

void loop() {
  if (Serial.available()) {
    char incoming[256];
    uint8_t posx = 0;
    char c = Serial.read();
    while (Serial.available()) {
      if (c > 31) incoming[posx++] = c;
      else if (c == 13 || c == 10) {
        while (Serial.available()) c = Serial.read();
        break;
      }
      c = Serial.read();
    }
    incoming[posx] = 0;
    if (incoming[0] == '/') {
      // / command
      Serial.print("> ");
      Serial.println(incoming + 1);
      std::map<string, void (*)()>::iterator it;
      it = commands.find(string(incoming));
      if (it == commands.end()) {
        Serial.println("Not a known command!");
        showHelp();
      } else {
        it->second();
      }
    } else {
      Serial.println("Not a command!");
    }
  }

  if (GPS_Serial.available()) {
    char c = GPS_Serial.read();
    if (c == 0xB5) {
      isItB562();
      return;
    }
    if (waitForDollar && c == '$') {
      waitForDollar = false;
      buffer[0] = '$';
      ix = 1;
    } else if (waitForDollar == false) {
      if (c == 13) {
        buffer[ix] = 0;
        c = GPS_Serial.read();
        delay(50);
        string nextLine = string(buffer);
        userStrings.push_back(nextLine.substr(0, nextLine.size() - 3));
        waitForDollar = true;
      } else if (c == 0xB5) {
        if (isItB562()) return;
      } else {
        buffer[ix++] = c;
      }
    }
  }
  if (userStrings.size() > 0) {
    string nextLine = userStrings[0];
    userStrings.erase(userStrings.begin());
    if (nextLine.substr(0, 1) != "$") {
      // Serial.print("Not an NMEA string!\n>> ");
      // Serial.println(nextLine.c_str());
      return;
    } else {
      vector<string>result = parseNMEA(nextLine);
      if (result.size() == 0) return;
      string verb = result.at(0);
      if (verb.substr(3, 3) == "RMC") {
        parseGPRMC(result);
      } else if (verb.substr(3, 3) == "GSV") {
        parseGPGSV(result);
      } else if (verb.substr(3, 3) == "GGA") {
        parseGPGGA(result);
      } else if (verb.substr(3, 3) == "GLL") {
        parseGPGLL(result);
      } else if (verb.substr(3, 3) == "GSA") {
        //parseGPGSA(result);
      } else if (verb.substr(3, 3) == "VTG") {
        //parseGPVTG(result);
      } else if (verb.substr(3, 3) == "ZDA") {
        parseGPZDA(result);
      } else if (verb.substr(3, 3) == "TXT") {
        parseGPTXT(result);
      } else {
        Serial.println(nextLine.c_str());
      }
    }
  }
}
