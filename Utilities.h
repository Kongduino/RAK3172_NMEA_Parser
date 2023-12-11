#include <stdio.h>
#include <map>
using namespace std;

void hexDump(unsigned char *, uint16_t);
void Fletcher(uint8_t *, uint8_t);
void handleACK();
void handleMONGNSS();
void sendMONVER();
void sendNAVX5();
void sendCFGPRT();
void sendCFGDAT();
void sendBDSGNSS();
void askBDSGNSS();
void sendMONGNSS();
void sendCFGNMEA();
void showHelp();

char buffer[256];
char B562[256];
char infoText[128];

std::map<string, void (*)()> commands;

#define GPSSup 1
#define GlonassSup 2
#define BeidouSup 4
#define GalileoSup 8

/**@brief Pretty-prints a buffer in hexadecimal, 16 bytes a line
          wth ASCII representation on the right àla hexdump -C
*/
void hexDump(unsigned char *buf, uint16_t len) {
  char alphabet[17] = "0123456789abcdef";
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
  Serial.print(F("   |.0 .1 .2 .3 .4 .5 .6 .7 .8 .9 .a .b .c .d .e .f | |      ASCII     |\n"));
  for (uint16_t i = 0; i < len; i += 16) {
    if (i % 128 == 0)
      Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
    char s[] = "|                                                | |                |\n";
    uint8_t ix = 1, iy = 52;
    for (uint8_t j = 0; j < 16; j++) {
      if (i + j < len) {
        uint8_t c = buf[i + j];
        s[ix++] = alphabet[(c >> 4) & 0x0F];
        s[ix++] = alphabet[c & 0x0F];
        ix++;
        if (c > 31 && c < 128) s[iy++] = c;
        else s[iy++] = '.';
      }
    }
    uint8_t index = i / 16;
    if (i < 256) Serial.write(' ');
    Serial.print(index, HEX); Serial.write('.');
    Serial.print(s);
  }
  Serial.print(F("   +------------------------------------------------+ +----------------+\n"));
}

void initCommands() {
  commands["/MON-VER"] = &sendMONVER;
  commands["/CFG-NAVX5"] = &sendNAVX5;
  commands["/CFG-PRT"] = &sendCFGPRT;
  commands["/CFG-DAT"] = &sendCFGDAT;
  commands["/MON-GNSS"] = &sendMONGNSS;
  commands["/BDS-GNSS"] = &sendBDSGNSS;
  commands["/ASK-GNSS"] = &askBDSGNSS;
  commands["/CFG-NMEA"] = &sendCFGNMEA;
  commands["/?"] = &showHelp;
}

void Fletcher(uint8_t *mb, uint8_t len) {
  uint8_t i, CK_A, CK_B;
  CK_A = 0;
  CK_B = 0;
  for (i = 2; i < len; i++) {
    CK_A += mb[i];
    CK_B += CK_A;
  }
  mb[len] = CK_A;
  mb[len + 1] = CK_B;
}

void handleACK() {
  //  05 01 02 00 06 3e 4c 75
  uint8_t response = B562[1];
  char temp[48];
  Serial.println("   +------------------------------------------------+");
  if (response == 1) sprintf(temp, "ACK-ACK. Class: %#2X, ID: %#2X", B562[4], B562[5]);
  else sprintf(temp, "ACK-NACK. Class: 0x%2X, ID: 0x%#2X", B562[4], B562[5]);
  sprintf(infoText, "   |%32s                |", temp);
  Serial.println(infoText);
  Serial.println("   +------------------------------------------------+");
}

void handleMONGNSS() {
  // 0a 28 08 00 00 0f 03 03 03 00 00 00 52 16
  sprintf(infoText, "Version: %d", B562[4]);
  Serial.println(infoText);
  if (B562[5] & GPSSup) Serial.println(" * GPS supported");
  else Serial.println(" * GPS not supported");
  if (B562[5] & GlonassSup) Serial.println(" * GLONASS supported");
  else Serial.println(" * GLONASS not supported");
  if (B562[5] & BeidouSup) Serial.println(" * Beidou supported");
  else Serial.println(" * Beidou not supported");
  if (B562[5] & GalileoSup) Serial.println(" * Galileo supported");
  else Serial.println(" * Galileo not supported");
  if (B562[6] & GPSSup) Serial.println(" * GPS enabled by default");
  else Serial.println(" * GPS not enabled by default");
  if (B562[6] & GlonassSup) Serial.println(" * GLONASS enabled by default");
  else Serial.println(" * GLONASS not enabled by default");
  if (B562[6] & BeidouSup) Serial.println(" * Beidou enabled by default");
  else Serial.println(" * Beidou not enabled by default");
  if (B562[6] & GalileoSup) Serial.println(" * Galileo enabled by default");
  else Serial.println(" * Galileo not enabled by default");
  if (B562[7] & GPSSup) Serial.println(" * GPS enabled");
  else Serial.println(" * GPS not enabled");
  if (B562[7] & GlonassSup) Serial.println(" * GLONASS enabled");
  else Serial.println(" * GLONASS not enabled");
  if (B562[7] & BeidouSup) Serial.println(" * Beidou enabled");
  else Serial.println(" * Beidou not enabled");
  if (B562[7] & GalileoSup) Serial.println(" * Galileo enabled");
  else Serial.println(" * Galileo not enabled");
  sprintf(infoText, "Maximum number of concurrent major GNSS that can be supported by this receiver: %d", B562[8]);
  Serial.println(infoText);
}

void SendCmd0B(uint8_t myClass, uint8_t myID, char *cmd) {
  uint8_t mb[8];
  memset(mb, 0, 8);
  mb[0] = 0xB5; // Sync Char 1
  mb[1] = 0x62; // Sync Char 2
  mb[2] = myClass;
  mb[3] = myID;
  mb[4] = 0x00;
  mb[5] = 0x00; // Payload Length, Little Endian, 16-bit
  // hexDump(mb, 8);
  Fletcher(mb, 6);
  Serial.print("Sending ");
  Serial.print(cmd);
  Serial.println(" command:");
  hexDump(mb, 8);
  Serial1.write((char *)mb, 8);
  delay(100);
}

void sendMONVER() {
  SendCmd0B(MONVER[0], MONVER[1], (char*)"MON-VER");
}

void sendNAVX5() {
  SendCmd0B(NAVX5[0], NAVX5[1], (char*)"UBX-CFG-NAVX5");
}

void sendCFGPRT() {
  SendCmd0B(CFGPRT[0], CFGPRT[1], (char*)"CFG-PRT");
}

void sendCFGDAT() {
  SendCmd0B(CFGDAT[0], CFGDAT[1], (char*)"CFG-DAT");
}

void sendCFGNMEA() {
  SendCmd0B(CFGNMEA[0], CFGNMEA[1], (char*)"CFG-NMEA");
}

void sendCFGRST() {
  SendCmd0B(CFGRST[0], CFGRST[1], (char*)"CFG-RST");
}

void sendBDSGNSS() {
  Serial.println("Sending CFG-BDS-GNSS:");
  Fletcher(CFGGNSS, 42);
  hexDump(CFGGNSS, 44);
  Serial1.write((char*)CFGGNSS, 44);
  delay(100);
}

void askBDSGNSS() {
  Serial.println("Asking about CFG-BDS-GNSS:");
  SendCmd0B(CFGGNSS[2], CFGGNSS[3], (char*)"CFG-BDS-GNSS");
  delay(100);
}

void sendMONGNSS() {
  Serial.println("Sending MON-GNSS:");
  hexDump(MONGNSS, 16);
  Serial1.write((char*)MONGNSS, 16);
  delay(100);
}

void showHelp() {
  // Display all of the mapped functions
  Serial.println("Commands:");
  std::map<string, void (*)()>::iterator it;
  for (it = commands.begin(); it != commands.end(); ++it) {
    Serial.print(" > ");
    Serial.println(it->first.c_str());
  }
}

void F0x060x06() {
  Serial.println("CFG-DAT");
  uint16_t ln = (uint16_t)B562[2];
  Serial.println(" . Payload length: " + String(ln));
  if (ln != 52) {
    Serial.println(" . Wrong Payload length");
    return;
  }

  uint8_t ix = 4; // Offset
  uint16_t dn = (uint16_t)B562[ix];
  sprintf(infoText, " . Datum Number: %d\n", dn);
  Serial.print(infoText);
  sprintf(infoText, " . Datum name: %s\n", &B562[ix + 2]);
  Serial.print(infoText);

  double R8 = (double)B562[ix + 8];
  sprintf(infoText, " . Semi-major Axis: %.2f\n", R8);
  Serial.print(infoText);

  R8 = (float)B562[ix + 16];
  sprintf(infoText, " . 1.0 / Flattening: %.2f\n", R8);
  Serial.print(infoText);

  float R4 = (float)B562[ix + 24];
  sprintf(infoText, " . dX: %.2f\n", R4);
  Serial.print(infoText);

  R4 = (float)B562[ix + 28];
  sprintf(infoText, " . dY: %.2f\n", R4);
  Serial.print(infoText);

  R4 = (float)B562[ix + 32];
  sprintf(infoText, " . dZ: %.2f\n", R4);
  Serial.print(infoText);

  R4 = (float)B562[ix + 36];
  sprintf(infoText, " . rotX: %.2f\n", R4);
  Serial.print(infoText);

  R4 = (float)B562[ix + 40];
  sprintf(infoText, " . rotY: %.2f\n", R4);
  Serial.print(infoText);

  R4 = (float)B562[ix + 44];
  sprintf(infoText, " . rotZ: %.2f\n", R4);
  Serial.print(infoText);
  R4 = (float)B562[ix + 44];
  sprintf(infoText, " . scale: %.2f\n", R4);
}

float toRad(float x) {
  return x * 3.141592653 / 180;
}

float haversine(float lat1, float lon1, float lat2, float lon2) {
  float R = 6371; // km
  float x1 = lat2 - lat1;
  float dLat = toRad(x1);
  float x2 = lon2 - lon1;
  float dLon = toRad(x2);
  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(toRad(lat1)) * cos(toRad(lat2)) *
            sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  float d = R * c;
  return round(d * 100.0) / 100;
}
