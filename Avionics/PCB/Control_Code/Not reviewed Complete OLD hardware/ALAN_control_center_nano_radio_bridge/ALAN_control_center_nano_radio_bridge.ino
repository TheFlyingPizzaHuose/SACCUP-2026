/*
  ALAN Control Center Nano Radio Bridge

  Board: Arduino Nano
  Radio: RFM95 / LoRa, RadioHead RH_RF95
  Purpose:
    - Receive packed binary telemetry from ALAN VF PCB.
    - Print decoded telemetry as JSON over USB Serial for later HTML dashboard.
    - Send binary command packets to PCB from Serial commands.

  Serial examples:
    ARM
    IGNITION
    ABORT
    GET_STATUS
    GET_GPS
    GRAVITY
    BME_CAL
    OVERRIDE 12345678
    OPEN_MAIN
    CLOSE_MAIN
    OPEN_RELIEF
    CLOSE_RELIEF
    EXIT_OVERRIDE
    SET_STAGE 5
    SET_TIME 26 5 30 14 20 00 6
    RAW 0x02
*/

#include <SPI.h>
#include <RH_RF95.h>

// ---------------- Nano RFM95 pins ----------------
#define RFM95_CS   10
#define RFM95_INT  3
#define RFM95_RST  4

// ---------------- Radio settings ----------------
float RADIO_FREQ_MHZ = 433.0f;
const int RFM95_TX_POWER_DBM = 23;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// ---------------- Protocol classes ----------------
const uint8_t MSG_CLASS_COMMAND   = 0x01;
const uint8_t MSG_CLASS_TELEMETRY = 0x02;

// ---------------- Telemetry message IDs from PCB ----------------
const uint8_t TEL_FAST_STATUS = 0x01;
const uint8_t TEL_GPS         = 0x02;
const uint8_t TEL_MODULES     = 0x03;
const uint8_t TEL_COMMAND_ACK = 0x04;

// ---------------- Command IDs sent to PCB ----------------
const uint8_t CMD_ABORT        = 0x01;
const uint8_t CMD_ARM          = 0x02;
const uint8_t CMD_IGNITION     = 0x03;
const uint8_t CMD_OPEN_MAIN    = 0x04;
const uint8_t CMD_CLOSE_MAIN   = 0x05;
const uint8_t CMD_OPEN_DUMP    = 0x06;
const uint8_t CMD_CLOSE_DUMP   = 0x07;
const uint8_t CMD_OPEN_RELIEF  = 0x08;
const uint8_t CMD_CLOSE_RELIEF = 0x09;
const uint8_t CMD_GET_STATUS   = 0x0A;
const uint8_t CMD_GET_GPS      = 0x0B;
const uint8_t CMD_GRAVITY      = 0x0C;
const uint8_t CMD_BME_CAL      = 0x0D;
const uint8_t CMD_OVERRIDE     = 0x0E;
const uint8_t CMD_EXIT_OVERRIDE= 0x0F;
const uint8_t CMD_SET_TIME     = 0x10;
const uint8_t CMD_SET_STAGE    = 0x11;

// ---------------- Serial command input ----------------
char serialLine[96];
uint8_t serialIndex = 0;

// ---------------- Packet helpers ----------------
uint8_t radioChecksum(const uint8_t* data, uint8_t len) {
  uint8_t sum1 = 0;
  uint8_t sum2 = 0;
  for (uint8_t i = 0; i < len; i++) {
    sum1 = (sum1 + data[i]) % 15;
    sum2 = (sum2 + sum1) % 15;
  }
  return (sum2 << 4) | sum1;
}

uint16_t readU16(const uint8_t* p) {
  return ((uint16_t)p[0]) | ((uint16_t)p[1] << 8);
}

int16_t readI16(const uint8_t* p) {
  return (int16_t)readU16(p);
}

uint32_t readU32(const uint8_t* p) {
  return ((uint32_t)p[0]) |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

int32_t readI32(const uint8_t* p) {
  return (int32_t)readU32(p);
}

void packU8(uint8_t* b, uint8_t& i, uint8_t v) {
  b[i++] = v;
}

void packU16(uint8_t* b, uint8_t& i, uint16_t v) {
  b[i++] = (uint8_t)(v & 0xFF);
  b[i++] = (uint8_t)((v >> 8) & 0xFF);
}

void packU32(uint8_t* b, uint8_t& i, uint32_t v) {
  b[i++] = (uint8_t)(v & 0xFF);
  b[i++] = (uint8_t)((v >> 8) & 0xFF);
  b[i++] = (uint8_t)((v >> 16) & 0xFF);
  b[i++] = (uint8_t)((v >> 24) & 0xFF);
}

void printBoolJson(const char* name, bool value, bool comma=true) {
  Serial.print('"'); Serial.print(name); Serial.print(":");
  Serial.print(value ? 1 : 0);
  if (comma) Serial.print(',');
}

// ---------------- Radio TX ----------------
void sendPacket(uint8_t* packet, uint8_t len_without_checksum) {
  packet[len_without_checksum] = radioChecksum(packet, len_without_checksum);
  uint8_t total_len = len_without_checksum + 1;

  rf95.send(packet, total_len);
  rf95.waitPacketSent();

  Serial.print("{\"type\":\"tx\",\"class\":");
  Serial.print(packet[0]);
  Serial.print(",\"id\":");
  Serial.print(packet[1]);
  Serial.print(",\"bytes\":");
  Serial.print(total_len);
  Serial.println("}");
}

void sendSimpleCommand(uint8_t command_id) {
  uint8_t p[16];
  uint8_t i = 0;
  packU8(p, i, MSG_CLASS_COMMAND);
  packU8(p, i, command_id);

  // Current PCB safety confirmation flags.
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);

  sendPacket(p, i);
}

void sendOverrideCommand(const char* password) {
  uint8_t p[24];
  uint8_t i = 0;
  packU8(p, i, MSG_CLASS_COMMAND);
  packU8(p, i, CMD_OVERRIDE);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);

  // 8-byte ASCII password. Pads with 0 if shorter.
  for (uint8_t k = 0; k < 8; k++) {
    char c = password[k];
    packU8(p, i, c ? (uint8_t)c : 0x00);
  }

  sendPacket(p, i);
}

void sendSetStageCommand(uint8_t stage) {
  uint8_t p[16];
  uint8_t i = 0;
  packU8(p, i, MSG_CLASS_COMMAND);
  packU8(p, i, CMD_SET_STAGE);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, stage);
  sendPacket(p, i);
}

void sendSetTimeCommand(uint8_t yy, uint8_t mo, uint8_t dd,
                        uint8_t hh, uint8_t mm, uint8_t ss, uint8_t dow) {
  uint8_t p[24];
  uint8_t i = 0;
  packU8(p, i, MSG_CLASS_COMMAND);
  packU8(p, i, CMD_SET_TIME);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, 0xFF);
  packU8(p, i, yy);   // year minus 2000, example: 26 for 2026
  packU8(p, i, mo);
  packU8(p, i, dd);
  packU8(p, i, hh);
  packU8(p, i, mm);
  packU8(p, i, ss);
  packU8(p, i, dow);  // 1..7, whatever convention PCB uses
  sendPacket(p, i);
}

// ---------------- Telemetry decode ----------------
void decodeFastTelemetry(const uint8_t* payload, uint8_t len, int16_t rssi) {
  if (len < 31) {
    Serial.print("{\"type\":\"rx_error\",\"reason\":\"fast_short\",\"len\":");
    Serial.print(len);
    Serial.println("}");
    return;
  }

  uint8_t i = 0;

  int32_t alt_cm       = readI32(payload + i); i += 4;
  int16_t vel_cms      = readI16(payload + i); i += 2;
  int16_t tilt_cdeg    = readI16(payload + i); i += 2;
  int32_t apogee_cm    = readI32(payload + i); i += 4;

  uint8_t stage        = payload[i++];
  uint8_t flags1       = payload[i++];
  uint8_t flags2       = payload[i++];

  uint16_t batt_mv     = readU16(payload + i); i += 2;

  uint16_t tank_x10    = readU16(payload + i); i += 2;
  uint16_t chamber_x10 = readU16(payload + i); i += 2;
  uint16_t co2_x10     = readU16(payload + i); i += 2;

  int32_t timeline_ms  = readI32(payload + i); i += 4;
  int8_t radio_db      = (int8_t)payload[i++];

  uint16_t cmd_count   = readU16(payload + i); i += 2;
  uint8_t last_cmd     = payload[i++];

  Serial.print("{\"type\":\"fast\",");

  Serial.print("\"alt_m\":");
  Serial.print(alt_cm / 100.0f, 2);
  Serial.print(',');

  Serial.print("\"vel_mps\":");
  Serial.print(vel_cms / 100.0f, 2);
  Serial.print(',');

  Serial.print("\"tilt_deg\":");
  Serial.print(tilt_cdeg / 100.0f, 2);
  Serial.print(',');

  Serial.print("\"pred_apogee_m\":");
  Serial.print(apogee_cm / 100.0f, 2);
  Serial.print(',');

  Serial.print("\"stage\":");
  Serial.print(stage);
  Serial.print(',');

  Serial.print("\"flags1\":");
  Serial.print(flags1);
  Serial.print(',');

  Serial.print("\"flags2\":");
  Serial.print(flags2);
  Serial.print(',');

  Serial.print("\"main\":");
  Serial.print((flags1 & 0x01) ? 1 : 0);
  Serial.print(',');

  Serial.print("\"relief\":");
  Serial.print((flags1 & 0x02) ? 1 : 0);
  Serial.print(',');

  Serial.print("\"dump\":");
  Serial.print((flags1 & 0x04) ? 1 : 0);
  Serial.print(',');

  Serial.print("\"battery_v\":");
  Serial.print(batt_mv / 1000.0f, 3);
  Serial.print(',');

  Serial.print("\"tank_psi\":");
  Serial.print(tank_x10 / 10.0f, 1);
  Serial.print(',');

  Serial.print("\"chamber_psi\":");
  Serial.print(chamber_x10 / 10.0f, 1);
  Serial.print(',');

  Serial.print("\"co2_psi\":");
  Serial.print(co2_x10 / 10.0f, 1);
  Serial.print(',');

  Serial.print("\"timeline_s\":");
  Serial.print(timeline_ms / 1000.0f, 3);
  Serial.print(',');

  Serial.print("\"radio_db\":");
  Serial.print(radio_db);
  Serial.print(',');

  Serial.print("\"rssi\":");
  Serial.print(rssi);
  Serial.print(',');

  Serial.print("\"commands_received\":");
  Serial.print(cmd_count);
  Serial.print(',');

  Serial.print("\"last_cmd\":");
  Serial.print(last_cmd);

  Serial.println("}");
}

void decodeGpsTelemetry(const uint8_t* payload, uint8_t len, int16_t rssi) {
  if (len < 17) return;
  uint8_t i = 0;
  int32_t lat_e7 = readI32(payload + i); i += 4;
  int32_t lon_e7 = readI32(payload + i); i += 4;
  int32_t alt_cm = readI32(payload + i); i += 4;
  uint8_t siv = payload[i++];
  uint8_t pps = payload[i++];
  uint16_t cmd_count = readU16(payload + i); i += 2;
  uint8_t last_cmd = payload[i++];

  Serial.print("{\"type\":\"gps\",");
  Serial.print("\"lat\":"); Serial.print(lat_e7 / 10000000.0, 7); Serial.print(',');
  Serial.print("\"lon\":"); Serial.print(lon_e7 / 10000000.0, 7); Serial.print(',');
  Serial.print("\"alt_m\":"); Serial.print(alt_cm / 100.0f, 2); Serial.print(',');
  Serial.print("\"siv\":"); Serial.print(siv); Serial.print(',');
  Serial.print("\"pps\":"); Serial.print(pps); Serial.print(',');
  Serial.print("\"commands_received\":"); Serial.print(cmd_count); Serial.print(',');
  Serial.print("\"last_cmd\":"); Serial.print(last_cmd); Serial.print(',');
  Serial.print("\"rssi\":"); Serial.print(rssi);
  Serial.println("}");
}

void decodeModuleTelemetry(const uint8_t* payload, uint8_t len, int16_t rssi) {
  if (len < 8) return;
  uint8_t i = 0;
  uint8_t module_flags = payload[i++];
  uint8_t makes_flags = payload[i++];
  uint8_t valve_conn_flags = payload[i++];
  uint16_t cmd_count = readU16(payload + i); i += 2;
  uint8_t last_cmd = payload[i++];
  uint8_t stage = payload[i++];
  uint8_t state_flags = payload[i++];

  Serial.print("{\"type\":\"modules\",");
  Serial.print("\"module_flags\":"); Serial.print(module_flags); Serial.print(',');
  Serial.print("\"makes_flags\":"); Serial.print(makes_flags); Serial.print(',');
  Serial.print("\"valve_conn_flags\":"); Serial.print(valve_conn_flags); Serial.print(',');
  Serial.print("\"commands_received\":"); Serial.print(cmd_count); Serial.print(',');
  Serial.print("\"last_cmd\":"); Serial.print(last_cmd); Serial.print(',');
  Serial.print("\"stage\":"); Serial.print(stage); Serial.print(',');
  Serial.print("\"state_flags\":"); Serial.print(state_flags); Serial.print(',');
  Serial.print("\"rssi\":"); Serial.print(rssi);
  Serial.println("}");
}

void decodeCommandAck(const uint8_t* payload, uint8_t len, int16_t rssi) {
  if (len < 5) return;
  uint8_t i = 0;
  uint16_t cmd_count = readU16(payload + i); i += 2;
  uint8_t last_cmd = payload[i++];
  uint8_t accepted = payload[i++];
  uint8_t reason = payload[i++];

  Serial.print("{\"type\":\"command_ack\",");
  Serial.print("\"commands_received\":"); Serial.print(cmd_count); Serial.print(',');
  Serial.print("\"last_cmd\":"); Serial.print(last_cmd); Serial.print(',');
  Serial.print("\"accepted\":"); Serial.print(accepted); Serial.print(',');
  Serial.print("\"reason\":"); Serial.print(reason); Serial.print(',');
  Serial.print("\"rssi\":"); Serial.print(rssi);
  Serial.println("}");
}

void readRadio() {
  if (!rf95.available()) return;

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (!rf95.recv(buf, &len)) return;
  if (len < 3) return;

  uint8_t expected = radioChecksum(buf, len - 1);
  if (buf[len - 1] != expected) {
    Serial.print("{\"type\":\"rx_error\",\"reason\":\"checksum\",\"len\":");
    Serial.print(len);
    Serial.println("}");
    return;
  }

  uint8_t msg_class = buf[0];
  uint8_t msg_id = buf[1];
  const uint8_t* payload = buf + 2;
  uint8_t payload_len = len - 3;
  int16_t rssi = rf95.lastRssi();

  if (msg_class == MSG_CLASS_TELEMETRY) {
    if (msg_id == TEL_FAST_STATUS) decodeFastTelemetry(payload, payload_len, rssi);
    else if (msg_id == TEL_GPS) decodeGpsTelemetry(payload, payload_len, rssi);
    else if (msg_id == TEL_MODULES) decodeModuleTelemetry(payload, payload_len, rssi);
    else if (msg_id == TEL_COMMAND_ACK) decodeCommandAck(payload, payload_len, rssi);
    else {
      Serial.print("{\"type\":\"rx_unknown\",\"class\":");
      Serial.print(msg_class);
      Serial.print(",\"id\":");
      Serial.print(msg_id);
      Serial.println("}");
    }
  }
}

// ---------------- Serial command parser ----------------
void trimLine(char* s) {
  int n = strlen(s);
  while (n > 0 && (s[n-1] == '\r' || s[n-1] == '\n' || s[n-1] == ' ' || s[n-1] == '\t')) {
    s[--n] = 0;
  }
  while (*s == ' ' || *s == '\t') memmove(s, s+1, strlen(s));
}

uint8_t parseCommandId(const char* text) {
  if (strcmp(text, "ABORT") == 0) return CMD_ABORT;
  if (strcmp(text, "ARM") == 0) return CMD_ARM;
  if (strcmp(text, "IGNITION") == 0) return CMD_IGNITION;
  if (strcmp(text, "IGNITION_START") == 0) return CMD_IGNITION;
  if (strcmp(text, "OPEN_MAIN") == 0) return CMD_OPEN_MAIN;
  if (strcmp(text, "CLOSE_MAIN") == 0) return CMD_CLOSE_MAIN;
  if (strcmp(text, "OPEN_DUMP") == 0) return CMD_OPEN_DUMP;
  if (strcmp(text, "CLOSE_DUMP") == 0) return CMD_CLOSE_DUMP;
  if (strcmp(text, "OPEN_RELIEF") == 0) return CMD_OPEN_RELIEF;
  if (strcmp(text, "CLOSE_RELIEF") == 0) return CMD_CLOSE_RELIEF;
  if (strcmp(text, "GET_STATUS") == 0) return CMD_GET_STATUS;
  if (strcmp(text, "GET_GPS") == 0) return CMD_GET_GPS;
  if (strcmp(text, "GRAVITY") == 0) return CMD_GRAVITY;
  if (strcmp(text, "GRAVITY_ANCHOR") == 0) return CMD_GRAVITY;
  if (strcmp(text, "BME_CAL") == 0) return CMD_BME_CAL;
  if (strcmp(text, "EXIT_OVERRIDE") == 0) return CMD_EXIT_OVERRIDE;
  if (strncmp(text, "0x", 2) == 0 || strncmp(text, "0X", 2) == 0) return (uint8_t)strtoul(text, NULL, 16);
  return (uint8_t)strtoul(text, NULL, 10);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  ARM | IGNITION | ABORT"));
  Serial.println(F("  GET_STATUS | GET_GPS | GRAVITY | BME_CAL"));
  Serial.println(F("  OVERRIDE <8-char-password> | EXIT_OVERRIDE"));
  Serial.println(F("  OPEN_MAIN | CLOSE_MAIN | OPEN_DUMP | CLOSE_DUMP | OPEN_RELIEF | CLOSE_RELIEF"));
  Serial.println(F("  SET_STAGE <1..5>"));
  Serial.println(F("  SET_TIME yy mo dd hh mm ss dow"));
  Serial.println(F("  RAW <cmd_id_decimal_or_hex>"));
}

void handleSerialCommand(char* line) {
  trimLine(line);
  if (strlen(line) == 0) return;

  if (strcmp(line, "HELP") == 0 || strcmp(line, "?") == 0) {
    printHelp();
    return;
  }

  if (strncmp(line, "OVERRIDE ", 9) == 0) {
    sendOverrideCommand(line + 9);
    return;
  }

  if (strncmp(line, "SET_STAGE ", 10) == 0) {
    uint8_t st = (uint8_t)atoi(line + 10);
    sendSetStageCommand(st);
    return;
  }

  if (strncmp(line, "SET_TIME ", 9) == 0) {
    int yy, mo, dd, hh, mm, ss, dow;
    if (sscanf(line + 9, "%d %d %d %d %d %d %d", &yy, &mo, &dd, &hh, &mm, &ss, &dow) == 7) {
      sendSetTimeCommand((uint8_t)yy, (uint8_t)mo, (uint8_t)dd,
                         (uint8_t)hh, (uint8_t)mm, (uint8_t)ss, (uint8_t)dow);
    } else {
      Serial.println(F("{\"type\":\"cmd_error\",\"reason\":\"bad_SET_TIME_format\"}"));
    }
    return;
  }

  if (strncmp(line, "RAW ", 4) == 0) {
    uint8_t id = parseCommandId(line + 4);
    sendSimpleCommand(id);
    return;
  }

  uint8_t id = parseCommandId(line);
  if (id == 0) {
    Serial.println(F("{\"type\":\"cmd_error\",\"reason\":\"unknown_command\"}"));
    return;
  }

  sendSimpleCommand(id);
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      serialLine[serialIndex] = 0;
      handleSerialCommand(serialLine);
      serialIndex = 0;
      return;
    }

    if (serialIndex < sizeof(serialLine) - 1) {
      serialLine[serialIndex++] = c;
    }
  }
}

void setupRadio() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println(F("{\"type\":\"ground_status\",\"rfm95_online\":0}"));
    while (1) {}
  }

  if (!rf95.setFrequency(RADIO_FREQ_MHZ)) {
    Serial.println(F("{\"type\":\"ground_status\",\"rfm95_online\":0,\"reason\":\"freq\"}"));
    while (1) {}
  }

  rf95.setTxPower(RFM95_TX_POWER_DBM, false);

  Serial.print(F("{\"type\":\"ground_status\",\"rfm95_online\":1,\"freq\":"));
  Serial.print(RADIO_FREQ_MHZ, 1);
  Serial.println(F("}"));
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  setupRadio();
  printHelp();
}

void loop() {
  readSerial();
  readRadio();
}
