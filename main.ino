/*
 * Doc ECU Bosch EDC16C39 (Hyundai Santa Fe) bang Arduino Uno + MCP2515
 *
 * Thu vien can cai: "mcp_can" cua Cory J. Fowler (Library Manager -> tim "MCP_CAN")
 *
 * Noi day MCP2515 <-> Arduino Uno:
 *   VCC -> 5V      GND -> GND
 *   CS  -> D10     SO  -> D12 (MISO)
 *   SI  -> D11     SCK -> D13
 *   INT -> D2 (khong bat buoc, code nay khong dung)
 *
 * Noi CAN:
 *   CANH -> chan K/84 cua ECU   (theo hinh Alfa/Fiat, CHUA kiem chung tren Hyundai)
 *   CANL -> chan K/83 cua ECU
 *   GND cua Arduino phai noi chung voi GND cua nguon ECU
 *
 * Serial Monitor: 115200 baud, chon "Newline"
 * Lenh:
 *   s = che do nghe (sniff), in tat ca khung CAN ECU phat ra
 *   o = gui yeu cau OBD-II (RPM, nhiet do nuoc, toc do) moi 1 giay
 *   r = do lai toc do CAN tu dau
 */

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 10

// Doi thanh MCP_16MHZ neu tren thach anh cua module ghi 16.000
#define MCP_CLOCK MCP_8MHZ

MCP_CAN CAN(CAN_CS_PIN);

const uint8_t speedCodes[] = { CAN_500KBPS, CAN_250KBPS, CAN_125KBPS };
const char *speedNames[]   = { "500 kbps", "250 kbps", "125 kbps" };
const uint8_t NUM_SPEEDS   = 3;

char mode = 's';          // 's' = sniff, 'o' = OBD
int  activeSpeed = -1;    // -1 = chua do duoc
unsigned long lastObd = 0;

// ---------------------------------------------------------------
// Doc 1 khung CAN (neu co). Tra ve true neu doc duoc khung HOP LE.
unsigned long badFrames = 0;   // so khung rac (len > 8) => nghi loi SPI/day
static byte bigBuf[256];       // dem lon de thu vien khong ghi tran bo nho

bool readFrame(unsigned long &id, bool &ext, byte &len, byte *buf) {
  if (CAN.checkReceive() != CAN_MSGAVAIL) return false;
  CAN.readMsgBuf(&id, &len, bigBuf);
  ext = (id & 0x80000000UL) == 0x80000000UL;
  id &= 0x1FFFFFFFUL;
  // Khung CAN that: toi da 8 byte. len=255 / ID=0x1FFFFFFF la du lieu rac.
  if (len > 8 || id == 0x1FFFFFFFUL) {
    badFrames++;
    return false;
  }
  memcpy(buf, bigBuf, len);
  return true;
}

void printFrame(unsigned long id, bool ext, byte len, byte *buf) {
  Serial.print(ext ? F("EXT 0x") : F("STD 0x"));
  Serial.print(id, HEX);
  Serial.print(F("  ["));
  Serial.print(len);
  Serial.print(F("]  "));
  for (byte i = 0; i < len; i++) {
    if (buf[i] < 0x10) Serial.print('0');
    Serial.print(buf[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
}

// ---------------------------------------------------------------
// Thu tung toc do, nghe 3 giay. Tra ve chi so toc do neu thay khung CAN.
int detectSpeed(uint8_t canMode) {
  for (uint8_t i = 0; i < NUM_SPEEDS; i++) {
    if (CAN.begin(MCP_ANY, speedCodes[i], MCP_CLOCK) != CAN_OK) {
      Serial.println(F("Loi khoi tao MCP2515. Kiem tra day SPI, nguon 5V, thach anh 8/16MHz."));
      return -2;
    }
    CAN.setMode(canMode);

    Serial.print(F("Thu "));
    Serial.print(speedNames[i]);
    Serial.println(canMode == MCP_LISTENONLY ? F(" (chi nghe)...") : F(" (normal, co ACK)..."));

    unsigned long t0 = millis();
    unsigned long id; bool ext; byte len; byte buf[8];
    int count = 0;
    badFrames = 0;
    while (millis() - t0 < 3000) {
      // Chi dem khung co du lieu (len >= 1). Khung len = 0 voi ID ngau nhien la nhieu.
      if (readFrame(id, ext, len, buf) && len >= 1) count++;
    }
    if (count == 0 && badFrames > 0) {
      Serial.println(F("LOI PHAN CUNG: MCP2515 tra ve du lieu rac (len=255, ID=0x1FFFFFFF)."));
      Serial.println(F("=> Khong phai du lieu ECU. Kiem tra: day SO(MISO)->D12, SI(MOSI)->D11, SCK->D13, CS->D10,"));
      Serial.println(F("   VCC->5V, GND->GND, thach anh 8/16MHz dung, day long/han lanh."));
      return -2;
    }
    if (count >= 3) {   // can it nhat 3 khung that moi coi la co ECU
      Serial.print(F("=> Thay "));
      Serial.print(count);
      Serial.print(F(" khung o toc do "));
      Serial.println(speedNames[i]);
      return i;
    }
  }
  return -1;
}

void runDetection() {
  activeSpeed = detectSpeed(MCP_LISTENONLY);
  if (activeSpeed == -1) {
    Serial.println(F("Khong thay gi o che do chi nghe. Thu lai o che do NORMAL (module se gui ACK)..."));
    activeSpeed = detectSpeed(MCP_NORMAL);
  }
  if (activeSpeed >= 0) {
    Serial.println(F("San sang. Go 's' de nghe, 'o' de gui yeu cau OBD."));
    if (mode == 'o') CAN.setMode(MCP_NORMAL);
  } else if (activeSpeed == -1) {
    Serial.println(F("ECU khong phat khung nao. Kiem tra nguon ECU (K/1, K/5, K/48, GND), day CANH/CANL, GND chung."));
    Serial.println(F("Luu y: ECU de ban co the im lang cho den khi duoc hoi. Thu lenh 'o'."));
    // van khoi tao san 500 kbps de co the thu lenh 'o'
    CAN.begin(MCP_ANY, CAN_500KBPS, MCP_CLOCK);
    CAN.setMode(MCP_NORMAL);
    activeSpeed = 0;
  }
    if (activeSpeed == -2) {
    Serial.println(F("DUNG: MCP2515 khong phan hoi. Sua phan cung roi reset Arduino."));
    while (true) { delay(1000); }
  }
}

// ---------------------------------------------------------------
// Gui yeu cau OBD-II Mode 01. Tra ve true neu co phan hoi (a, b = byte du lieu).
bool obdQuery(byte pid, byte &a, byte &b) {
  byte req[8] = { 0x02, 0x01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
  if (CAN.sendMsgBuf(0x7DF, 0, 8, req) != CAN_OK) return false;

  unsigned long t0 = millis();
  unsigned long id; bool ext; byte len; byte buf[8];
  while (millis() - t0 < 300) {
    if (readFrame(id, ext, len, buf)) {
      if (id >= 0x7E8 && id <= 0x7EF && buf[1] == 0x41 && buf[2] == pid) {
        a = buf[3];
        b = buf[4];
        return true;
      }
    }
  }
  return false;
}

void doObd() {
  byte a, b;
  Serial.println(F("--- Yeu cau OBD-II ---"));

  if (obdQuery(0x0C, a, b)) {
    Serial.print(F("RPM: "));
    Serial.println(((unsigned int)a * 256 + b) / 4);
  } else {
    Serial.println(F("RPM: khong co phan hoi"));
  }

  if (obdQuery(0x05, a, b)) {
    Serial.print(F("Nhiet do nuoc: "));
    Serial.print((int)a - 40);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("Nhiet do nuoc: khong co phan hoi"));
  }

  if (obdQuery(0x0D, a, b)) {
    Serial.print(F("Toc do: "));
    Serial.print(a);
    Serial.println(F(" km/h"));
  } else {
    Serial.println(F("Toc do: khong co phan hoi"));
  }
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("=== Doc ECU bang MCP2515 ==="));
  runDetection();
}

void loop() {
  // Nhan lenh tu Serial
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      mode = 's';
      Serial.println(F("Che do nghe (sniff)."));
    } else if (c == 'o') {
      mode = 'o';
      CAN.setMode(MCP_NORMAL);
      Serial.println(F("Che do OBD (gui yeu cau moi 1 giay)."));
    } else if (c == 'r') {
      runDetection();
    }
  }

  if (mode == 's') {
    unsigned long id; bool ext; byte len; byte buf[8];
    if (readFrame(id, ext, len, buf)) printFrame(id, ext, len, buf);
  } else if (mode == 'o') {
    if (millis() - lastObd > 1000) {
      lastObd = millis();
      doObd();
    }
  }
}
