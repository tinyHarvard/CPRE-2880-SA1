#include <ArduinoBLE.h>
// ── Change this before flashing each board ──
#define TX_ID 1   // 0, 1, or 2
// ────────────────────────────────────────────
BLEService beaconService("1809");
static uint16_t seq = 0;
const int8_t TX_POWER_DBM = -62;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  if (!BLE.begin()) {
    Serial.println("ERR:BLE_INIT");
    while (1);
  }
  // Each TX advertises a unique name so RX can pre-filter by name too
  String name = "BLE_TX_" + String(TX_ID);
  BLE.setLocalName(name.c_str());
  BLE.setAdvertisedService(beaconService);
  BLE.addService(beaconService);
  Serial.print("TX_"); Serial.print(TX_ID);
  Serial.println(" ready");
}
void loop() {
  uint32_t ts = millis();
  // Manufacturer data layout (11 bytes):
  // [0-1]  Manufacturer ID
  // [2]    Packet type tag 0xA1
  // [3]    TX_ID
  // [4-7]  Timestamp (uint32 little-endian)
  // [8]    TX power (int8)
  // [9-10] Sequence number (uint16 little-endian)
  uint8_t mfr[11];
  mfr[0]  = 0xFF; mfr[1] = 0xFF;
  mfr[2]  = 0xA1;
  mfr[3]  = TX_ID;
  mfr[4]  = (ts >>  0) & 0xFF;
  mfr[5]  = (ts >>  8) & 0xFF;
  mfr[6]  = (ts >> 16) & 0xFF;
  mfr[7]  = (ts >> 24) & 0xFF;
  mfr[8]  = (uint8_t)TX_POWER_DBM;
  mfr[9]  = (seq >> 0) & 0xFF;
  mfr[10] = (seq >> 8) & 0xFF;
  seq++;
  BLE.stopAdvertise();
  BLE.setManufacturerData(mfr, sizeof(mfr));
  BLE.advertise();
  Serial.print("TX_"); Serial.print(TX_ID);
  Serial.print(" seq="); Serial.print(seq);
  Serial.print(" ts="); Serial.println(ts);
  delay(200);
}
