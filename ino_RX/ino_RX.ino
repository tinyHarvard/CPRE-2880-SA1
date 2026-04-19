#include <ArduinoBLE.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("ERR:BLE_INIT");
    while (1);
  }

  BLE.scan(true);  // accept duplicates for continuous updates

  Serial.println("# Single RX listening for TX_0, TX_1, TX_2");
  Serial.println("tx_id,seq,tx_ts_ms,rx_ts_ms,rssi,tx_power");
}

void loop() {
  BLEDevice peripheral = BLE.available();
  if (!peripheral) return;

  // Accept any of our 3 TX beacons by name prefix
  String name = peripheral.localName();
  if (!name.startsWith("BLE_TX_")) return;

  uint8_t mfr[32];
  int mfrLen = peripheral.manufacturerData(mfr, sizeof(mfr));

  // Need 11 bytes and correct type tag
  if (mfrLen < 11 || mfr[2] != 0xA1) return;

  uint8_t  txId    = mfr[3];
  uint32_t txTs    = ((uint32_t)mfr[4])
                   | ((uint32_t)mfr[5] <<  8)
                   | ((uint32_t)mfr[6] << 16)
                   | ((uint32_t)mfr[7] << 24);
  int8_t   txPower = (int8_t)mfr[8];
  uint16_t seq     = (uint16_t)mfr[9] | ((uint16_t)mfr[10] << 8);
  uint32_t rxTs    = millis();
  int      rssi    = peripheral.rssi();

  // Guard: only accept known TX IDs
  if (txId > 2) return;

  Serial.print(txId);    Serial.print(",");
  Serial.print(seq);     Serial.print(",");
  Serial.print(txTs);    Serial.print(",");
  Serial.print(rxTs);    Serial.print(",");
  Serial.print(rssi);    Serial.print(",");
  Serial.println(txPower);
}