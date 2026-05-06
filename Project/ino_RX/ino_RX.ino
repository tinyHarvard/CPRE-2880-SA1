#include <ArduinoBLE.h>

void setup() {
    Serial.begin(115200);
    while (!Serial);

    if (!BLE.begin()) {
        Serial.println("ERR:BLE_INIT");
        while (1);
    }

    BLE.scan(true);   // true = accept duplicates
    Serial.println("# RX ready");
}

void loop() {
    // restart scan every 500ms to reset duplicate filtering
    static unsigned long last_restart = 0;
    if (millis() - last_restart > 500) {
        BLE.stopScan();
        BLE.scan(true);
        last_restart = millis();
    }

    BLEDevice dev = BLE.available();
    if (!dev) return;

    String name = dev.localName();
    if (!name.startsWith("BLE_TX_")) return;

    uint8_t mfr[20];
    int mfr_len = dev.manufacturerData(mfr, sizeof(mfr));
    if (mfr_len < 11) return;
    if (mfr[2] != 0xA1) return;

    int      tx_id    = mfr[3];
    uint32_t tx_ts    = mfr[4] | ((uint32_t)mfr[5]<<8)
                               | ((uint32_t)mfr[6]<<16)
                               | ((uint32_t)mfr[7]<<24);
    int8_t   tx_power = (int8_t)mfr[8];
    uint16_t seq      = mfr[9] | ((uint16_t)mfr[10]<<8);
    int      rssi     = dev.rssi();
    uint32_t rx_ts    = millis();

    // CSV: tx_id,seq,tx_ts,rx_ts,rssi,tx_power
    Serial.print(tx_id);    Serial.print(",");
    Serial.print(seq);      Serial.print(",");
    Serial.print(tx_ts);    Serial.print(",");
    Serial.print(rx_ts);    Serial.print(",");
    Serial.print(rssi);     Serial.print(",");
    Serial.println(tx_power);
}
