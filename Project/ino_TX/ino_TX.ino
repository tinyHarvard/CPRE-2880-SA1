#include <ArduinoBLE.h>

#define TX_ID 1
BLEService beaconService("1809");
static uint16_t seq = 0;
const int8_t TX_POWER_DBM = -74;

void setup() {
    Serial.begin(115200);
    // removed while(!Serial) — allows running without laptop
    delay(500);

    if (!BLE.begin()) {
        pinMode(LED_BUILTIN, OUTPUT);
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(100);
            digitalWrite(LED_BUILTIN, LOW);  delay(100);
        }
    }

    String name = "BLE_TX_" + String(TX_ID);
    BLE.setLocalName(name.c_str());
    BLE.setAdvertisedService(beaconService);
    BLE.addService(beaconService);

    if (Serial) {
        Serial.print("TX_"); Serial.print(TX_ID);
        Serial.println(" ready");
    }
}

void loop() {
    uint32_t ts = millis();

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

//    if (Serial) {
//        Serial.print("TX_"); Serial.print(TX_ID);
//        Serial.print(" seq="); Serial.print(seq);
//        Serial.print(" ts="); Serial.println(ts);
//    }

    delay(200);
}
