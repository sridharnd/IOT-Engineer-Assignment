#include <Arduino.h>
#include <bluefruit.h>  // adafruit nrf52 BLE library, included with the board package

// relay is connected to pin 5
// when we write HIGH to this pin, relay closes and pump turns on
#define RELAY_PIN  5
#define LED_PUMP   LED_BUILTIN  // mirrors pump state so you can see whats happening
#define LED_BLE    7            // lights up when ESP32 is connected

// safety cutoff — if pump runs longer than 35 minutes we force it off
// 35 min = 35 * 60 * 1000 milliseconds
#define MAX_PUMP_TIME  (35UL * 60UL * 1000UL)

// these UUIDs must match the ESP32 central code exactly
#define PUMP_SERVICE_UUID       "12345678-1234-1234-1234-1234567890CD"
#define PUMP_CMD_CHAR_UUID      "12345678-1234-1234-1234-1234567890CE"
#define PUMP_STATUS_CHAR_UUID   "12345678-1234-1234-1234-1234567890CF"

// BLE service and two characteristics — one for receiving commands, one for reporting status
BLEService        pumpService(PUMP_SERVICE_UUID);
BLECharacteristic pumpCmdChar(PUMP_CMD_CHAR_UUID, BLEWrite, 1);       // central writes 1 byte here
BLECharacteristic pumpStatusChar(PUMP_STATUS_CHAR_UUID, BLERead | BLENotify, 1); // we report status here

bool     g_pumpOn        = false;  // is pump currently running
uint32_t g_pumpStartTime = 0;      // millis() when pump turned on, used for safety timer
bool     g_connected     = false;  // is central connected right now

// forward declarations
void setupBLE();
void setPump(bool on);
void onConnect(uint16_t conn_handle);
void onDisconnect(uint16_t conn_handle, uint8_t reason);
void onCommandWrite(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len);


void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("pump controller starting...");

    // very important: set pin LOW before setting it as output
    // this prevents a brief relay trigger glitch on power up
    digitalWrite(RELAY_PIN, LOW);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW); // write it again just to be sure

    pinMode(LED_PUMP, OUTPUT);
    pinMode(LED_BLE, OUTPUT);
    digitalWrite(LED_PUMP, LOW); // pump led starts off
    digitalWrite(LED_BLE, LOW);  // ble led starts off

    g_pumpOn = false; // make sure state matches hardware

    Serial.println("relay set to OFF, safe to start");

    setupBLE();
    Serial.println("advertising started, ready for commands");
}


void loop() {
    // this is the most important thing in the loop
    // even if ESP32 crashes or BLE drops, this will stop the pump
    if (g_pumpOn) {
        uint32_t runTime = millis() - g_pumpStartTime; // how long has pump been on

        if (runTime >= MAX_PUMP_TIME) {
            Serial.println("SAFETY: pump hit 35 min limit, forcing off");
            setPump(false); // cut the pump regardless of what central says
        }
    }

    delay(100); // nothing else to do, BLE callbacks handle everything
}


// single function to turn pump on or off
// always use this instead of writing to the relay pin directly
// keeps hardware and software state in sync every time
void setPump(bool on) {
    g_pumpOn = on;

    if (on) {
        g_pumpStartTime = millis();    // record when it started for safety timer
        digitalWrite(RELAY_PIN, HIGH); // close relay = pump on
        digitalWrite(LED_PUMP, HIGH);  // led on
        Serial.println("PUMP ON");
    } else {
        digitalWrite(RELAY_PIN, LOW);  // open relay = pump off
        digitalWrite(LED_PUMP, LOW);   // led off
        Serial.println("PUMP OFF");

        // log how long it ran
        uint32_t seconds = (millis() - g_pumpStartTime) / 1000;
        Serial.print("ran for ");
        Serial.print(seconds);
        Serial.println(" seconds");
    }

    // update the status characteristic so ESP32 can verify the command worked
    uint8_t statusByte = on ? 0x01 : 0x00;
    pumpStatusChar.writeValue(&statusByte, 1); // also sends notify if central subscribed
}


// this gets called automatically when ESP32 writes to the command characteristic
void onCommandWrite(uint16_t conn_handle, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
    if (len < 1) {
        Serial.println("empty write, ignoring"); // shouldnt happen but just in case
        return;
    }

    uint8_t cmd = data[0]; // first byte is the command

    Serial.print("command received: 0x");
    Serial.println(cmd, HEX);

    if (cmd == 0x01) {
        // turn pump on
        if (!g_pumpOn) {
            setPump(true);
        } else {
            Serial.println("pump already on, ignoring"); // dont reset the timer
        }
    }
    else if (cmd == 0x00 || cmd == 0xFF) {
        // 0x00 = normal off, 0xFF = emergency stop, both do the same thing
        if (g_pumpOn) {
            setPump(false);
        } else {
            Serial.println("pump already off, ignoring");
        }
    }
    else {
        Serial.print("unknown command: 0x");
        Serial.println(cmd, HEX); // log it but dont do anything
    }
}


void setupBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);         // +4dBm, good for 50m outdoor range
    Bluefruit.setName("PumpController"); // name ESP32 scans for

    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    pumpService.begin(); // initialise service before adding characteristics

    // command characteristic — central writes a single byte to control the pump
    pumpCmdChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    pumpCmdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN); // open write, no pairing needed
    pumpCmdChar.setFixedLen(1);                            // always exactly 1 byte
    pumpCmdChar.setWriteCallback(onCommandWrite);          // call our function when written
    pumpCmdChar.begin();

    // status characteristic — we write here to tell central what the pump is doing
    pumpStatusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    pumpStatusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // read only
    pumpStatusChar.setFixedLen(1);
    pumpStatusChar.begin();

    // set initial status to OFF
    uint8_t off = 0x00;
    pumpStatusChar.writeValue(&off, 1);

    // advertising setup
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(pumpService);
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.restartOnDisconnect(true); // unlike sensor, pump always re-advertises
                                                     // because ESP32 needs to reconnect at 7:30 to turn it off
    Bluefruit.Advertising.setInterval(160, 320);     // 100ms fast, 200ms slow after 30s
    Bluefruit.Advertising.setFastTimeout(30);
    Bluefruit.Advertising.start(0);                  // 0 = advertise forever, never stop
}


void onConnect(uint16_t conn_handle) {
    Serial.println("central connected");
    g_connected = true;
    digitalWrite(LED_BLE, HIGH); // ble led on
}


void onDisconnect(uint16_t conn_handle, uint8_t reason) {
    Serial.print("central disconnected, reason: 0x");
    Serial.println(reason, HEX);
    g_connected = false;
    digitalWrite(LED_BLE, LOW); // ble led off

    // pump keeps running after disconnect if it was on
    // this is intentional — central will reconnect at 7:30 to turn it off
    // and the 35 min safety timer is the backup if that fails
    if (g_pumpOn) {
        Serial.println("note: pump still running, waiting for off command");
    }
}
