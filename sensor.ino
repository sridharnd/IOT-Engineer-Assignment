#include <Arduino.h>
#include <bluefruit.h>   // this comes with the adafruit nrf52 board package, no separate install

// ultrasonic sensor pins
#define TRIG_PIN  11   // we pulse this HIGH to start a measurement
#define ECHO_PIN  12   // sensor pulls this HIGH for however long the sound took to bounce back
#define LED_PIN   LED_BUILTIN  // onboard led, just for visual feedback while testing

// well dimensions, change these to match your actual well
#define SENSOR_HEIGHT  0.30   // sensor is mounted 30cm above the water when well is full
#define MAX_DEPTH      2.00   // well holds 2 metres of water when full

// our custom BLE service and characteristic UUIDs
// these must match exactly what the ESP32 central is looking for
#define WATER_SERVICE_UUID     "12345678-1234-1234-1234-1234567890AB"
#define WATER_LEVEL_CHAR_UUID  "12345678-1234-1234-1234-1234567890AC"

// create the BLE service and characteristic objects
BLEService        waterService(WATER_SERVICE_UUID);
BLECharacteristic waterLevelChar(WATER_LEVEL_CHAR_UUID, BLERead | BLENotify, sizeof(float));
// sizeof(float) = 4 bytes, thats how we send the percentage value

float g_level = 0.0;       // stores the latest water level reading
bool  g_connected = false; // tracks if ESP32 is currently connected


// forward declarations
float measureLevel();
float getDistanceCm();
void  setupBLE();
void  onConnect(uint16_t conn_handle);
void  onDisconnect(uint16_t conn_handle, uint8_t reason);
void  goToSleep();


void setup() {
    Serial.begin(115200);
    delay(500); // wait a moment so serial monitor can open

    Serial.println("sensor node starting...");

    // set up the ultrasonic sensor pins
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW); // make sure trigger starts low

    // take the reading first before we even start BLE
    // this way the value is ready the moment ESP32 connects
    g_level = measureLevel();
    Serial.print("level read: ");
    Serial.print(g_level, 1);
    Serial.println("%");

    setupBLE(); // now start advertising with the fresh value loaded

    // blink once to show we finished setup
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);

    Serial.println("advertising, waiting for central to connect...");
}


void loop() {
    static uint32_t startedAt = millis(); // remember when we started advertising

    if (g_connected) {
        // ESP32 has connected, push the value into the characteristic
        // it can either read it directly or receive it as a notification
        waterLevelChar.writeValue((byte*)&g_level, sizeof(float));
        delay(500); // give central half a second to actually read it
    }

    // if nobody connected within 30 seconds just go to sleep anyway
    // no point staying awake burning battery
    if (!g_connected && millis() - startedAt > 30000) {
        Serial.println("nobody connected in 30s, going to sleep");
        goToSleep();
    }

    // if we were connected and now we arent, central has finished reading
    // go back to sleep and wake up in 60 seconds for next reading
    if (!g_connected && !Bluefruit.Advertising.isRunning()) {
        Serial.println("central disconnected, going to sleep");
        delay(100);
        goToSleep();
    }

    delay(100); // small pause so BLE stack can do its thing
}


// takes 5 ultrasonic readings and returns the average water level as a percentage
float measureLevel() {
    float total = 0;
    int   valid = 0;

    for (int i = 0; i < 5; i++) {
        float d = getDistanceCm();

        if (d > 5.0 && d < 400.0) {  // ignore obviously wrong values
            total += d;
            valid++;
        }
        delay(60); // HC-SR04 needs at least 60ms between pings or readings get corrupted
    }

    if (valid == 0) {
        Serial.println("all readings failed, returning -1");
        return -1.0; // something is wrong with the sensor
    }

    float avgCm = total / valid;     // average of good readings
    float avgM  = avgCm / 100.0;    // convert cm to metres

    // when well is full, distance = 0.30m (sensor to water surface)
    // when well is empty, distance = 0.30 + 2.00 = 2.30m
    // so water depth = 2.30 - measured distance
    float waterDepth = (SENSOR_HEIGHT + MAX_DEPTH) - avgM;
    float percent    = (waterDepth / MAX_DEPTH) * 100.0; // turn into 0-100%

    percent = constrain(percent, 0.0, 100.0); // clamp so we never return 101% or -2%

    Serial.print("avg distance: ");
    Serial.print(avgCm, 1);
    Serial.println("cm");

    return percent;
}


// fires the ultrasonic sensor once and returns distance in centimetres
float getDistanceCm() {
    // send a 10 microsecond pulse to trigger pin to start measurement
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);  // pulse must be exactly 10us
    digitalWrite(TRIG_PIN, LOW);

    // echo pin stays HIGH for as long as the sound took to travel out and back
    // 30000us timeout = roughly 5 metres max range
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0) return -1.0; // no echo came back, sensor might be blocked

    // distance = time * speed of sound / 2
    // speed of sound = 0.0343 cm per microsecond
    // divide by 2 because sound travels there AND back
    return (duration * 0.0343) / 2.0;
}


void setupBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);              // +4dBm gives about 50m outdoor range
    Bluefruit.setName("WaterLevelSensor"); // this is what ESP32 scans for by name

    // register our callbacks so we know when central connects or leaves
    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    // set up the GATT service
    waterService.begin(); // must call begin on service before adding characteristics

    // set up the characteristic
    waterLevelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    waterLevelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS); // anyone can read, nobody can write
    waterLevelChar.setFixedLen(sizeof(float)); // always exactly 4 bytes
    waterLevelChar.begin();

    // load the current reading into the characteristic right now
    // so if central reads immediately on connect it gets a valid value
    waterLevelChar.writeValue((byte*)&g_level, sizeof(float));

    // set up advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(waterService); // include our service UUID in the ad
    Bluefruit.ScanResponse.addName();               // name goes in scan response packet
    Bluefruit.Advertising.restartOnDisconnect(false); // dont re-advertise after disconnect, we want to sleep
    Bluefruit.Advertising.setInterval(160, 160);    // advertise every 100ms (160 x 0.625ms)
    Bluefruit.Advertising.start(30);                // advertise for max 30 seconds then stop
}


void onConnect(uint16_t conn_handle) {
    Serial.println("central connected");
    g_connected = true;
    digitalWrite(LED_PIN, HIGH); // led on while connected
}


void onDisconnect(uint16_t conn_handle, uint8_t reason) {
    Serial.print("central disconnected, reason: 0x");
    Serial.println(reason, HEX); // HEX so its easier to look up in BLE spec
    g_connected = false;
    digitalWrite(LED_PIN, LOW);
}


// puts the board to sleep for 60 seconds then wakes up and starts over
void goToSleep() {
    Bluefruit.Advertising.stop();
    Serial.println("sleeping for 60 seconds...");
    Serial.flush(); // make sure serial output finishes before we sleep

    delay(60000); // 60 second sleep
    // in production replace this with proper nRF52 System OFF for real low power
    // but delay works fine for testing and development

    NVIC_SystemReset(); // software reset, this re-runs setup() with a fresh reading
}
