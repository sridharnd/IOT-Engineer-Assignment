#include <WiFi.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <ArduinoJson.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// wifi credentials, change these to yours
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// where we send the water level data
const char* SERVER_URL = "http://your-server.com/api/data";

// pump turns on at 7:00 and off at 7:30 every morning
const int START_HOUR   = 7;
const int START_MINUTE = 0;
const int STOP_HOUR    = 7;
const int STOP_MINUTE  = 30;

// if well is below 20% we skip watering, dont want to burn the pump
const float LOW_WATER_THRESHOLD = 20.0;

// these are the names we scan for over BLE
const char* SENSOR_NAME = "WaterLevelSensor";
const char* PUMP_NAME   = "PumpController";

// UUIDs must match exactly what the nRF52840 boards are using
const char* WATER_SERVICE_UUID    = "12345678-1234-1234-1234-1234567890AB";
const char* WATER_LEVEL_CHAR_UUID = "12345678-1234-1234-1234-1234567890AC";
const char* PUMP_SERVICE_UUID     = "12345678-1234-1234-1234-1234567890CD";
const char* PUMP_CMD_CHAR_UUID    = "12345678-1234-1234-1234-1234567890CE";
const char* PUMP_STATUS_CHAR_UUID = "12345678-1234-1234-1234-1234567890CF";

// NTP gives us real time, 19800 is IST offset from UTC in seconds (5.5 hours)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 19800, 60000);

// keeping track of a few things across loop() calls
float    g_waterLevel         = -1.0;   // -1 means we havent read it yet
bool     g_pumpIsOn           = false;  // is the pump running right now
bool     g_irrigationDone     = false;  // did we already water today
int      g_lastDay            = -1;     // used to detect when a new day starts
uint32_t g_lastSensorReadTime = 0;      // millis() of last sensor read

// this is used during BLE scanning to store the address of the device we found
NimBLEAddress g_foundAddress;
bool          g_deviceFound = false;

// this class gets called every time BLE scan finds a nearby device
class ScanCallback : public NimBLEAdvertisedDeviceCallbacks {
public:
    String targetName; // we set this before scanning so we know what to look for

    void onResult(NimBLEAdvertisedDevice* device) override {
        // check if this device has a name and if it matches what we want
        if (device->haveName() && device->getName() == targetName.c_str()) {
            Serial.print("found it: ");
            Serial.println(device->getName().c_str()); // print what we found
            g_foundAddress = device->getAddress(); // save the address to connect later
            g_deviceFound  = true;                 // flag so the main code knows
            NimBLEDevice::getScan()->stop();        // stop scanning, no need to continue
        }
    }
};

ScanCallback scanCallback; // one instance we reuse for all scans

// forward declarations so functions can call each other in any order
void          connectWiFi();
void          syncTime();
bool          getTime(int& hour, int& minute, int& day);
float         readWaterLevel();
bool          sendPumpCommand(uint8_t cmd);
bool          uploadData(float level, time_t ts);
NimBLEClient* connectToBLE(const char* name);
void          disconnectBLE(NimBLEClient* client);


void setup() {
    Serial.begin(115200);
    delay(1000); // small pause so serial monitor has time to open

    Serial.println("garden hub starting up...");

    connectWiFi();   // get on wifi first
    syncTime();      // then sync the clock

    // init BLE, "GardenHub" is our name but as central nobody really sees it
    NimBLEDevice::init("GardenHub");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max power for outdoor range

    Serial.println("ready. waiting for 7am...");
}


void loop() {
    uint32_t now = millis();

    // read the sensor roughly every 60 seconds
    if (now - g_lastSensorReadTime >= 60000 || g_lastSensorReadTime == 0) {
        g_lastSensorReadTime = now;

        Serial.println("reading water level...");
        float level = readWaterLevel(); // go do the BLE read

        if (level >= 0) {
            g_waterLevel = level; // save it
            Serial.print("water level: ");
            Serial.print(level, 1);
            Serial.println("%");

            time_t ts = timeClient.getEpochTime(); // get current unix timestamp
            uploadData(level, ts);                 // send it to server
        } else {
            Serial.println("sensor read failed this time");
        }
    }

    // check what time it is and handle the irrigation schedule
    int h, m, d;
    if (getTime(h, m, d)) {

        // midnight crossed, new day, reset the done flag so we water again tomorrow
        if (d != g_lastDay) {
            g_irrigationDone = false;
            g_lastDay = d;
            Serial.println("new day, irrigation reset");
        }

        // 7:00 am and pump is off and we havent watered yet today
        if (h == START_HOUR && m == START_MINUTE && !g_pumpIsOn && !g_irrigationDone) {
            Serial.println("7am, time to water");

            // check the well level before starting
            if (g_waterLevel >= 0 && g_waterLevel < LOW_WATER_THRESHOLD) {
                Serial.println("water too low, skipping today");
            } else {
                bool ok = sendPumpCommand(0x01); // 0x01 = turn pump on
                if (ok) {
                    g_pumpIsOn = true;
                    Serial.println("pump is on, watering started");
                } else {
                    Serial.println("failed to turn pump on, will retry next minute");
                }
            }
        }

        // 7:30 am and pump is running, time to stop
        if (h == STOP_HOUR && m == STOP_MINUTE && g_pumpIsOn) {
            Serial.println("7:30am, stopping pump");

            bool ok = sendPumpCommand(0x00); // 0x00 = turn pump off
            if (ok) {
                g_pumpIsOn = false;
                g_irrigationDone = true; // mark done so it doesnt trigger again today
                Serial.println("pump off, done watering for today");
            } else {
                Serial.println("failed to turn pump off, will keep trying");
                // we dont set irrigationDone here so it retries next loop
            }
        }
    }

    // reconnect wifi if it dropped
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("wifi dropped, reconnecting...");
        connectWiFi();
        syncTime();
    }

    delay(30000); // check every 30 seconds, no need to hammer it faster
}


// scans for a BLE device by name, connects, and returns the client object
// returns nullptr if we couldnt find or connect to it
NimBLEClient* connectToBLE(const char* name) {
    Serial.print("scanning for: ");
    Serial.println(name);

    scanCallback.targetName = name; // tell the callback what name we want
    g_deviceFound = false;          // reset from last scan

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&scanCallback, false);
    scan->setActiveScan(true);  // active scan so we get the full device name
    scan->setInterval(100);
    scan->setWindow(99);
    scan->start(10, false);     // scan for up to 10 seconds

    // wait until found or timeout
    uint32_t start = millis();
    while (!g_deviceFound && millis() - start < 11000) {
        delay(100);
    }
    scan->stop();

    if (!g_deviceFound) {
        Serial.print("couldnt find: ");
        Serial.println(name); // device not nearby or not advertising
        return nullptr;
    }

    // try to connect to the address we found
    NimBLEClient* client = NimBLEDevice::createClient();
    client->setConnectTimeout(10); // give it 10 seconds to connect

    Serial.print("connecting to: ");
    Serial.println(g_foundAddress.toString().c_str());

    if (!client->connect(g_foundAddress)) {
        Serial.println("connection failed");
        NimBLEDevice::deleteClient(client);
        return nullptr;
    }

    // discover services on the connected device
    if (!client->discoverAttributes()) {
        Serial.println("service discovery failed");
        client->disconnect();
        NimBLEDevice::deleteClient(client);
        return nullptr;
    }

    Serial.println("connected");
    return client; // caller is responsible for disconnecting
}


void disconnectBLE(NimBLEClient* client) {
    if (client) {
        if (client->isConnected()) client->disconnect();
        delay(100); // small pause to let BLE stack settle
        NimBLEDevice::deleteClient(client); // free memory
    }
}


// connects to the sensor, reads the water level float, disconnects
// returns the level percentage or -1 if something went wrong
float readWaterLevel() {
    for (int attempt = 1; attempt <= 3; attempt++) {
        Serial.print("sensor attempt ");
        Serial.println(attempt);

        NimBLEClient* client = connectToBLE(SENSOR_NAME);
        if (!client) continue; // try again

        // find our custom service
        NimBLERemoteService* svc = client->getService(WATER_SERVICE_UUID);
        if (!svc) {
            Serial.println("water service not found on device");
            disconnectBLE(client);
            continue;
        }

        // find the characteristic that holds the level value
        NimBLERemoteCharacteristic* ch = svc->getCharacteristic(WATER_LEVEL_CHAR_UUID);
        if (!ch || !ch->canRead()) {
            Serial.println("level characteristic not readable");
            disconnectBLE(client);
            continue;
        }

        std::string raw = ch->readValue(); // read the raw bytes

        if (raw.length() < 4) {
            Serial.println("got too few bytes back");
            disconnectBLE(client);
            continue;
        }

        float level;
        memcpy(&level, raw.data(), sizeof(float)); // convert 4 bytes into a float

        if (level < 0.0 || level > 100.0) {
            Serial.println("value out of range, ignoring");
            disconnectBLE(client);
            continue;
        }

        disconnectBLE(client);
        return level; // all good
    }

    return -1.0; // all 3 attempts failed
}


// connects to pump controller, writes the command byte, disconnects
// returns true if successful
bool sendPumpCommand(uint8_t cmd) {
    for (int attempt = 1; attempt <= 5; attempt++) {
        Serial.print("pump command attempt ");
        Serial.print(attempt);
        Serial.print(", cmd=0x");
        Serial.println(cmd, HEX);

        NimBLEClient* client = connectToBLE(PUMP_NAME);
        if (!client) {
            delay(3000); // wait a bit before retry
            continue;
        }

        NimBLERemoteService* svc = client->getService(PUMP_SERVICE_UUID);
        if (!svc) {
            Serial.println("pump service not found");
            disconnectBLE(client);
            continue;
        }

        NimBLERemoteCharacteristic* ch = svc->getCharacteristic(PUMP_CMD_CHAR_UUID);
        if (!ch || !ch->canWrite()) {
            Serial.println("cant write to pump characteristic");
            disconnectBLE(client);
            continue;
        }

        bool written = ch->writeValue(&cmd, 1, true); // true = wait for acknowledgement
        if (!written) {
            Serial.println("write failed");
            disconnectBLE(client);
            delay(2000);
            continue;
        }

        Serial.println("command sent ok");

        // optional: read back the status to confirm pump did what we said
        NimBLERemoteCharacteristic* status = svc->getCharacteristic(PUMP_STATUS_CHAR_UUID);
        if (status && status->canRead()) {
            std::string val = status->readValue();
            if (val.length() > 0) {
                Serial.print("pump confirmed: ");
                Serial.println((uint8_t)val[0] == 0x01 ? "ON" : "OFF");
            }
        }

        disconnectBLE(client);
        return true; // success
    }

    return false; // all retries failed
}


// sends water level reading to the server as JSON over HTTP
bool uploadData(float level, time_t ts) {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("no wifi, cant upload");
        return false;
    }

    // build the JSON body
    StaticJsonDocument<200> doc;
    doc["device"]      = "garden_hub";
    doc["water_level"] = round(level * 10.0) / 10.0; // one decimal place
    doc["timestamp"]   = (long)ts;
    doc["pump_on"]     = g_pumpIsOn;

    String body;
    serializeJson(doc, body);

    Serial.print("uploading: ");
    Serial.println(body);

    HTTPClient http;
    http.begin(SERVER_URL);
    http.addHeader("Content-Type", "application/json");
    http.setTimeout(10000); // 10 second timeout

    int code = http.POST(body);

    if (code == 200 || code == 201) {
        Serial.println("upload ok");
        http.end();
        return true;
    } else {
        Serial.print("upload failed, code: ");
        Serial.println(code);
        http.end();
        return false;
    }
}


void connectWiFi() {
    Serial.print("connecting to wifi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
        delay(500);
        Serial.print("."); // show progress dots
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("connected, IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("wifi failed, will retry later");
    }
}


void syncTime() {
    if (WiFi.status() != WL_CONNECTED) return;
    timeClient.begin();
    timeClient.update();
    Serial.print("time synced: ");
    Serial.println(timeClient.getFormattedTime()); // shows HH:MM:SS
}


// fills in hour, minute and day from the NTP client
// returns false if clock hasnt been set yet
bool getTime(int& hour, int& minute, int& day) {
    timeClient.update();

    if (timeClient.getEpochTime() < 1000000) return false; // clock not synced yet

    hour   = timeClient.getHours();
    minute = timeClient.getMinutes();

    time_t epoch = timeClient.getEpochTime();
    struct tm* t = localtime(&epoch);
    day = t->tm_mday; // day of month, used to detect new day

    return true;
}
