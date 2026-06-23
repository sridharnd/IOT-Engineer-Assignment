# IOT-Engineer-Assignment
  Garden Irrigation system

# main.ino 
Contains the code for the central MCU, which is the ESP32-WROOM-32. This is the brain of the system. It connects to both BLE modules, reads the water level from the sensor, uploads the data to the server over WiFi, and sends the pump commands at 7:00 and 7:30 every morning.

# sensor.ino 
Contains the code for the water level sensor node. This runs on the nRF52840 BLE module placed near the well. It reads the ultrasonic sensor and advertises the water level over BLE.

# pump.ino 
Contains the code for the pump controller node. This also runs on an nRF52840 BLE module, placed near the water pump. It listens for ON/OFF commands over BLE and controls the relay that switches the pump.
