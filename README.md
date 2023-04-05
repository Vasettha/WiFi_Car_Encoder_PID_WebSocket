# WiFi_Car_Encoder_PID_WebSocket
A WiFi Car with encoder, a websocket and an asynchronous server

This project is a wifi-controlled robot car built with ESP32, which receives signals from a client via WiFi and sends the corresponding commands to the motor controller. The project uses a web server to control the robot car's movements, and the client can control the speed and direction of the car's movement using a graphical user interface (GUI) served by the web server.

## Prerequisites
### Hardware used:

1. An ESP32 development board
2. A motor driver (BTS7960)
3. One DC motors with wheels
4. A rotary encoder
5. Lead-acid battery
6. Buck converter
7. Some cables and an esp32 expansion shield for ease of wiring

### Software used:
1. Arduino IDE
2. ESP32 board support package
3. AsyncTCP library (https://codeload.github.com/me-no-dev/AsyncTCP/zip/refs/heads/master)
4. ESPAsyncWebServer library (https://codeload.github.com/me-no-dev/ESPAsyncWebServer/zip/refs/heads/master)

### Wiring

![Wiring diagram](https://github.com/Vasettha/WiFi_Car_Encoder_PID_WebSocket/blob/add-license-1/Wiring%20diagram.png)

