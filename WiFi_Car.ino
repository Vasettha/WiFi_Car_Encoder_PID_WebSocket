#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

volatile int encoderValue = 0;  //value of encoder
float distanceValue = 0.0;      //distance traveled in cm

//PID controller
int out = 0;
long unsigned int time_;
double target = 100.0;
double Setpoint, Input, Output;
double Kp = 0.8, Ki = 0.5, Kd = 0.9;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Pin definitions

//for motor control
#define R_PWM 15
#define L_PWM 2
#define L_EN 0
#define R_EN 4
//for encoder
#define ENCA 22
#define ENCB 23
//for LCD
#define SDA 26
#define SCL 25
//for Bluetooth
#define BLE_RX 16
#define BLE_TX 17
//Dimension of the wheel & encoder pulse per rotation
#define CIR 45.553055  //Circumference of the wheel
#define PPR 900.0      //pulse per rotation

//create LCD object
LiquidCrystal_I2C lcd(0x3F, 16, 2);
//ip address
IPAddress IP;

/*        ENCODER     */

//ISR to count encoder pulse
void IRAM_ATTR ISR() {

  if (digitalRead(ENCB) == digitalRead(ENCA)) {
    //counterclockwise
    encoderValue--;
  } else {
    //clockwise
    encoderValue++;
  }
}
//Processing the encoder value to display in cm
float encToDistance(int encoderValue) {
  // Serial.println((encoderValue/PPR)*CIR);
  return ((encoderValue / PPR) * CIR);
}
/*        Channge the PID parameters through serial     */

void checkSerial1() {
  if (Serial1.available()) {
    Serial.println("Serial data received");
    String data = Serial1.readStringUntil('\n');
    String values[4];  //creates an array of values
    int i = 0;
    while (i < 4) {
      int commaIndex = data.indexOf(',');
      if (commaIndex == -1) {
        values[i] = data;
        data = "";
      } else {
        values[i] = data.substring(0, commaIndex);
        data = data.substring(commaIndex + 1);
      }
      i++;
    }
    Kp = values[0].toFloat();
    Ki = values[1].toFloat();
    Kd = values[2].toFloat();
    Setpoint = values[3].toFloat();

    //confirmation through the lcd
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data received:");
    lcd.setCursor(0, 1);
    lcd.print(String(Kp) + "," + String(Ki) + "," + String(Kd));
  }
}



/*        PID Control     */

//Function take in setpoint(target distance).
//Its Input is based on cm from distanceValue
//it returns Output (-255 to 255)

int PIDcontrol(double target) {
  myPID.SetOutputLimits(-255.0, 255.0);
  myPID.SetSampleTime(100);
  distanceValue = encToDistance(encoderValue);
  Setpoint = target;
  Input = distanceValue;
  myPID.Compute();
  // Serial.print(Kp);
  // Serial.print(" ");
  // Serial.print(Ki);
  // Serial.print(" ");
  // Serial.print(Kd);
  // Serial.print(" ");
  // Serial.print(Setpoint);
  // Serial.print(" ");
  // Serial.print(Input);
  // Serial.print(" ");
  Serial.println(Output);

  return Output;
}

/*        Server     */

//WiFi name and password
// const char *ssid = "Tselhome-6DD8";
// const char *password = "80962622";
const char *ssid = "Group1";
const char *password = "12345678";

//set web server to use port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//HTML page shown to the client
const char *htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
    <style>
    .header-wrapper {
        text-align: center;
    }
    #content{
        display: flex;
        flex-direction: row;
     }
    #left {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
        width: 30%;
        padding: 0 20px;
    }
    #right {
        display: flex;
        flex-direction: column;
        align-items: flex-start;
        width: 70%;
        padding: 0 20px;
    }
    #slider-container {
        height: 300px;
        display: flex;
        flex-direction: column;
        justify-content: center;
        align-items: center;
        margin: 0 auto;
    }
    #slider {
        appearance: none;
        transform: rotate(270deg) translateY(-50%);
        width: 200px;
        height: 20px;
        background: #AAAAAA;
        outline: none;
        opacity: 0.7;
        transition: opacity 0.2s;
        position: absolute;
        top: 50%;
    }
    #slider::-webkit-slider-thumb {
        appearance: none;
        width: 40px;
        height: 40px;
        background: #678dc8;
        cursor: pointer;
        border-radius: 50%;
        border: 1px solid white;
    }
    form {
        display: flex;
        flex-direction: column;
        align-items: center;
    }
    label {
        display: flex;
        flex-direction: row;
        justify-content: center;
        align-items: center;
        margin-bottom: 10px;
    }
    label span {
        margin-right: 10px;
    }
    .noselect {
      -webkit-touch-callout: none; /* iOS Safari */
        -webkit-user-select: none; /* Safari */
         -khtml-user-select: none; /* Konqueror HTML */
           -moz-user-select: none; /* Firefox */
            -ms-user-select: none; /* Internet Explorer/Edge */
                user-select: none; /* Non-prefixed version, currently
                                      supported by Chrome and Opera */
    }
    </style>
  </head>
  <title>Group 1 WiFi Car</title>
  <body class="noselect" align="center" style="background-color:dimgrey">
  <div class="header-wrapper">
  <h1 style="color: black;text-align:center;">Microcontroller Group 1</h1>
  <h2 style="color: black;text-align:center;">Wi-Fi &#128663; Control</h2>
  </div>
    <div id="content">
      <div id="left">
		<div id="slider-container">
          <input type="range" min="-100" max="100" value="0" step="1" id="slider">
        </div>
      </div>

      <div id="right">
        <p>Distance moved: <span id="encoderValue">0</span> cm</p>
        <label for="speed">Speed: <span id="speedVal">0</span> </label>

          <form id="myForm">
              <table>
                  <tr>
                      <td><label for="Kp">Kp:<span id="disp_Kp">0</span></label></td>
                      <td><input type="number" id="Kp" name="Kp" step="0.01"></td>
                  </tr>
                  <tr>
                      <td><label for="Ki">Ki:<span id="disp_Ki">0</span></label></td>
                      <td><input type="number" id="Ki" name="Ki" step="0.01"></td>
                  </tr>
                  <tr>
                      <td><label for="Kd">Kd:<span id="disp_Kd">0</span></label></td>
                      <td><input type="number" id="Kd" name="Kd" step="0.01"></td>
                  </tr>
                  <tr>
                      <td><label for="setPoint">Target:<span id="disp_setPoint">0</span></label></td>
                      <td><input type="number" id="setPoint" name="setPoint" step="0.01"></td>
                  </tr>
                  <tr>
                      <td colspan="2"><input type="submit" value="Run!"></td>
                  </tr>
              </table>
          </form>

      </div>
    </div>
    
  </body>
    <script>
      var webSocketUrl = "ws:\/\/" + window.location.hostname + "/ws";
      var websocket;
		
      function initWebSocket() 
      {
        websocket = new WebSocket(webSocketUrl);
        websocket.onopen    = function(event){};
        websocket.onclose   = function(event){setTimeout(initWebSocket, 2000);};
        websocket.onmessage = function(event){ 
          // Parse the received data as JSON
            let data = JSON.parse(event.data);
            var time_ = 0;
            var distance_=0;
            // Check if the received data contains the new encoder value
            if (data.encoderValue) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("encoderValue").innerHTML = data.encoderValue;
                distance = data.encoderValue;
            }
            if (data.time_){
                time_ = data.time_;
                addDataPoint(time_, distance_);
            }
            if (data.disp_Kp) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("disp_Kp").innerHTML = data.disp_Kp;
            }
            if (data.disp_Ki) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("disp_Ki").innerHTML = data.disp_Ki;
            }
            if (data.disp_Kd) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("disp_Kd").innerHTML = data.disp_Kd;
            }
            if (data.disp_setPoint) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("disp_setPoint").innerHTML = data.disp_setPoint;
            }

        };
      }          
      window.onload = initWebSocket;
      
        const slider = document.getElementById("slider");
        
        slider.addEventListener("input", () => {
        const value = slider.value;
        // Create a JSON object to send to the server
        const data = {sliderValue: value};
        // Convert the JSON object to a string and send it over the WebSocket
        websocket.send(JSON.stringify(data));
        document.getElementById("speedVal").innerHTML = value;
        });

        slider.addEventListener("touchend", () => {
        if (slider.value !== 0) {
            slider.value = 0;
            const value = slider.value;
        // Create a JSON object to send to the server
            const data = {sliderValue: value};
        // Convert the JSON object to a string and send it over the WebSocket
            websocket.send(JSON.stringify(data));
            document.getElementById("speedVal").innerHTML = value;
        }
        });
        
        const form = document.querySelector('form');
        form.addEventListener('submit', (event) => {
        event.preventDefault(); // prevent the form from submitting normally
        const data = {
        Kp: parseFloat(document.getElementById('Kp').value),
        Ki: parseFloat(document.getElementById('Ki').value),
        Kd: parseFloat(document.getElementById('Kd').value),
        setPoint: parseFloat(document.getElementById('setPoint').value)
        };
        websocket.send(JSON.stringify(data));
        document.getElementById("myForm").reset();
        });

    </script>
</html> 

)HTMLHOMEPAGE";


/*        Server -> Client     */

void processCarMovement(String inputValue) {
  int inputValue_ = inputValue.toInt();
  //Move the Robot
  if (inputValue_ > 0) {
    ledcWrite(0, map(abs(inputValue_), 0, 100, 0, 255));  //(channel, dutycycle)L_PWM
    ledcWrite(1, 0);                                      //(channel, dutycycle)R_PWM
  } else if (inputValue_ < 0) {
    ledcWrite(0, 0);                                      //(channel, dutycycle)L_PWM
    ledcWrite(1, map(abs(inputValue_), 0, 100, 0, 255));  //(channel, dutycycle)R_PWM
  } else {
    ledcWrite(0, 0);  //(channel, dutycycle)L_PWM
    ledcWrite(1, 0);  //(channel, dutycycle)R_PWM
  }
}

void process_PID_parameter(double Kp_, double Ki_, double Kd_, double Setpoint_) {
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  Setpoint = Setpoint_;

  myPID.SetTunings(Kp, Ki, Kd);
  distanceValue = encToDistance(encoderValue);
  double difference = Setpoint - distanceValue;
  while (difference > 1 || difference < -1) {
    // time_ = millis();
    // ws.printfAll("{\"time_\": %.d}", time_);
    // ws.printfAll("{\"encoderValue\": %.2f}", distanceValue);
    out = PIDcontrol(Setpoint);
    if (out > 0) {  //forward
      ledcWrite(0, abs(out));
      ledcWrite(1, 0);
    } else if (out < 0) {  //reverse
      ledcWrite(0, 0);
      ledcWrite(1, abs(out));
    } else {  //stop both motors
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
    distanceValue = encToDistance(encoderValue);
    difference = Setpoint - distanceValue;
    // Serial.println(difference);
  }
      ledcWrite(0, 0);
      ledcWrite(1, 0);
}

void handleRoot(AsyncWebServerRequest *request) {
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "File Not Found");
}


void onWebSocketEvent(AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      //      client->text("Hello World");
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      processCarMovement("0");
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo *)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        std::string myData = "";
        myData.assign((char *)data, len);

        // Parse JSON data
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, myData);

        if (doc.containsKey("sliderValue")) {
          processCarMovement(doc["sliderValue"]);
        } else if (doc.containsKey("Kp") && doc.containsKey("Ki") && doc.containsKey("Kd") && doc.containsKey("setPoint")) {
          process_PID_parameter(doc["Kp"], doc["Ki"], doc["Kd"], doc["setPoint"]);
        } else {
          // Handle unsupported JSON tag
          Serial.println("Unsupported JSON tag");
        }
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}
/*        Main Code     */

void setup() {
  //Initialize Serial COM
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, BLE_RX, BLE_TX);
  // Set WiFi mode to an Acess point with the previously defined ssid and password
  WiFi.softAP(ssid, password);

  // WiFi.begin(ssid, password); 
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }
  // Serial.println("WiFi connected");

  //get and print IP Adress
  // IP = WiFi.localIP();
  IP = WiFi.softAPIP();

  Serial.print("AP IP address: ");
  Serial.println(IP);

  //to interface with the LCD display using I2C
  Wire.begin(SDA, SCL);
  lcd.begin();
  lcd.backlight();


  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.begin();
  Serial.println("HTTP server started");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP:" + IP.toString());
  lcd.setCursor(0, 1);
  lcd.print("Server started");

  //encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR, RISING);  //(Interrupt channel, function to call, condition for calling)

  //BTS7960 MOTOR CONTROL
  ledcAttachPin(L_PWM, 0);  // (pin, pwm channel)
  ledcAttachPin(R_PWM, 1);
  ledcSetup(0, 1000, 8);  // (channel,, freq, resolution)
  ledcSetup(1, 1000, 8);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  //Encoder pinMode
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  //Enable Motor to run
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  //Map the speedVal that we obtain from the webserver client to an 8 bit value
  checkSerial1();  //update PID parameters

  //update the encoderValue and speedVal to the webserver
  distanceValue = encToDistance(encoderValue);
  ws.printfAll("{\"encoderValue\": %.2f}", distanceValue);
  ws.printfAll("{\"disp_Kp\": %.2f}", Kp);
  ws.printfAll("{\"disp_Ki\": %.2f}", Ki);
  ws.printfAll("{\"disp_Kd\": %.2f}", Kd);
  ws.printfAll("{\"disp_setPoint\": %.2f}", Setpoint);
  //free resources of disconected clients
  ws.cleanupClients();
  delay(100);
}
