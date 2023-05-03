#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//Action taken
#define UP 1
#define DOWN 2
#define STOP 0

//motor movement action
#define FORWARD 1
#define BACKWARD -1

int speedVal = 0;               //0-10 from the webserver input
int motorPWM = 0;               //map speedVal from 0-255 for pwm magnitude
volatile int encoderValue = 0;  //value of encoder
float distanceValue = 0.0;      //distance traveled in cm

//PID controller
double target = 100.0;
double Setpoint, Input, Output;
double Kp = 1.0, Ki = 1.0, Kd = 1.0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
double difference = 0.0;

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

//Create task to pin to cores
TaskHandle_t Task1;
TaskHandle_t Task2;

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
    target = values[3].toFloat();
    Serial.print("New Parameters    : ");
    Serial.print(Kp);
    Serial.print(",");
    Serial.print(Ki);
    Serial.print(",");
    Serial.print(Kd);
    Serial.print(",");
    Serial.println(target);
    
    //confirmation through the lcd
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data received:");
    lcd.setCursor(0, 1);
    lcd.print(String(Kp)+","+String(Ki)+","+String(Kd)+",");
  }
}

/*        PID Control     */

//Function take in setpoint(target distance).
//Its Input is based on cm from distanceValue
//it returns Output (-255 to 255)

int PIDcontrol(double target) {
  myPID.SetOutputLimits(-255.0, 255.0);
  Setpoint = target;
  Input = distanceValue;
  myPID.Compute();
  // Serial.println(Output);
  return abs(Output);
}

/*        Server     */

//WiFi name and password
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
    .state {
     font-size: 1.5rem;
     color:#8c8c8c;
     font-weight: bold;
   }
    .arrows {
      font-size:70px;
      color:gainsboro;
    }
    td {
      background-color:teal;
      border-radius:25%;
      box-shadow: 5px 5px #888888;
    }
    td:active {
      transform: translate(5px,5px);
      box-shadow: none; 
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
     
    <h1 style="color: black;text-align:center;">Microcontroller Group 1</h1>
    <h2 style="color: black;text-align:center;">Wi-Fi &#128663; Control</h2>
    <p>Distance moved: <span id="encoderValue">0</span> cm</p>
    <p>Speed: <span id="speedVal">0</span></p>
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr>
        <td ontouchstart='onTouchStartAndEnd("1")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8679;</span></td>
        <td ontouchstart='onTouchStartAndEnd("3")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8679;</span></td>
      </tr>
      <tr>
        <td ontouchstart='onTouchStartAndEnd("5")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >Move 1m</span></td>
      </tr>
      
      <tr>
        <td ontouchstart='onTouchStartAndEnd("2")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8681;</span></td>
        <td ontouchstart='onTouchStartAndEnd("4")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8681;</span></td>
      </tr>
    </table>

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
            // Check if the received data contains the new encoder value
            if (data.encoderValue) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("encoderValue").innerHTML = data.encoderValue;
            }
            if (data.speedVal) {
                // Update the innerHTML property of the encoderValue element with the new value
                document.getElementById("speedVal").innerHTML = data.speedVal;
            }
        };
      }

      function onTouchStartAndEnd(value) 
      {
        websocket.send(value);
      }
          
      window.onload = initWebSocket;
      document.getElementById("mainTable").addEventListener("touchend", function(event){
        event.preventDefault()
      });      
    </script>
    
  </body>
</html> 

)HTMLHOMEPAGE";

/*        Server -> Client     */

void processCarMovement(String inputValue) {
  Serial.printf("Got value as %s %d\n", inputValue.c_str(), inputValue.toInt());
  switch (inputValue.toInt()) {

    case UP:
      ledcWrite(0, motorPWM);  //(channel, dutycycle)L_PWM
      ledcWrite(1, 0);         //(channel, dutycycle)R_PWM

      break;

    case DOWN:
      ledcWrite(0, 0);         //(channel, dutycycle)L_PWM
      ledcWrite(1, motorPWM);  //(channel, dutycycle)R_PWM

      break;

    case 3:
      if (speedVal < 10) {
        speedVal = speedVal + 1;
      }
      break;

    case 4:
      if (speedVal > 0) {
        speedVal = speedVal - 1;
      }
      break;
    case 5:
      myPID.SetTunings(Kp, Ki, Kd);
      // Serial.println("PID triggered");
      difference = target - distanceValue;
      while (difference > 1 || difference < -1) {
        // Serial.println("Inside the while loop");
        
        if (PIDcontrol(difference) > 0) {  //forward
          ledcWrite(0, PIDcontrol(target));
          ledcWrite(1, 0);
        } else if (PIDcontrol(difference) < 0) {  //reverse
          ledcWrite(0, 0);
          ledcWrite(1, PIDcontrol(target));
        } else {  //stop both motors
          ledcWrite(0, 0);
          ledcWrite(1, 0);
        }
        distanceValue = encToDistance(encoderValue);
        difference = target - distanceValue;
      }
      break;

    case STOP:
      ledcWrite(0, 0);  //(channel, dutycycle)L_PWM
      ledcWrite(1, 0);  //(channel, dutycycle)R_PWM

      break;

    default:
      ledcWrite(0, 0);  //(channel, dutycycle)L_PWM
      ledcWrite(1, 0);  //(channel, dutycycle)R_PWM

      break;
  }
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
        processCarMovement(myData.c_str());
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
  //get and print IP Adress
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

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  // xTaskCreatePinnedToCore(
  //                   Task1code,   /* Task function. */
  //                   "Task1",     /* name of task. */
  //                   10000,       /* Stack size of task */
  //                   NULL,        /* parameter of the task */
  //                   1,           /* priority of the task */
  //                   &Task1,      /* Task handle to keep track of created task */
  //                   0);          /* pin task to core 0 */
  // delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}


// void Task1code( void * pvParameters ){
//   Serial.print("Task1 running on core ");
//   Serial.println(xPortGetCoreID());
// for(;;){

// }
// }

void Task2code(void *pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    //Map the speedVal that we obtain from the webserver client to an 8 bit value
    motorPWM = map(speedVal, 0, 10, 0, 255);
    checkSerial1();  //update PID parameters

    // Serial.println(encoderValue);
    // Serial.println(motorPWM);
    //update the encoderValue and speedVal to the webserver
    distanceValue = encToDistance(encoderValue);
    ws.printfAll("{\"encoderValue\": %.2f}", distanceValue);
    ws.printfAll("{\"speedVal\": %d}", speedVal);
    //free resources of disconected clients
    ws.cleanupClients();
    delay(100);
  }
}

void loop() {
}
