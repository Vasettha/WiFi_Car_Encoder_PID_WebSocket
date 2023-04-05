#include <Arduino.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

//motor numbering
#define MOTOR 0

//Action taken
#define UP 1
#define DOWN 2
#define STOP 0

//motor movement action
#define FORWARD 1
#define BACKWARD -1

int speedVal =0;//0-10 from the webserver input
int motorPWM =0;//map speedVal from 0-255 for pwm magnitude
volatile int encoderValue = 0;//value of encoder
volatile int lastMillis = 0;
// Pin definitions
//for motor control
#define R_PWM 15
#define L_PWM 2
#define L_EN  0
#define R_EN  4
//for encoder
#define ENCA 12
#define ENCB 14
//Dimension of the wheel & encoder pulse per rotation
#define CIR  0 //Circumference of the wheel
#define PPR  0 //pulse per rotation
#


//Create task to pin to cores
TaskHandle_t Task1;
TaskHandle_t Task2;

// portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;

//ISR to count encoder pulse
void IRAM_ATTR ISR() {

  // portENTER_CRITICAL(&synch);
  if (millis()-lastMillis > 1){// To debounce
    if (digitalRead(ENCB) == digitalRead(ENCA)){
      //counterclockwise
      encoderValue--;
      }
    else{
      //clockwise
      encoderValue++;
      }
      lastMillis = millis();
  } 
    // portEXIT_CRITICAL(&synch);
  
}
//Processing the encoder value to display in cm
int encToDistance(int encoderValue)
{
  return (encoderValue/PPR)*CIR  ;
}


//motor pins struct
struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;    
};

std::vector<MOTOR_PINS> motorPins = 
{
  {R_PWM, L_PWM},  //MOTOR
};

//WiFi name and password
const char* ssid     = "Group1";
const char* password = "12345678";

//set web server to use port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//HTML page shown to the client
const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
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
      color:red;
    }
    .circularArrows {
      font-size:80px;
      color:blue;
    }
    td {
      background-color:black;
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
  <body class="noselect" align="center" style="background-color:white">
     
    <h1 style="color: teal;text-align:center;">Microcontroller Group 1</h1>
    <h2 style="color: teal;text-align:center;">Wi-Fi &#128663; Control</h2>
    <p>Rotary Encoder Value: <span id="encoderValue">0</span></p>
    <p>Speed: <span id="speedVal">0</span></p>
    <table id="mainTable" style="width:400px;margin:auto;table-layout:fixed" CELLSPACING=10>
      <tr>
        <td ontouchstart='onTouchStartAndEnd("1")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8679;</span></td>
        <td ontouchstart='onTouchStartAndEnd("3")' ontouchend='onTouchStartAndEnd("0")'><span class="arrows" >&#8679;</span></td>
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

//To handle moving motor forward and backward
void rotateMotor(int motorNumber, int motorDirection)
{
  if (motorDirection == FORWARD)
  { 
    ledcWrite(0, motorPWM);//(channel, dutycycle)L_PWM
    ledcWrite(1, 0);//(channel, dutycycle)R_PWM

  }
  else if (motorDirection == BACKWARD)
  {
    ledcWrite(0, 0);//(channel, dutycycle)L_PWM
    ledcWrite(1, motorPWM);//(channel, dutycycle)R_PWM
  }
  else
  {
    ledcWrite(0, 0);//(channel, dutycycle)L_PWM
    ledcWrite(1, 0);//(channel, dutycycle)R_PWM     
  }
}

//Actions triggered from websocket movement buttons
void processCarMovement(String inputValue)
{
  Serial.printf("Got value as %s %d\n", inputValue.c_str(), inputValue.toInt());  
  switch(inputValue.toInt())
  {

    case UP:
      rotateMotor(MOTOR, FORWARD);
                
      break;
  
    case DOWN:
      rotateMotor(MOTOR, BACKWARD);
 
      break;

    case 3:
    if (speedVal < 10){
       speedVal = speedVal +1;
    }
      break;

    case 4:
    if (speedVal > 0){
       speedVal = speedVal -1;
    }
      break;

  
    case STOP:
      rotateMotor(MOTOR, STOP);
  
      break;
  
    default:
      rotateMotor(MOTOR, STOP);
  
      break;
  }
}

void handleRoot(AsyncWebServerRequest *request) 
{
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request) 
{
    request->send(404, "text/plain", "File Not Found");
}


void onWebSocketEvent(AsyncWebSocket *server, 
                      AsyncWebSocketClient *client, 
                      AwsEventType type,
                      void *arg, 
                      uint8_t *data, 
                      size_t len) 
{                      
  switch (type) 
  {
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
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) 
      {
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

  //MAIN CODE

}

void setup() {
  //Initialize Serial COM
  Serial.begin(115200); 
  // Set WiFi mode to an Acess point with the previously defined ssid and password
  WiFi.softAP(ssid, password);
  //get and print IP Adress
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  server.begin();
  Serial.println("HTTP server started");

  //encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR, RISING);

  //BTS7960 MOTOR CONTROL
  ledcAttachPin(L_PWM, 0);// (pin, pwm channel)
  ledcAttachPin(R_PWM, 1);
  ledcSetup(0, 1000, 8);// (channel,, freq, resolution)
  ledcSetup(1, 1000, 8);
  
  pinMode(L_EN,OUTPUT);
  pinMode(R_EN,OUTPUT);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  
  //Enable Motor to run
 digitalWrite(L_EN,HIGH);
 digitalWrite(R_EN,HIGH);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}


void Task1code( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
for(;;){
  
delay(1);
} 
}

void Task2code( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
for(;;){
  //Map the speedVal that we obtain from the webserver client to an 8 bit value 
  motorPWM = map(speedVal,0,10,0,255);
  
  Serial.println(encoderValue);
  Serial.println(motorPWM);
  //update the encoderValue and speedVal to the webserver
  ws.printfAll("{\"encoderValue\": %d}", encToDistance(encoderValue));
  ws.printfAll("{\"speedVal\": %d}", speedVal);
  //free resources of disconected clients
  ws.cleanupClients(); 
  delay(100);
}
}

void loop() {
  
}
