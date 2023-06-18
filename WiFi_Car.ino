#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <PID_v1.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "SPIFFS.h"

volatile int encoderValue = 0; // value of encoder
float distanceValue = 0.0;     // distance traveled in cm
// PID controller
int out = 0;
float time_;
double target = 100.0;
double Setpoint, Input, Output;
double Kp = 5, Ki = 0.8, Kd = 1.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// freeRTOS
TaskHandle_t updateTask;
TaskHandle_t serverTask;
TaskHandle_t PIDTask;

// Pin definitions

// for motor control
#define R_PWM 15
#define L_PWM 2
#define L_EN 0
#define R_EN 4
// for encoder
#define ENCA 22
#define ENCB 23
// for LCD
#define SDA 26
#define SCL 25
// for Bluetooth
#define BLE_RX 16
#define BLE_TX 17
// Dimension of the wheel & encoder pulse per rotation
#define CIR 45.553055 // Circumference of the wheel
#define PPR 900.0     // pulse per rotation

// create LCD object
LiquidCrystal_I2C lcd(0x3F, 16, 2);
// ip address
IPAddress IP;

// Json for websocket communication
DynamicJsonDocument doc(512);

/*        ENCODER     */

// ISR to count encoder pulse
void IRAM_ATTR ISR()
{

  if (digitalRead(ENCB) == digitalRead(ENCA))
  {
    // counterclockwise
    encoderValue--;
  }
  else
  {
    // clockwise
    encoderValue++;
  }
}
// Processing the encoder value to display in cm
float encToDistance(int encoderValue)
{
  // Serial.println((encoderValue/PPR)*CIR);
  return ((encoderValue / PPR) * CIR);
}
/*        Channge the PID parameters through serial     */

void checkSerial1()
{
  if (Serial1.available())
  {
    Serial.println("Serial data received");
    String data = Serial1.readStringUntil('\n');
    String values[4]; // creates an array of values
    int i = 0;
    while (i < 4)
    {
      int commaIndex = data.indexOf(',');
      if (commaIndex == -1)
      {
        values[i] = data;
        data = "";
      }
      else
      {
        values[i] = data.substring(0, commaIndex);
        data = data.substring(commaIndex + 1);
      }
      i++;
    }
    Kp = values[0].toFloat();
    Ki = values[1].toFloat();
    Kd = values[2].toFloat();
    Setpoint = values[3].toFloat();

    // confirmation through the lcd
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Data received:");
    lcd.setCursor(0, 1);
    lcd.print(String(Kp) + "," + String(Ki) + "," + String(Kd));
  }
}

/*        Server     */

// WiFi name and password

const char *ssid = "your_ssid";
const char *password = "your_password";


// set web server to use port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

/*        PID Control     */

// Function take in setpoint(target distance).
// Its Input is based on cm from distanceValue
// it returns Output (-255 to 255)

int PIDcontrol(double target)
{
  myPID.SetOutputLimits(-255.0, 255.0);
  myPID.SetSampleTime(50);
  Setpoint = target;
  Input = distanceValue;
  myPID.Compute();
  return Output;
}

/*        Server <-> Client     */

void processCarMovement(String inputValue)
{
  int inputValue_ = inputValue.toInt();
  // Move the Robot
  if (inputValue_ > 0)
  {
    ledcWrite(0, map(abs(inputValue_), 0, 100, 0, 255)); //(channel, dutycycle)L_PWM
    ledcWrite(1, 0);                                     //(channel, dutycycle)R_PWM
  }
  else if (inputValue_ < 0)
  {
    ledcWrite(0, 0);                                     //(channel, dutycycle)L_PWM
    ledcWrite(1, map(abs(inputValue_), 0, 100, 0, 255)); //(channel, dutycycle)R_PWM
  }
  else
  {
    ledcWrite(0, 0); //(channel, dutycycle)L_PWM
    ledcWrite(1, 0); //(channel, dutycycle)R_PWM
  }
}

void process_PID_parameter(double Kp_, double Ki_, double Kd_, double Setpoint_)
{
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  Setpoint = Setpoint_;

  // confirmation through the lcd
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Data received:");
  lcd.setCursor(0, 1);
  lcd.print(String(Kp) + "," + String(Ki) + "," + String(Kd));

  if (PIDTask != NULL)
  {
    vTaskResume(PIDTask);
    // Serial.println("PIDTask Started");
  }
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send(SPIFFS, "/website.html", "text/html");
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
    info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      std::string myData = "";
      myData.assign((char *)data, len);

      // Parse JSON data
      doc.clear();
      deserializeJson(doc, myData);

      if (doc.containsKey("sliderValue"))
      {
        processCarMovement(doc["sliderValue"]);
      }
      else if (doc.containsKey("Kp") && doc.containsKey("Ki") && doc.containsKey("Kd"))
      {
        Kp = doc["Kp"];
        Ki = doc["Ki"];
        Kd = doc["Kd"];
      }
      else if (doc.containsKey("setPoint"))
      {
        process_PID_parameter(Kp, Ki, Kd, doc["setPoint"]);
      }
      else
      {
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

/*        FreeRTOS Task     */

void updateTaskCode(void *parameter)
{
  for (;;)
  {
    vTaskDelay(300 / portTICK_PERIOD_MS);
    // update the encoderValue to the webserver
    distanceValue = encToDistance(encoderValue);
    ws.printfAll("{\"encoderValue\": %.2f}", distanceValue);
    ws.cleanupClients();
    checkSerial1(); // update PID parameters
  }
}
void serverTaskCode(void *parameter)
{
  // start server and websocket
  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.println("HTTP server started");
  for (;;)
  {  
    ws.printfAll("{\"disp_Kp\": %.2f, \"disp_Ki\": %.2f, \"disp_Kd\": %.2f, \"disp_setPoint\": %.2f}",Kp, Ki, Kd, Setpoint);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
void PIDTaskCode(void *parameter)
{
  while(1){
  Serial.println("PID Task Started");
  // PID code
  myPID.SetTunings(Kp, Ki, Kd);
  double difference = Setpoint - distanceValue;
  int lastMillis2 = 0;
  while (difference > 0.5 || difference < -0.5) 
  {
    distanceValue = encToDistance(encoderValue);
    difference = Setpoint - distanceValue;
    if (millis() > lastMillis2 + 300)
    {
      time_ = millis() / 1000;
      ws.printfAll("{\"encoderValue\": %.2f, \"time_\": %.2f}", distanceValue, time_);
      lastMillis2 = millis();
    }
    out = PIDcontrol(Setpoint);
    Serial.println(out);
    if (out > 0)
    { // forward
      ledcWrite(0, abs(out));
      ledcWrite(1, 0);
    }
    else if (out < 0)
    { // reverse
      ledcWrite(0, 0);
      ledcWrite(1, abs(out));
    }
    else
    { // stop both motors
      ledcWrite(0, 0);
      ledcWrite(1, 0);
    }
  }
  ledcWrite(0, 0);
  ledcWrite(1, 0);
  Serial.println("PID Task Done");

  vTaskSuspend(NULL);
  }
}

/*        Main Code     */

void setup()
{
  // Initialize Serial COM
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, BLE_RX, BLE_TX);

  // Set WiFi mode to an STA mode with the previously defined ssid and password
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("WiFi connected");

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // check if the file exist
  if (!SPIFFS.exists("/website.html"))
  {
    Serial.println("website file does not exist");
    return;
  }

  // get and print IP Adress
  IP = WiFi.localIP();
  // IP = WiFi.softAPIP();

  Serial.print("IP address: ");
  Serial.println(IP);

  // to interface with the LCD display using I2C
  Wire.begin(SDA, SCL);
  lcd.begin();
  lcd.backlight();

  // update lcd
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("IP:" + IP.toString());
  lcd.setCursor(0, 1);
  lcd.print("Server started");

  // encoder
  attachInterrupt(digitalPinToInterrupt(ENCA), ISR, RISING); //(Interrupt channel, function to call, condition for calling)

  // BTS7960 MOTOR CONTROL
  ledcAttachPin(L_PWM, 0); // (pin, pwm channel)
  ledcAttachPin(R_PWM, 1);
  ledcSetup(0, 1000, 8); // (channel,, freq, resolution)
  ledcSetup(1, 1000, 8);
  pinMode(L_EN, OUTPUT);
  pinMode(R_EN, OUTPUT);
  // Encoder pinMode
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);

  // Enable Motor to run
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);

  // turn the PID on
  myPID.SetMode(AUTOMATIC);

  //FreeRTOS Task
  xTaskCreatePinnedToCore(updateTaskCode, "updateTask", 5000, NULL, 1, &updateTask, 0);
  xTaskCreatePinnedToCore(PIDTaskCode, "PIDTask", 5000, NULL, 1, &PIDTask, 0);
  if (PIDTask != NULL)
  {
    vTaskSuspend(PIDTask);
    // Serial.println("Task Sucessfully suspended");
  }
  xTaskCreatePinnedToCore(serverTaskCode, "serverTask", 10000, NULL, 1, &serverTask, 1);
}

void loop()
{
  delay(1000);
}
