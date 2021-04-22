/* 
 * Ronnie Keating
 * Connect to Web Server
 * CS452
 * April 6, 2021
 * Code used from https://randomnerdtutorials.com/esp32-http-get-post-arduino/#http-post
 * Connects to web server using JSON
 */

// libraries
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <string.h>
#include "Stepper.h"
#include "ClosedCube_HDC1080.h"
#include "rgb_lcd.h"
#include "PixelFunctions.h"
#ifdef __AVR__
#include <avr/power.h>
#endif

// pins
#define IN1 33
#define IN2 4
#define IN3 32
#define IN4 14
#define PIN 21
#define DIP0 34
#define DIP1 39
#define DIP2 36
#define DIP3 15
#define DIP4 26

// definitions
#define ONE_SEC 1000 / portTICK_PERIOD_MS
#define ONE_MIN ONE_SEC * 60
#define NUM_STEPS_PER_REV 2048
#define MOTOR_SPEED 4
#define NUM_LEDS 4
#define BRIGHTNESS 50

// wifi server
WiFiServer server(80);

// stepper motor (switch IN2 & IN3 for counter)
Stepper step_motor(NUM_STEPS_PER_REV, IN1, IN3, IN2, IN4);

// I2C objects
ClosedCube_HDC1080 temp_hum_sensor;
rgb_lcd lcd;

// all possible options for server commands
enum IOTAPICommands
{
  ping,
  login,
  query,
  data,
  shutdown
};

enum RGBCommands
{
  still,
  blink,
  rainbw
};

// wifi connection
const char ssid[] = "----";
const char passwd[] = "----";

// server locations
const char detectServer[] = "http://13.83.132.121:5000//IOTAPI/DetectServer";
const char registerServer[] = "http://13.83.132.121:5000//IOTAPI/RegisterWithServer";
const char commandServer[] = "http://13.83.132.121:5000//IOTAPI/QueryServerForCommands";
const char dataServer[] = "http://13.83.132.121:5000//IOTAPI/IOTData";
const char shutdownServer[] = "http://13.83.132.121:5000//IOTAPI/IOTShutdown";

// tasks
void TaskMain(void *pvParameters);
void TaskWebsite(void *pvParameters);
void TaskHDC(void *pvParameters);

// queues and semaphores
QueueHandle_t HDCQueue;
QueueHandle_t AuthCodeQueue;
SemaphoreHandle_t HDCSemaphore;

// functions
void controlLED(int r, int g, int b, int w, RGBCommands command)
{
  strand_t *pStrand = &STRANDS[0];
  switch (command)
  {
  case still:
    for (uint16_t i = 0; i < pStrand->numPixels; i++)
      pStrand->pixels[i] = pixelFromRGBW(r, g, b, w);
    digitalLeds_updatePixels(pStrand);
    break;
  case blink:
    break;
  case rainbw:
    rainbow(pStrand, 0, 2000);
    break;
  }
}

void hostWebsite(unsigned long currentTime, unsigned long previousTime, long timeoutTime, String header, WiFiClient client)
{
  currentTime = millis();
  previousTime = currentTime;
  RGBCommands command = still;
  Serial.println("New Client."); // print a message out in the serial port
  String currentLine = "";       // make a String to hold incoming data from the client
  while (client.connected() && currentTime - previousTime <= timeoutTime)
  { // loop while the client's connected
    currentTime = millis();
    if (client.available())
    {                         // if there's bytes to read from the client,
      char c = client.read(); // read a byte, then
      Serial.write(c);        // print it out the serial monitor
      header += c;
      if (c == '\n')
      { // if the byte is a newline character
        // if the current line is blank, you got two newline characters in a row.
        // that's the end of the client HTTP request, so send a response:
        if (currentLine.length() == 0)
        {
          // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
          // and a content-type so the client knows what's coming, then a blank line:
          client.println("HTTP/1.1 200 OK");
          client.println("Content-type:text/html");
          client.println("Connection: close");
          client.println();

          // turns the GPIOs on and off
          if (header.indexOf("GET /clockwise/on") >= 0)
          {
            controlLED(0, 255, 0, 0, command);
            step_motor.step(NUM_STEPS_PER_REV / 4);
            controlLED(0, 0, 0, 0, command);
          }
          else if (header.indexOf("GET /counter/on") >= 0)
          {
            controlLED(255, 0, 0, 0, command);
            step_motor.step(-NUM_STEPS_PER_REV / 4);
            controlLED(0, 0, 0, 0, command);
          }

          // Display the HTML web page
          client.println("<!DOCTYPE html><html>");
          client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
          client.println("<link rel=\"icon\" href=\"data:,\">");
          // CSS to style the on/off buttons
          // Feel free to change the background-color and font-size attributes to fit your preferences
          client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
          client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
          client.println("text-decoration: none; font-size: 15px; margin: 2px; cursor: pointer;}");
          client.println(".button2 {background-color: #FF0000; border: none; color: white; padding: 16px 40px;");
          client.println("text-decoration: none; font-size: 15px; margin: 2px; cursor: pointer;}</style></head>");

          // Web Page Heading
          client.println("<body><h1>ESP32 Web Server</h1>");
          client.println("<p>Stepper Motor Controller (Moves 90 degrees)</p>");
          // If the output26State is off, it displays the ON button
          client.println("<p><a href=\"/clockwise/on\"><button class=\"button\">Clockwise</button></a></p>");

          // Display current state, and ON/OFF buttons for GPIO 27
          // If the output27State is off, it displays the ON button
          client.println("<p><a href=\"/counter/on\"><button class=\"button2\">Counter</button></a></p>");
          client.println("</body></html>");

          // The HTTP response ends with another blank line
          client.println();
          // Break out of the while loop
          break;
        }
        else
        { // if you got a newline, then clear currentLine
          currentLine = "";
        }
      }
      else if (c != '\r')
      {                   // if you got anything else but a carriage return character,
        currentLine += c; // add it to the end of the currentLine
      }
    }
    taskYIELD();
  }
  // Clear the header variable
  header = "";
  // Close the connection
  client.stop();
  Serial.println("Client disconnected.");
  Serial.println("");
}

void serverFunctions(IOTAPICommands command, char *auth_code)
{
  HTTPClient http;
  int httpResponseCode;
  String response;
  String value, temp_str;
  double temp = 0, humid = 0;
  //Check WiFi connection status
  if (WiFi.status() == WL_CONNECTED)
  {
    switch (command)
    {
    case ping:
      http.begin(detectServer);
      http.addHeader("Content-Type", "application/json");
      httpResponseCode = http.POST("{\"key\": \"2436e8c114aa64ee\"}");
      response = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.println(response);
      http.end();
      if (httpResponseCode == -11)
        serverFunctions(ping, auth_code);
      break;

    case login:
      http.begin(registerServer);
      http.addHeader("Content-Type", "application/json");
      httpResponseCode = http.POST("{\"key\": \"2436e8c114aa64ee\", \"iotid\": 1010}");
      response = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.println(response);
      http.end();
      xQueueSend(AuthCodeQueue, &response, portMAX_DELAY);
      if (httpResponseCode == -11)
        serverFunctions(login, auth_code);
      break;

    case query:
      break;

    case data:
      xSemaphoreGive(HDCSemaphore);
      vTaskDelay(ONE_SEC);
      xSemaphoreTake(HDCSemaphore, portMAX_DELAY);
      xQueueReceive(HDCQueue, &temp, 1);
      xQueueReceive(HDCQueue, &humid, 1);
      value = "{\"auth_code\": \"";
      value += auth_code;
      temp_str = "\", \"temperature\": ";
      value += temp_str;
      temp_str = temp;
      value += temp_str;
      temp_str = ", \"humidity\": ";
      value += temp_str;
      temp_str = humid;
      value += temp_str;
      temp_str = ", \"light\": 1.2345, \"time\": \"2008-01-01 00:00:01\" }";
      value += temp_str;
      http.begin(dataServer);
      http.addHeader("Content-Type", "application/json");
      httpResponseCode = http.POST(value);
      response = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.println(response);
      http.end();
      break;

    case shutdown:
      break;

    default:
      break;
    }
  }
  else
  {
    Serial.println("WiFi Disconnected");
  }
}

void setup()
{
  Serial.begin(115200);
  // init sensor with address
  temp_hum_sensor.begin(0x40);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(ssid, passwd);
  // reboots if wifi doesnt connect fast enough
  // usually doesnt connect the first boot anyways, dont know why
  int timeToReboot = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
    // reboot here
    if (timeToReboot == 5)
      ESP.restart();
    timeToReboot++;
  }
  Serial.println("Connected");
  if (digitalLeds_initStrands(STRANDS, STRANDCNT))
  {
    Serial.println("Init FAILURE: halting");
    while (true)
    {
    };
  }
  // need ip address to connect, DHCP usually gives the same one
  // but also dont have any new devices connecting
  // Should create a static IP but dont really want IP address
  // issues
  Serial.println(WiFi.localIP());
  // hdc senesor queue and semaphore for control and communication
  HDCQueue = xQueueCreate(2, sizeof(double));
  AuthCodeQueue = xQueueCreate(1, sizeof(String));
  HDCSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(HDCSemaphore);

  // start wifi server
  server.begin();

  // motor stuff
  step_motor.setSpeed(MOTOR_SPEED);

  // dip and buttons
  pinMode(DIP0, INPUT);
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  pinMode(DIP3, INPUT);
  pinMode(DIP4, INPUT);


  xTaskCreatePinnedToCore(TaskWebsite, "Web Server", 4096, NULL, configMAX_PRIORITIES - 3, NULL, 1);
  xTaskCreatePinnedToCore(TaskHDC, "Temp/Humidity Sensor", 2048, NULL, configMAX_PRIORITIES - 24, NULL, 1);
  xTaskCreatePinnedToCore(TaskMain, "Main function", 8192, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

void loop()
{
}

void TaskMain(void *pvParameters)
{
  (void)pvParameters;
  String header;
  unsigned long currentTime = millis();
  unsigned long previousTime = 0;
  const long timeoutTime = 2000;
  for (;;)
  {
    WiFiClient client = server.available(); // Listen for incoming clients
    if (client && digitalRead(DIP0) == 0)
      hostWebsite(currentTime, previousTime, timeoutTime, header, client);
    else if(digitalRead(DIP3) == 0)
    {
      controlLED(0, 0, 0, 0, rainbw);
    }
    else vTaskDelay(5);
  }
}

void TaskWebsite(void *pvParameters)
{
  (void)pvParameters;
  char auth_code[30];
  int i = 1, j = 0;
  RGBCommands command = still;
  String response;
  xSemaphoreTake(HDCSemaphore, portMAX_DELAY);
  serverFunctions(login, auth_code);
  xQueueReceive(AuthCodeQueue, &response, portMAX_DELAY);
  while(response[i] != ':') i++;
  i+=2;
  while(response[i] != '\"')
  {
    auth_code[j] = response[i];
    i++;
    j++;
  }
  auth_code[j] = '\0';
  for (;;)
  {
    vTaskDelay(ONE_MIN * 5);
    if(digitalRead(DIP2) == 0)
    {
      controlLED(0, 0, 0, 255, command);
      serverFunctions(data, auth_code);
      controlLED(0, 0, 0, 0, command);
    }
  }
}

// I2C temp humidity task, avoids cutting off the I2C read
void TaskHDC(void *pvParameters)
{
  (void)pvParameters;
  double humid, temp;
  for (;;)
  {
    xSemaphoreTake(HDCSemaphore, portMAX_DELAY);
    temp = temp_hum_sensor.readTemperature();
    humid = temp_hum_sensor.readHumidity();
    // add values to queue
    xQueueSend(HDCQueue, &temp, portMAX_DELAY);
    xQueueSend(HDCQueue, &humid, portMAX_DELAY);
    xSemaphoreGive(HDCSemaphore);
    vTaskDelay(ONE_SEC);
  }
}