/* 
 * Ronnie Keating
 * Simple ESP32 Web Server
 * CS452
 * March 30, 2021
 * Code used from https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
 * for the web server setup. Implemented with FreeRTOS and a temp/humidity sensor
 * HDC1080 sensor. Displays current temp and humidity of wherever the ESP32 is located.
 */
#include <Arduino.h>
#include <WiFi.h>
#include "ClosedCube_HDC1080.h"

ClosedCube_HDC1080 temp_hum_sensor;
WiFiServer server(80);

// remove when upload
const char ssid[] = "-------";
const char passwd[] = "-------";

void TaskWebsite(void *pvParameters);
void TaskHDC(void *pvParameters);

QueueHandle_t HDCQueue;
SemaphoreHandle_t HDCSemaphore;

void setup()
{
  Serial.begin(115200);
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
    if(timeToReboot == 5) ESP.restart();
    timeToReboot++;
  }
  Serial.println("Connected");
  // need ip address to connect, DHCP usually gives the same one
  // but also dont have any new devices connecting
  // Should create a static IP but dont really want IP address
  // issues
  Serial.println(WiFi.localIP());

  // START SERVER
  server.begin();
  HDCQueue = xQueueCreate(2, sizeof(int));
  HDCSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(HDCSemaphore);
  xTaskCreatePinnedToCore(TaskWebsite, "Web Server", 2048, NULL, configMAX_PRIORITIES - 1, NULL, 1);
  xTaskCreatePinnedToCore(TaskHDC, "Temp/Humidity Sensor", 2048, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}

void loop()
{
}

void TaskWebsite(void *pvParameters)
{
  (void)pvParameters;
  String header;
  unsigned long currentTime = millis();
  unsigned long previousTime = 0;
  const long timeoutTime = 2000;  
  int temp = 0, humid = 0;
  for (;;)
  {
    WiFiClient client = server.available(); // Listen for incoming clients

    if (client)
    { // If a new client connects,
      currentTime = millis();
      previousTime = currentTime;
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

              // Display the HTML web page
              client.println("<!DOCTYPE html><html>");
              client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
              client.println("<link rel=\"icon\" href=\"data:,\">");
              // CSS to style the on/off buttons
              // Feel free to change the background-color and font-size attributes to fit your preferences
              client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
              client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
              client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
              client.println(".button2 {background-color: #555555;}</style></head>");

              // Website content
              client.println("<body><h1>ESP32 Web Server</h1>");
              client.print("<h2 style=\"color: red;\">Temperature: ");
              client.print(temp);
              client.println("</h2>");
              client.print("<h2 style=\"color: blue;\">Humidity: ");
              client.print(humid);
              client.println("</h2>");
              client.println("</body></html>");

              // The HTTP response ends with another blank line
              client.println();
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
      }
      // Clear the header variable
      header = "";
      // Close the connection
      client.stop();
      Serial.println("Client disconnected.");
      Serial.println("");
    }
    else // collect temp and humidity info if no client
    {
      xSemaphoreGive(HDCSemaphore);
      vTaskDelay(50);
      xSemaphoreTake(HDCSemaphore, portMAX_DELAY);
      xQueueReceive(HDCQueue, &temp, 0);
      xQueueReceive(HDCQueue, &humid, 0);
    }
  }
}

// I2C temp humidity task, avoids cutting off the I2C read
void TaskHDC(void *pvParameters)
{
  (void)pvParameters;
  int humid, temp;
  for(;;)
  {
    xSemaphoreTake(HDCSemaphore, portMAX_DELAY);
    temp = temp_hum_sensor.readTemperature();
    humid = temp_hum_sensor.readHumidity();
    xQueueSend(HDCQueue, &temp, portMAX_DELAY);
    xQueueSend(HDCQueue, &humid, portMAX_DELAY);
    xSemaphoreGive(HDCSemaphore);
    vTaskDelay(50);
  }
}