/* Ronald Keating
 * CS452
 * Assignment 8
 * Stepper Motor
*/

#include <Arduino_FreeRTOS.h>
#include <Arduino.h>
#include <queue.h>
#include <semphr.h>
#include <string.h>
#include "avr/power.h"
#include "Adafruit_NeoPixel.h"
#include "ClosedCube_HDC1080.h" // Humidity/Temperature library
#include "Stepper.h" // stepper motor library
#include "Adafruit_ADS1015.h"

// Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
Adafruit_ADS1015 ads;     /* Use thi for the 12-bit version */

#define LED_BUILTIN 13

// segment sides pins
#define SevenSegCC1 44 // right
#define SevenSegCC2 46 // left

// segment sections pins                  
#define SevenSegA 4 // top                 A
#define SevenSegB 5 // top right         F   B
#define SevenSegC 6 // bottom right        G
#define SevenSegD 7 // bottom            E   C
#define SevenSegE 8 // bottom left         D   DP
#define SevenSegF 9 // top left
#define SevenSegG 10 // middle
#define SevenSegDP 11 // decimal

// dip switches
#define DIP0 53
#define DIP1 51
#define DIP2 49
#define DIP3 47
#define DIP4 45
#define DIP5 43
#define DIP6 41
#define DIP7 39

// three buttons, button 3 doesnt work very well
#define BUTTONS1 2
#define BUTTONS2 28
#define BUTTONS3 29

// just some extra definitions I included
#define FRAME_RATE 2/3
#define ONE_SEC 1000 / portTICK_PERIOD_MS
#define HALF_SEC 1000 / portTICK_PERIOD_MS / 2
#define ONE_TICK 1
#define ON 1
#define OFF 0
#define MOTOR_SPEED 6
#define NUM_LEDS 4
#define BRIGHTNESS 50

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, 24, NEO_GRBW + NEO_KHZ800);
// class to use the temp and humidity sensor
ClosedCube_HDC1080 temp_hum_sensor;

// number of steps per revolution
#define NUM_STEPS_PER_REV 2048

// stepper motor, IN1-4 go to 22, 24, 26, 28 respectively
// it may be weird that 26 and 24 are flipped below, but 
// it fixes the motor so that it goes backwards as well
Stepper step_motor(NUM_STEPS_PER_REV, 26, 30, 34, 32);

// define two tasks for Blink & AnalogRead
void TaskSevenSeg(void *pvParamters);
void TaskMain(void *pvParameters);
void TaskBlink(void *pvParameters);
void TaskStepperMotor(void *pvParameters);
void TaskRGB(void *pvParameters);

// three seperate queues, for each side of the seven seg and the blink task
QueueHandle_t LeftQueue;
QueueHandle_t RightQueue;
QueueHandle_t BlinkQueue;
QueueHandle_t LEDQueue;
QueueHandle_t LEDQueue2;


// additional queue for stepper task communications
QueueHandle_t StepperQueue;

SemaphoreHandle_t DisplaySemaphore;
SemaphoreHandle_t LEDSemaphore;

TaskHandle_t LeftTask_Handle;
TaskHandle_t RightTask_Handle;

// the setup function runs once when you press reset or power the board
void setup()
{
  // begin serial monitor
  Serial.begin(115200);
  // start reading from Temperature/Humidity sensor
  temp_hum_sensor.begin(0x40);
  // initialize pin mode
  pinMode(SevenSegA, OUTPUT);
  pinMode(SevenSegB, OUTPUT);
  pinMode(SevenSegC, OUTPUT);
  pinMode(SevenSegD, OUTPUT);
  pinMode(SevenSegE, OUTPUT);
  pinMode(SevenSegF, OUTPUT);
  pinMode(SevenSegG, OUTPUT);
  pinMode(SevenSegDP, OUTPUT);

  // almost forgot to turn on pin 13 led
  pinMode(LED_BUILTIN, OUTPUT);

  // seven seg init
  pinMode(SevenSegCC1, OUTPUT);
  pinMode(SevenSegCC2, OUTPUT);

  // dip switches
  pinMode(DIP0, INPUT);
  pinMode(DIP1, INPUT);
  pinMode(DIP2, INPUT);
  pinMode(DIP3, INPUT);
  // only need to read from first 4
  pinMode(DIP4, INPUT);
  pinMode(DIP5, INPUT);
  pinMode(DIP6, INPUT);
  pinMode(DIP7, INPUT);

  // setup buttons
  pinMode(BUTTONS1, INPUT);
  pinMode(BUTTONS2, INPUT);
  pinMode(BUTTONS3, INPUT);
  step_motor.setSpeed(MOTOR_SPEED); // set the speed of the stepper
  // 4 for each rotation being 15 seconds

  ads.begin();

  // three queues: left and right seven seg and blink queue
  LeftQueue = xQueueCreate(5, sizeof(int)); // size of 5
  RightQueue = xQueueCreate(5, sizeof(int)); // + size of 5 = controlqueue of 10 (had bouncing numbers with one queue)
  StepperQueue = xQueueCreate(2, sizeof(int)); // stepper queue that should probably be of size 1
  BlinkQueue = xQueueCreate(1, sizeof(int)); // not using this currently
  LEDQueue = xQueueCreate(1, sizeof(int));
  LEDQueue2 = xQueueCreate(14, sizeof(int));

  DisplaySemaphore = xSemaphoreCreateBinary(); // main task completely controls seven seg with this
  xSemaphoreGive(DisplaySemaphore); // gotta give here even tho I do so in main task? works atleast...
  LEDSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(LEDSemaphore);

  // check all queues where made successfully, didnt include stepper queue
  if(BlinkQueue != NULL && LeftQueue != NULL && RightQueue != NULL)
  { 
    xTaskCreate(TaskMain, "Highest priority task, main", 128, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(TaskBlink, "Blink D13", 128, NULL, 0, NULL);
    xTaskCreate(TaskSevenSeg, "Left 7 Seg Display", 128, NULL, 1, &LeftTask_Handle);
    xTaskCreate(TaskStepperMotor, "Stepper motor task", 256, NULL, 3, NULL);
    xTaskCreate(TaskRGB, "RGB LED task", 256, NULL, 3, NULL);
  }
  strip.setBrightness(50);
  strip.begin();
  strip.show();
  vTaskStartScheduler();
}

// everything is done in tasks
void loop()
{
  // i guess you do need this
}

/*--------------------------------------------------*/
/*-------------------- FUNCTIONS -------------------*/
/*--------------------------------------------------*/

// prints the numbers, will add hex in the future
void numbers(int num, int ON_OFF)
{
  // hopefully this makes sense to anyone
  switch (num)
  {
  case 0: // zero
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    break;
  case 1: // 1
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    break;
  case 2: // 2
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    break;
  case 3: // 3
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    break;
  case 4: // 4
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    break;
  case 5: // 5
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    break;
  case 6: // 6
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 7: // 7
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    break;
  case 8: // 8
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 9: // 9
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 10: // A
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 11: // b (lowercase cuz 8)
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 12: // C
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    break;
  case 13: // d (lowercase cuz 0)
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 14: // E
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 15: // F
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 16: // backwards C, counter clock wise
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegD, ON_OFF);
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    break;
  case 17: // H for humidity
    digitalWrite(SevenSegB, ON_OFF);
    digitalWrite(SevenSegC, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
    digitalWrite(SevenSegG, ON_OFF);
    break;
  case 18: // weird looking t for temperature
    digitalWrite(SevenSegA, ON_OFF);
    digitalWrite(SevenSegE, ON_OFF);
    digitalWrite(SevenSegF, ON_OFF);
  default:
    break;
  }
}

void pixelClear()
{
  for(int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  strip.show();
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3,0);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3,0);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0,0);
}

void fullWhite() {
  
    for(uint16_t i=0; i<strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
    }
      strip.show();
}


void whiteOverRainbow(TickType_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) {
  
  if(whiteLength >= strip.numPixels()) whiteLength = strip.numPixels() - 1;

  int head = whiteLength - 1;
  int tail = 0;

  int loops = 3;
  int loopNum = 0;

  static unsigned long lastTime = 0;


  while(true){
    for(int j=0; j<256; j++) {
      for(uint16_t i=0; i<strip.numPixels(); i++) {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) ){
          strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
        }
        else{
          strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
        }
        
      }

      if(millis() - lastTime > whiteSpeed) {
        head++;
        tail++;
        if(head == strip.numPixels()){
          loopNum++;
        }
        lastTime = millis();
      }

      if(loopNum == loops) return;
    
      head%=strip.numPixels();
      tail%=strip.numPixels();
        strip.show();
        vTaskDelay(wait);
    }
  }
  
}

// Slightly different, this makes the rainbow equally distributed throughout
int rainbowCycle(TickType_t wait) {
  uint16_t i, j;

  for(j=0; j<256 * 5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      if(digitalRead(DIP0) == ON) return 0;
      strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    vTaskDelay(wait);
  }
}

void rainbow(TickType_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, Wheel((i+j) & 255));
    }
    strip.show();
    vTaskDelay(wait);
  }
}

// function to print
void printLR(int side, int off_side, int num)
{
    digitalWrite(off_side, LOW);
    digitalWrite(side, HIGH);
    numbers(num, HIGH);
    vTaskDelay(FRAME_RATE);
    numbers(num, LOW);
}

void segManager(int lNum, int rNum)
{
  xQueueSend(LeftQueue, &lNum, portMAX_DELAY);
  xQueueSend(RightQueue, &rNum, portMAX_DELAY);
}

void checkQueueIsFull(int test)
{
  //--------------------------------------
  // TEST: This is just to show that hex works and also fills number queue
  // because number queue never gets full...
  if(test == 0)
  {
    for(int i = 0; i <= 15; i++)
    {
      //xQueueSend(RightQueue, &i, portMAX_DELAY); // (to go back to 2 queues)
      for(int j = 0; j <= 15; j++)
      {
        // round robin effect here, one task gets to the queue first
        // because they are the same priority
        // this is in each instance of adding to the queue
        segManager(i, j);
        vTaskDelay(HALF_SEC/4);
      }
    }
    for(int i = 0; i < 5; i++) segManager(11, 11);
  }
  //--------------------------------------

  // check to see if queue is full, if so reset the queue(s)
  if(xQueueIsQueueFullFromISR(LeftQueue) == pdTRUE) // if left is full, so is right
  {
    xQueueReset(LeftQueue);
    xQueueReset(RightQueue);
    if(xQueueIsQueueEmptyFromISR(LeftQueue) == pdTRUE)
    {
      Serial.println("Queue emptied");
    }
    xSemaphoreGive(DisplaySemaphore);
    segManager(0, 15);
    vTaskDelay(ONE_SEC * 5);
    xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
  }
  // this is the part of the assignment where I have to 
  // display message for X seconds, X being 3 seconds in this case
  else 
  {
    xSemaphoreGive(DisplaySemaphore);
    vTaskDelay(ONE_SEC * 3);
    xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
    //vTaskDelay(ONE_SEC * 3);
  }
}

int adsValues(int controller, int adsVal)
{
  // return brightness value
  if(adsVal == 0)
    return 0;
  else if(controller == 0)
  {
    return 1660/17/100; // ADS turn thing; divide by 17 to get a number ~ between 100. Divide by 100 to get percentage
  }
  // return color value
  else
  {
    return 1660/4; // returns color value, have to do comparison later
  }
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

// This is the driver of the system
// It uses two queues to transfer data back and forth from the two left and right
// seven segment displays.
// 
// This function I feel will be useful for later in the semester, feels 
// moduler enough
void TaskMain(void *pvParameters)
{
  (void)pvParameters;
  int test = 1;
  int stepCount = 0;
  int16_t adc = 0;
  int command = 0;
  int color_bright[13];
  int tmp;
  // initialize
  color_bright[0] = 0;
  color_bright[1] = 0;
  color_bright[2] = 255;
  color_bright[3] = 0;
  color_bright[4] = 0;
  color_bright[5] = 255;
  color_bright[6] = 0;
  color_bright[7] = 0;
  color_bright[8] = 255;
  color_bright[9] = 0;
  color_bright[10] = 0;
  color_bright[11] = 255;
  int temp, humid; // temp and humidity variables, need address to pass to queue
  int i = 0;
  for(;;)
  {
    // state 5 - turn everything off, could vtaskdelay to save power...
    if(digitalRead(DIP3) == OFF)
    {
      pixelClear();
      // dont do anything with DIP3 on
      if(digitalRead(BUTTONS1) == ON)
      {
        vTaskDelay(5);
      // button 3 doesnt work very well...
        while(digitalRead(DIP3) == OFF)
        {
          xQueueReset(LEDQueue2);
          // edit each pixel, dip switches correspond to them
          if(digitalRead(DIP0) == OFF)
          {
            command = 1;
          }
          else if(digitalRead(DIP0) == ON)
          {
            command = 0;
          }

          // enter specific led prompts
          if(command == 0)
          {
            adc = ads.readADC_SingleEnded(3);
            // control brightness
            if(digitalRead(DIP1) == OFF)
            {
              // pixel 1: array values 0-2
              if(digitalRead(DIP4) == OFF)
              {
                for(int i = 0; i < 3; i++)
                {
                  if(color_bright[i] > 0)
                    color_bright[i] = adc/7 + 1;
                }
              }
              // pixel 2: array values 3-5
              else if(digitalRead(DIP5) == OFF)
              {
                for(int i = 3; i < 6; i++)
                {
                  if(color_bright[i] > 0)
                    color_bright[i] = adc/7 + 1;
                }
              }
              // pixel 3: array values 6-8
              else if(digitalRead(DIP6) == OFF)
              {
                for(int i = 6; i < 9; i++)
                {
                  if(color_bright[i] > 0)
                    color_bright[i] = adc/7 + 1;
                }
              }
              // pixel 4: array values 9-11
              else if(digitalRead(DIP7) == OFF)
              {
                for(int i = 9; i < 12; i++)
                {
                  if(color_bright[i] > 0)
                    color_bright[i] = adc/7 + 1;
                }
              }
            }
            // control color
            else if(digitalRead(DIP2) == OFF)
            {
              // pixel 1: array values 0-2
              if(digitalRead(DIP4) == OFF)
              {
                for(int i = 0; i < 3; i++)
                {
                  if(color_bright[i] > 0)
                    tmp = color_bright[i];
                }
                if(adc < 415)
                {
                  color_bright[0] = tmp;
                  color_bright[1] = 0;
                  color_bright[2] = 0; 
                }
                else if(adc < 830)
                {
                  color_bright[0] = 0;
                  color_bright[1] = tmp;
                  color_bright[2] = 0; 
                }
                else
                {
                  color_bright[0] = 0;
                  color_bright[1] = 0;
                  color_bright[2] = tmp; 
                }
              }
              // pixel 2: array values 3-5
              else if(digitalRead(DIP5) == OFF)
              {
                for(int i = 3; i < 6; i++)
                {
                  if(color_bright[i] > 0)
                    tmp = color_bright[i];
                }
                if(adc < 415)
                {
                  color_bright[3] = tmp;
                  color_bright[4] = 0;
                  color_bright[5] = 0; 
                }
                else if(adc < 830)
                {
                  color_bright[3] = 0;
                  color_bright[4] = tmp;
                  color_bright[5] = 0; 
                }
                else
                {
                  color_bright[3] = 0;
                  color_bright[4] = 0;
                  color_bright[5] = tmp; 
                }
              }
              // pixel 3: array values 6-8
              else if(digitalRead(DIP6) == OFF)
              {
                for(int i = 6; i < 9; i++)
                {
                  if(color_bright[i] > 0)
                    tmp = color_bright[i];
                }
                if(adc < 415)
                {
                  color_bright[6] = tmp;
                  color_bright[7] = 0;
                  color_bright[8] = 0; 
                }
                else if(adc < 830)
                {
                  color_bright[6] = 0;
                  color_bright[7] = tmp;
                  color_bright[8] = 0; 
                }
                else
                {
                  color_bright[6] = 0;
                  color_bright[7] = 0;
                  color_bright[8] = tmp; 
                }
              }
              // pixel 4: array values 9-11
              else if(digitalRead(DIP7) == OFF)
              {
                for(int i = 9; i < 12; i++)
                {
                  if(color_bright[i] > 0)
                    tmp = color_bright[i];
                }
                if(adc < 415)
                {
                  color_bright[9] = tmp;
                  color_bright[10] = 0;
                  color_bright[11] = 0; 
                }
                else if(adc < 830)
                {
                  color_bright[9] = 0;
                  color_bright[10] = tmp;
                  color_bright[11] = 0; 
                }
                else
                {
                  color_bright[9] = 0;
                  color_bright[10] = 0;
                  color_bright[11] = tmp; 
                }
              }
            }
            for(int i = 0; i < 12; i++)
            {
              xQueueSend(LEDQueue2, &color_bright[i], portMAX_DELAY);
            }
          }
          //xSemaphoreGive(LEDSemaphore);
          xQueueSend(LEDQueue, &command, portMAX_DELAY);
          taskYIELD();
          xQueueReceive(LEDQueue, &command, portMAX_DELAY);
          //xSemaphoreTake(LEDSemaphore, portMAX_DELAY);
        }
        pixelClear();
      }
    }
    // state 4 - move one rotation clockwise and one rotation counterclockwise and repeat
    else if(digitalRead(DIP2) == OFF && digitalRead(DIP1) == OFF)
    {
      // this one varies
      segManager(4, 5); // pass values to seven seg queues
      checkQueueIsFull(test); // quick check with X seconds delay if queue isnt full
      while(digitalRead(DIP2) == OFF && digitalRead(DIP1) == OFF && digitalRead(DIP3) == ON)
      {
        i = 1; // incrementer
        segManager(4, 12); // print C for clockwise
        stepCount = 1; // almost always 1 unless -1
        Serial.print("Stepper motor moving clockwise ");
        Serial.print(NUM_STEPS_PER_REV); // 2048 for full rotation
        Serial.println(" steps");
        // move stepper motor 1 step at a time, while also constantly checking dip switches states
        while(i <= NUM_STEPS_PER_REV && digitalRead(DIP2) == OFF && digitalRead(DIP1) == OFF && digitalRead(DIP3) == ON)
        {
          // send 1 to stepper queue
          xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
          // context switch (stepper motor task)
          taskYIELD();
          // keep display alive
          xSemaphoreGive(DisplaySemaphore);
          vTaskDelay(ONE_TICK);
          xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
          i++; // increment to ensure full rotation
        }
        i = 1; // set i back to one for counterclockwise rotation
        // print state 4 with a backwards C for counterclockwise
        segManager(4, 16);
        stepCount = -1; // going backwards
        Serial.print("Stepper motor moving counterclockwise ");
        Serial.print(NUM_STEPS_PER_REV);
        Serial.println(" steps");
        // move stepper motor 1 step at a time, while also constantly checking dip switches states
        while(i <= NUM_STEPS_PER_REV && digitalRead(DIP2) == OFF && digitalRead(DIP1) == OFF && digitalRead(DIP3) == ON)
        {
          // same thing as above
          xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();
          xSemaphoreGive(DisplaySemaphore);
          vTaskDelay(ONE_TICK);
          xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
          i++;
        }
      }
    }
    // state 3 - move stepper motor counterclockwise constantly (unless higher priority dip switch or turned off)
    else if(digitalRead(DIP2) == OFF)
    {
      // print state 3 with backwards C
      segManager(3, 16);
      checkQueueIsFull(test); // check
      Serial.println("Stepper motor moving counterclockwise until DIP switch changes");
      // getting a little repetitive here
      while(digitalRead(DIP2) == OFF && digitalRead(DIP3) == ON && digitalRead(DIP1) == ON)
      {
        stepCount = -1; // move backwards
        xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
        taskYIELD();
        xSemaphoreGive(DisplaySemaphore);
        vTaskDelay(ONE_TICK);
        xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
      }
    }
    // state 2 - move stepper motor clockwise constantly
    // these all look the same as above
    else if(digitalRead(DIP1) == OFF)
    {
      segManager(2, 12);
      checkQueueIsFull(test);
      Serial.println("Stepper motor moving clockwise until DIP switch changes");
      while(digitalRead(DIP1) == OFF && digitalRead(DIP2) == ON)
      {
        stepCount = 1;
        xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
        taskYIELD();
        xSemaphoreGive(DisplaySemaphore);
        vTaskDelay(ONE_TICK);
        xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
      }
    }
    // state 1 humidity - move stepper based on humidity
    else if(digitalRead(DIP0) == OFF)
    {
      segManager(1, 17); // print H
      checkQueueIsFull(test);
      Serial.println("Stepper motor moving based on humidity");
      while(digitalRead(DIP0) == OFF && digitalRead(DIP1) == ON && digitalRead(DIP2) == ON && digitalRead(DIP3) == ON)
      {
        humid = temp_hum_sensor.readHumidity(); // gather data from humidity sensor and store
        Serial.print("Humidity: ");
        Serial.println(humid);
        i = 1;
        // in order to pass 1 to stepper queue, gotta do it like this
        while(i <= humid)
        {
          stepCount = 1;
          xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();
          // DISPLAY!!!
          xSemaphoreGive(DisplaySemaphore);
          vTaskDelay(ONE_TICK);
          xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
          i++;
        }
        // this messes up display, oh well. Delay is to see how far the stepper moves, rather than constantly move
        xSemaphoreGive(DisplaySemaphore);
        vTaskDelay(HALF_SEC/2);
        xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
      }
    }
    // state 0 temp - move stepper based on temp, same thing as humidity
    else if(digitalRead(DIP0) == ON)
    {
      segManager(1, 18);
      checkQueueIsFull(test);
      Serial.println("Stepper motor moving based on temperature");
      while(digitalRead(DIP0) == ON && digitalRead(DIP1) == ON && digitalRead(DIP2) == ON && digitalRead(DIP3) == ON)
      {
        temp = temp_hum_sensor.readTemperature();
        Serial.print("Temperature: ");
        Serial.println(temp);
        i = 1;
        while(i <= temp)
        {
          stepCount = 1;
          xQueueSend(StepperQueue, &stepCount, portMAX_DELAY);
          taskYIELD();
          xSemaphoreGive(DisplaySemaphore);
          vTaskDelay(ONE_TICK);
          xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
          i++;
        }
        xSemaphoreGive(DisplaySemaphore);
        vTaskDelay(HALF_SEC/2);
        xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
      }
    }
    //display OF
    strip.clear();
    test = 1;
  }
}

// blink task, originally implemented in main, but needs to be separate task
void TaskBlink(void *pvParameters)
{
  int blink = 0;
  for(;;)
  {
    xQueueReceive(BlinkQueue, &blink, portMAX_DELAY);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(HALF_SEC/2);
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  }
}

// this works with the left part of the seven segment display
void TaskSevenSeg(void *pvParameters)
{
  (void)pvParameters;
  int num = 0, num2 = 0;
  for (;;)
  {
    xSemaphoreTake(DisplaySemaphore, portMAX_DELAY);
    xQueueReceive(LeftQueue, &num, 0); // receieve from left queue value from main
    printLR(SevenSegCC2, SevenSegCC1, num); // function that task delays and prints to screen
    xQueueReceive(RightQueue, &num2, 0); // receieve from right queue value from main
    printLR(SevenSegCC1, SevenSegCC2, num2); // function that task delays and prints to screen
    xSemaphoreGive(DisplaySemaphore);
  }
}

// stepper motor task
void TaskStepperMotor(void *pvParameters)
{
  (void)pvParameters;
  int num_steps = 0; // need an address value to store from queue
  for(;;)
  {
    // receieve step total from stepper queue (generally 1 so I can stop the DIP changes on a dime)
    xQueueReceive(StepperQueue, &num_steps, portMAX_DELAY);
    step_motor.step(num_steps); // moves the motor based on number passed from stepper queue
    // context switch (go back to main)
    taskYIELD();
  }
}

void TaskRGB(void *pvParameters)
{
  (void)pvParameters;
  int command;
  int color_bright[13];
  //uint16_t i, j;
  for(;;)
  {
    //xSemaphoreTake(LEDSemaphore, portMAX_DELAY);
    xQueueReceive(LEDQueue, &command, portMAX_DELAY);
    if(command == 1)
    {
      rainbowCycle(1);
      command = 0;
    }
    else if(command == 0)
    {
      for(int i = 0; i < 12; i++)
      {
        if(xQueueIsQueueEmptyFromISR(LEDQueue2) == pdTRUE)
          i = 12;
        else 
        {
          xQueueReceive(LEDQueue2, &color_bright[i], portMAX_DELAY);
        }
      }

      strip.setPixelColor(0, strip.Color(color_bright[0], color_bright[1], color_bright[2]));
      strip.setPixelColor(1, strip.Color(color_bright[3], color_bright[4], color_bright[5]));
      strip.setPixelColor(2, strip.Color(color_bright[6], color_bright[7], color_bright[8]));
      strip.setPixelColor(3, strip.Color(color_bright[9], color_bright[10], color_bright[11]));
      strip.show();   // Send the updated pixel colors to the hardware.
    }
    xQueueSend(LEDQueue, &command, portMAX_DELAY);
    //xSemaphoreGive(LEDSemaphore);
    taskYIELD();
  }
}