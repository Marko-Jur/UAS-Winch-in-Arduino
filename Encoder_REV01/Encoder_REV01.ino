/*
  Program: Encoder REV001
  Purpose: To control the braking mechanism of UBC UAS winch. Convert rotation of encoder into speed of Rover deployment.
           Then use this speed to control distance of linear actuator. Control of braking mechanism uses PID.
  Authors: Marko Jurisic, Karmen Wang, Yekta Ataozden
  Date: 25th January 2020
*/



//Including libraries:
#include <Encoder.h>
#include <Servo.h>
#include <PID_v1.h>

//PIN definitions:
#define inputCLK 2  //PIN 2 has interrupt
#define inputDT 3  //PIN 3 has interrupt
#define Actuator 9 //Pin 9 for the linear actuator


//Declaring Constants:
const int gearEncoderRatio = 1000;  //Pulley to encoder ratio
const int arrSize = 100;           //Array size for calculating the average speed

//PID constants:
const float speedSetPoint = 50.0;  //Desired speed of deployment in terms of encoder rotations per time.
const float KP = 0.0;             //Proportion Constant
const float KI = 0.0;            //Integral Constant
const float KD = 0.0;           //Derivative Constant

//Objects:
Encoder UASEncoder(inputCLK,inputDT);  //Creating an encoder object
Servo linearServo;                    //Creating a servo object for the linear actuator


//Declaring Variables:
//State Variables
uint32_t currentState;    //uint32_t is an unsigned 32 bit integer. Use 32 bit as it can be read faster than a normal 64bit unsigned int
uint32_t previousState;  //uint32_t is an unsigned 32 bit integer. Use 32 bit as it can be read faster than a normal 64bit unsigned int
//Time variables
int currentTime;
int previousTime;
//Distance variable
int totalDistance = 0;
// Speed array
float avgSpeed[arrSize];
int arrPos = 0;
//PID variables:
float error = 0.0; 
float speedControlVariable = 0.0;

/*
  Function: GetFallSpeed
  Purpose: Converts the rotation of the encoder into the speed of deployment of the rover.
  Inputs: N/A
  Outputs: averageSpeed - the speed of deployment of the rover.
*/

float GetFallSpeed()
{
  //Get the amount of clicks
  currentState = uint32_t(abs(UASEncoder.read())); // Reading the encoders current value, converting to 32 bit absolute value.
  int stateDifference = currentState - previousState;
  previousState = currentState;                   //Reset the state to the current state

  // Get time
  currentTime = millis();       // Get time using this function
  int timeDifference = currentTime - previousTime;
  previousTime = currentTime;   //Reset previous time

  //Get Distance
  float distance = gearEncoderRatio * stateDifference; // Get distance travelled for one discrete rotation of the encoder
  totalDistance += distance;                          //Sum these discrete distances to have the total distance travelled

    
  float fallSpeed = abs(distance/timeDifference); //Convert to absolute value
  
  //Store speed values into an array, then calculate the average of these speed values. Prevents skipping
  avgSpeed[arrPos] = fallSpeed;
  arrPos++;
  if(arrPos == arrSize)
  {
    arrPos = 0;
  }
  float averageSpeed = 0;
  for(int i = 0; i < arrSize; i++)
  {
    
    averageSpeed += avgSpeed[i];
  }
  averageSpeed = averageSpeed / arrSize;
 
  return averageSpeed;
}


void setup() {

  // Set encoder pins as inputs  
  pinMode(inputCLK,INPUT);
  pinMode(inputDT,INPUT);
  
  //Setting up serial monitor
  Serial.begin(9600);

  //Connecting the linear actuator
  linearServo.attach(9);
  
  //Initiliazaing variables
  previousState = UASEncoder.read();
  previousTime = millis();

  for(int i = 0; i < arrSize; i++) 
  {
    avgSpeed[i] = 0.0; //Initialize array
  }

}

void loop() {

  float fallSpeed = GetFallSpeed();

  error = speedSetPoint - fallSpeed;

  speedControlVariable = 0.0;
 
  
}