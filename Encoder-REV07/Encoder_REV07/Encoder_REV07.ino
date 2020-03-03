/*
  Program: Encoder REV004
  Purpose: To control the braking mechanism of UBC UAS winch. Convert rotation of encoder into speed of Rover deployment.
           Then use this speed to control distance of linear actuator. Control of braking mechanism uses PID.
  Authors: Marko Jurisic, Karmen Wang, Yekta Ataozden
  Date: 25th January 2020

  //To setup arduino on vs code, use CTRL+SHIFT+P
*/

//Including libraries:
#include <Encoder.h>
#include <Servo.h>

//PIN definitions:
#define inputCLK 2  //PIN 2 has interrupt
#define inputDT 3  //PIN 3 has interrupt
#define Actuator 9 //Pin 9 for the linear actuator
#define RCSWITCH 7 //Pin 7 for the RC enabling, channel 5 for RC


//Declaring Constants: 
const int gearEncoderRatio = 1000;  //Pulley to encoder ratio
const int arrSize = 100;           //Array size for calculating the average speed
const int MINRANGECONSTANT = -50; //Constant for minimum speedoutput from encoder

//PID constants:
const float speedSetPoint = 1.0;  //Desired speed of deployment in terms of encoder rotations per time.

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
float previousError = 0.0; 
float speedOutput = 0.0;
float fallSpeed = 0.0;
double KP = 5;
double KI = 0;
double KD = 0;
float errorSum = 0.0;
double errorDifferential = 0.0;

//mapping constants
int maxRange = speedSetPoint * KP;
int minRange = MINRANGECONSTANT * KP;
int maxExtraction = 140;
int minExtraction = 130; //60

//RC input
int RCsignal;
 
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
  int timeDifference = currentTime - previousTime; ///////////////////////////////Small delay add
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

  //Setting the Rcinput
  pinMode(RCSWITCH, INPUT);
  RCsignal = pulseIn(RCSWITCH, HIGH, 25000);
  
}

void loop() {
//Read the RCSWITCH only if it's off
if (RCsignal < 1800){
  
  RCsignal = pulseIn(RCSWITCH, HIGH, 25000);
  
}
 
//If RC switch is on, then move into deployment with PID speed output
if (RCsignal > 1900){
  float fallSpeed = GetFallSpeed();

  //Calculating the error error Differential
  //previousError = error;

  //Getting error between set point speed and the actual speed
  error = speedSetPoint - fallSpeed;
  errorDifferential = error - previousError;

  //Calculating Error Sum
  //errorSum += error;

  // Calculating the output of the PID
  speedOutput = (KP * error) + (KI * errorSum) + (KD * errorDifferential);

  //mapping the speedOutput from PID to the servo position, which is fully retracted at 180 and fully extracted at 40
  int servoPosition =  map(speedOutput,maxRange,minRange,minExtraction,maxExtraction); //min of speed happens when rotary encoder is going fast so should appraoch 40 as it goes faster

  //Serial.println(speedOutput);
  linearServo.write(servoPosition);
  Serial.println(servoPosition);
}

//If RC switch is not on, then close the brake
else {
  Serial.println(RCsignal);
  linearServo.write(maxExtraction);
}

}
