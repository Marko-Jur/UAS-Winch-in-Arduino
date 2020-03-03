
//Including libraries
#include <Encoder.h>
#include <Servo.h>

//PIN definitions 
#define inputCLK 2  //PIN 2 has interrupt
#define inputDT 3  //PIN 3 has interrupt
#define Actuator 9 //Pin 9 for the linear actuator


//Declaring Constant
const int gearEncoderRatio = 1000; //Pulley to encoder ratio
const int arrSize = 100; //Array size for calculating the average speed
const float desiredSpeed = 50.0; //Desired speed of deployment in terms of encoder rotations per time.


//Objects

//Creating an encoder object
Encoder UASEncoder(inputCLK,inputDT);

//Creating a servo object for the linear actuator
Servo linearServo;



//Declaring Variables
uint32_t currentState;   //uint32_t is an unsigned 32 bit integer. Use 32 bit as it can be read faster than a normal 64bit unsigned int
uint32_t previousState; //uint32_t is an unsigned 32 bit integer. Use 32 bit as it can be read faster than a normal 64bit unsigned int
//Time variables
int currentTime;
int previousTime;
//Distance variable
int totalDistance = 0;
// Speed array
float avgSpeed[arrSize];
int arrPos = 0;






float GetFallSpeed()
{
  //Get the amount of clicks
  currentState = uint32_t(abs(UASEncoder.read())); // Reading the encoders current value, converting to 32 bit absolute value.
  int stateDifference = currentState - previousState;
  previousState = currentState; //Reset the state to the current state

  // Get time
  currentTime = millis(); // Get time using this function
  int timeDifference = currentTime - previousTime;
  previousTime = currentTime; //Reset previous time

  //Get Distance
  float distance = gearEncoderRatio * stateDifference; // Get distance travelled for one discrete rotation of the encoder
  totalDistance += distance;  //Sum these discrete distances to have the total distance travelled

    
  float fallSpeed = abs(distance/timeDifference);
  
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
  //Serial.println(averageSpeed);
  
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
    avgSpeed[i] = 0.0;
  }

}

void loop() {

  float fallSpeed = GetFallSpeed();
  //if (fallSpeed != 0)
  Serial.println(fallSpeed);


  

  if (fallSpeed > desiredSpeed){
    
    linearServo.write();
  
  }
  
  
}
