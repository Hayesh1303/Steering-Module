/**********************************************************/
//  SteeringModule  --  Board Rev 2
//  
//  Hardware: John Anderson     jma2388@iastate.edu
//
//  Software: Henry Hayes       hayesh@iastate.edu
//
/**********************************************************/
/*
12-Pin Deutsch Connector: 
  1: Tilt Motor Positive       12: Tilt Motor Negative
  2: Not Used                  11: Not Used
  3: Not Used                  10: Steering Sensor Input
  4: Encoder A                  9: +5 Volt Supply
  5: Encoder B                  8: Ground
  6: Steering Motor Positive    7: Steering Motor

Encoder Connector:
    Blue: +5 Volts OR Ground
   Green: +5 Volts OR Ground
  Yellow: Encoder A OR B
  Orange: Encoder A OR B

Don't worry about the steering sensor input. Not sure what that even does.
*/

/**********************************************/
/*                  Libraries                 */
/**********************************************/
#include <Encoder.h>
#include <Math.h>
#include <SPI.h>
#include <mcp2515.h>

/**********************************************************/
/*                  Function Declarations                 */          
/**********************************************************/
long wheel_bound(long wheelPosition);
long map_steering_motor (long boundedPosition, bool wheelDirection);
long map_steering_output(long boundedPosition);


/***************************************************/
/*                  Pin Definitions                */
/***************************************************/

//MCP2515
#define INT 2                               //MCP2515 Interrupt
#define CS 8                                //MCP2515 CS

//Lighting Stick MUX
#define MUXa A5                             //Lighting Stick MUX A
#define MUXb A6                             //Lighting Stick MUX B
#define MUXc 7                              //Lighting Stick MUX C
#define MUXread A7                            //LIghting Stick MUX Read

//Steering Motor Encoder
#define encoderA 3                          //Steering Motor Encoder A
#define encoderB 4                          //Steering Motor Encoder B

//Steering Motor
#define steeringPWMp 5                      //Steering Motor PWM Positive
#define steeringPWMn 6                      //Steering Motor PWM Negative
#define currentSenseSteeringp A1            //Steering Motor Current Sense Positive
#define currentSenseSteeringn A2            //Steering Motor Current Sense Negative

//Tilt Motor
#define tiltPWMp 9                          //Tilt Motor PWM Positive
#define tiltPWMn 10                         //Tilt Motor PWM Negative
#define currentSenseTiltp A3                //Tilt Motor Current Sense Positive
#define currentSenseTiltn A4                //Tilt Motor Current Sense Negative

//Other
#define sensorIn A0                         //Steering Sensor Input (Rack and Pinion Sensor Maybe)

/**********************************************************/
/*                  Constant Definitions                  */
/**********************************************************/
//Direction
#define LEFT true
#define RIGHT false

//Encoder
#define ENCODER_OPTIMIZE_INTERRUPTS

//Steering Rotation
#define TICK_PER_ROTATION 435.3 // Encoder ticks per rotation
#define TOTAL_ROTATION 2            // Rotations from max left to max right
#define TOTAL_ROTATION_TICKS (TICK_PER_ROTATION * TOTAL_ROTATION) // Max rotation ticks
#define MIN_ROTATION (TOTAL_ROTATION / -2)
#define MAX_ROTATION (TOTAL_ROTATION / 2)
#define TOLERANCE 45

//Steering Position Output
#define MAX_STEERING_OUTPUT 10000
#define MIN_STEERING_OUTPUT 0

//Steering Motor Power Output
#define MAX_MOTOR_OUTPUT 150

//CAN message IDs
#define steeringID 0x0CFF0105
/**********************************************************/
/*                  Variable Definitions                  */
/**********************************************************/
//CAN Variables
struct can_frame steeringMessage;
byte messageOut[8] {0, 0, 0, 0, 0, 0, 0, 0};

//Steering Variables
long oldPosition = 0;
long newPosition = 0;
long boundedPosition = 0;
int motorPower;
double rotation;
uint16_t steeringOut;

//Use in loops
int i, j, k;

/********************************************************/
/*                  Object Definitions                  */
/********************************************************/
//CAN Object
MCP2515 mcp2515(CS);

//Motor Encoder Object
Encoder motorEncoder(encoderB, encoderA);


/******************************************/
/*                  Setup                 */
/******************************************/
void setup() {
  Serial.begin(9600);

//CAN Init
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_16MHZ);     /*        <---  FIX THIS         */
  mcp2515.setNormalMode();

//CAN Message init
  steeringMessage.can_id = steeringID;
  steeringMessage.can_dlc = 8;
  
//Steering Motor Pin Init
  pinMode(steeringPWMp, OUTPUT);
  pinMode(steeringPWMn, OUTPUT);

//Tilt Motor Pin Init
  pinMode(tiltPWMp, OUTPUT);
  pinMode(tiltPWMn, OUTPUT);

//Bullshit
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(currentSenseSteeringn, INPUT);
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(sensorIn, INPUT);

//MUX Pin Init
  pinMode(MUXa, OUTPUT);
  pinMode(MUXb, OUTPUT);
  pinMode(MUXc, OUTPUT);
  pinMode(MUXread, INPUT);
}

void loop()
{
  //Read Encoder Position
  newPosition = motorEncoder.read();

  //Check if position Changed
  if (newPosition != oldPosition)
  {
    //Set old position
    oldPosition = newPosition;
    boundedPosition = wheel_bound(newPosition);
    
    steeringOut = map_steering_output(boundedPosition);
    
    //Serial.print("Out position: ");
    //Serial.println(steeringOut);

    //Maps Out Position to a CAN Message
    Serial.print("Can message: ");
    for(i = 0; i < 2; i++)
    {
      steeringMessage.data[i] = (steeringOut >> (8 * i) && 0xFF;
    }

    //Sends steering output message
    mcp2515.sendMessage (&steeringOut);


    //Causes resistance if passing Right Bound
    if(boundedPosition > MAX_ROTATION - TOLERANCE)
    {
      motorPower = map_steering_motor(boundedPosition, RIGHT);

      analogWrite(steeringPWMp, motorPower);
      analogWrite(steeringPWMn, 0);
    }

    //Causes resistance if passing left bound
    else if(boundedPosition < MIN_ROTATION + TOLERANCE)
    {
      motorPower = map_steering_motor(boundedPosition, LEFT);

      analogWrite(steeringPWMp, 0);
      analogWrite(steeringPWMn, motorPower);
    }

    //Resets the motor powers
    else
    {
      analogWrite(steeringPWMp, 0);
      analogWrite(steeringPWMn, 0);
    }
  }

  
}




/**********************************************/
/*                  Functions                 */
/**********************************************/
//This ensures the steering wheels rotation values stays within the working values of this code
long wheel_bound(long wheelPosition) 
{
  if (wheelPosition > MAX_ROTATION + TOLERANCE)     //Bounds max position
  {
    return MAX_ROTATION + TOLERANCE;
  }
  else if (wheelPosition < MIN_ROTATION - TOLERANCE)  //Bounds min position
  {
    return MIN_ROTATION - TOLERANCE;
  } 
  else  //Otherwise
  {
    return wheelPosition;
  }
}

//Maps steering wheel position to motor output pwm
long map_steering_motor (long boundedPosition, bool wheelDirection)
{
  long inMin,inMax;

  //Sets the min and max values
  if(wheelDirection)
  {
    inMin = MIN_ROTATION - TOLERANCE;
    inMax = MIN_ROTATION + TOLERANCE;
  }
  else
  {
    inMin = MAX_ROTATION - TOLERANCE;
    inMax = MAX_ROTATION + TOLERANCE;
  }

  // Maps steering power to exponential
  boundedPosition = boundedPosition - inMin;
  boundedPosition = 5 * abs(boundedPosition) + pow(boundedPosition, 4);

  //Sets new min and max values
  inMax = (inMax - inMin);
  inMax = 5 * abs(inMax) + pow(inMax, 4);
  inMin = 0;

  //Maps with adjusted values
  return map(boundedPosition, inMin, inMax, 0, MAX_MOTOR_OUTPUT); 
}

//Maps motor position for Can message
long map_steering_output(long boundedPosition)
{
  return map(boundedPosition, MIN_ROTATION - TOLERANCE, MAX_ROTATION + TOLERANCE, 0, MAX_STEERING_OUTPUT);
}
