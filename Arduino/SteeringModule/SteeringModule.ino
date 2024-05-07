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
#define MUXread A7;                            //LIghting Stick MUX Read

//Steering Motor Encoder
#define EncoderA 3                          //Steering Motor Encoder A
#define EncoderB 4                          //Steering Motor Encoder B

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
#define SensorIn A0                         //Steering Sensor Input (Rack and Pinion Sensor Maybe)

/**********************************************************/
/*                  Constant Definitions                  */
/**********************************************************/
#define ENCODER_OPTIMIZE_INTERRUPTS
#define motorTicksPerRotation 435.3 // Encoder ticks per rotation
#define TOTAL_ROTATION 2            // Rotations from max left to max right
#define TOTAL_ROTATION_TICKS (435.3 * TOTAL_ROTATION) // Max rotation ticks
#define MIN_ROTATION (TOTAL_ROTATION / -2)
#define MAX_ROTATION (TOTAL_ROTATION / 2)
#define TOLERANCE 45

/**********************************************************/
/*                  Variable Definitions                  */
/**********************************************************/
//CAN Variables
struct canFrame frame;
struct canFrame out;
uint32_t target = 0x0CF00105;     //SET CAN VALUE
byte messageOut[8] {0, 0, 0, 0, 0, 0, 0, 0};
byte messageIn[8] {0, 0, 0, 0, 0, 0, 0, 0};

//Steering Variables
long oldPosition = 0;
long newPosition = 0;
long boundedPositiont = 0;
int motorPower;
double rotation;
uint16_t steeringOut;

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
  Serial.begin(115200);

//CAN Init
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_12MHZ);
  mcp2515.setNormalMode();

//Steering Motor Pin Init
  pinMode(SteeringPWMp, OUTPUT);
  pinMode(SteeringPWMn, OUTPUT);

//Tilt Motor Pin Init
  pinMode(TiltPWMp, OUTPUT);
  pinMode(TiltPWMn, OUTPUT);

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
  newPosition = motor1.read();

  //Check if position Changed
  if (newPosition != oldPosition)
  {
    //Set old position
    oldPosition = newPosition;
    boundedPosition = wheel_bound(newPosition);
  
  }

  
}

//This ensures the steering wheels rotation values stays within the working values of this code
long wheel_bound(long wheelPosition) 
{
  long temp_value;
  if (wheelPosition > MAX_ROTATION + TOLERANCE) 
  {
    return MAX_ROTATION + TOLERANCE;
  } 
  else if (wheelPosition < MIN_ROTATION - TOLERANCE) 
  {
    return MIN_ROTATION - TOLERANCE;
  } 
  else 
  {
    return wheelPosition;
  }
}
