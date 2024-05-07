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

/**********************************************************/
/*                  Variable Definitions                  */
/**********************************************************/
long oldPosition = 0;
long newPosition = 0;
int motorPower;
double rotation;
float tolerance = 0.1; //FIND IDEAL VALUE
float maximumSteeringWheelTurns = 2; //FIND IDEAL VALUE
double minRotation = totalRotation / -maximumSteeringWheelTurns;
double maxRotation = totalRotation / maximumSteeringWheelTurns;
uint16_t outPosition;
struct canFrame frame;
struct canFrame out;
uint32_t target = 0x0CF00105;     //SET CAN VALUE
byte messageOut[8] {0, 0, 0, 0, 0, 0, 0, 0};
byte messageIn[8] {0, 0, 0, 0, 0, 0, 0, 0};

/********************************************************/
/*                  Object Definitions                  */
/********************************************************/
MCP2515 mcp2515(CS);
Encoder motorEncoder(encoderB, encoderA);


/******************************************/
/*                  Setup                 */
/******************************************/
void setup() {
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_250KBPS, MCP_12MHZ);
  mcp2515.setNormalMode();
  pinMode(SteeringPWMp, OUTPUT);
  pinMode(SteeringPWMn, OUTPUT);
  pinMode(TiltPWMp, OUTPUT);
  pinMode(TiltPWMn, OUTPUT);
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(currentSenseSteeringn, INPUT);
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(currentSenseSteeringp, INPUT);
  pinMode(sensorIn, INPUT);
  pinMode(MUXa, OUTPUT);
  pinMode(MUXb, OUTPUT);
  pinMode(MUXc, OUTPUT);
  pinMode(MUXread, INPUT);
}

void loop()
{
  //Read Encoder Position
  new_position = motor1.read();

  //Check if position Changed
  if (new_position != old_position)
  {
    
  }

  
}


void motorTest(bool motorA, bool motorB, double time = 10) 
{
  // Test Routine for Testing Motors
 
  if (motorA || motorB) 
  {
    for (int i = 0; i <= 255; i++) 
    {
      if (motorA) 
      {
        analogWrite(steeringPWMp, i);
        analogWrite(steeringPWMn, 0);
      }
      if (motorB) 
      {
        analogWrite(tiltPWMp, i);
        analogWrite(tiltPWMn, 0);
      }
      delay(time);
    }
    for (int i = 255; i >= 0; i--) {
      if (motorA) 
      {
        analogWrite(steeringPWMp, i);
        analogWrite(steeringPWMn, 0);
      }
      if (motorB) 
      {
        analogWrite(tiltPWMp, i);
        analogWrite(tiltPWMn, 0);
      }
      delay(time);
    }
  }
}
