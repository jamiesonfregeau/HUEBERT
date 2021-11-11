#include <AccelStepper.h>
#include <math.h>

#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 2
#define MOTOR_X_DIR_PIN 3

#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 4
#define MOTOR_Y_DIR_PIN 5

#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 6
#define MOTOR_Z_DIR_PIN 7

//functions

//constants
int a_max = 580;
int c_max = 580;
int b_max = 580;
//Variables
int alpha_speed = 150;
int beta_speed = 150;
int charlie_speed = 150;
int alpha_acc = 600;
int beta_acc = 600;
int charlie_acc = 500;

int a = 0;
int b = 0;
int c = 0;

unsigned long A_avg = 0;
unsigned long B_avg = 0;
unsigned long C_avg = 0;
//serial variables
String s_input = "";
bool input_complete = false;
int check = 1;
//Steppers
AccelStepper alpha(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper beta(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper charlie(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

void setup()
{
  //------------------------Set up steppers motors
  alpha.setEnablePin(MOTOR_X_ENABLE_PIN);
  alpha.setPinsInverted(false, false, false);
  beta.setEnablePin(MOTOR_Y_ENABLE_PIN);
  beta.setPinsInverted(true, false, false);
  charlie.setEnablePin(MOTOR_Z_ENABLE_PIN);
  charlie.setPinsInverted(false, false, false);

  alpha.setAcceleration(alpha_acc);
  beta.setAcceleration(beta_acc);
  charlie.setAcceleration(charlie_acc);

  alpha.enableOutputs();
  beta.enableOutputs();
  charlie.enableOutputs();

  alpha.setMaxSpeed(alpha_speed);
  beta.setMaxSpeed(beta_speed);
  charlie.setMaxSpeed(charlie_speed);

  //---------------------------
  //-----------------------board set up
  Serial.begin(9600);
  Serial.setTimeout(100);
  s_input.reserve(9);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(8, LOW);

  //-----------------------------
} //setup

void loop()
{
  //read Serial Data if available in a string
  if (Serial.available())
  {
    s_input = Serial.readStringUntil('\n');

    //parse the input string if it has content
    if (s_input.length() == 9)
    {
      a = (s_input.substring(0, 3)).toInt()/2;
      b = (s_input.substring(3, 6)).toInt()/2;
      c = (s_input.substring(6, 9)).toInt()/2;
    } // if (!isempty(s_input))
  }   //if

  if (  a < a_max && b < b_max && c < c_max)
  {
    alpha.moveTo(a);
    beta.moveTo(b);
    charlie.moveTo(c);

    //        Serial.print("Current Position :");
    //        Serial.print(alpha.currentPosition());
    //        Serial.print(beta.currentPosition());
    //        Serial.println(charlie.currentPosition());
    //        Serial.print("about to run to :");
    //        Serial.print(a);
    //        Serial.print(b);
    //        Serial.println(c);

  }
  //run motors and check for more input
  int n = 0;
  while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) && !Serial.available())
  {
    //    Serial.println("running");
    alpha.run();
    beta.run();
    charlie.run();
  }//while

} //loop
