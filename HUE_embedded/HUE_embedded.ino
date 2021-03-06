//** This program handle interaction between the HUE system and a server providing serial data.
// Currently: Given a 20 byte serial string ended with a \n and consisting of letters and integers,
//  this code will convert the integer values into positional instruction for the Nema 17 stepper
//  motors contained in the HUE system.
//**Created By: Matthew Ebert and Jamieson Fregeau
//**Date: 2021-11-02

#include <AccelStepper.h> // library used for making the AccellStepper objects.
#include <math.h>         // needed for the voltage2angle conversion math.

//These pins are connected from the arduino to the motor drivers.
#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 3
#define MOTOR_X_DIR_PIN 2

#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 5
#define MOTOR_Y_DIR_PIN 4

#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 7
#define MOTOR_Z_DIR_PIN 6

//precision of mathmatics used in the voltage2angle function.
#define p 1000

//These constants determine the pins to which the positional sensors must be connected.
#define prA A5
#define prB A6
#define prC A7

//Constants which define the range of motion of HUE and BERT.
//These define the maximum analog voltage reading of the arduino.
//THESE ARE FOR BERT
const int S1_max = 390;
const int S1_min = 0;
const int S2_max = 450;
const int S2_min = 0;
const int S3_max = 1000;
const int S3_min = -1000;

// These define the maximum relative angles which the HUE frame can take (in radians)
//  multiplied by the precision factor and stored as integers.
const double Q1_max = 2.41 ;
const double Q1_min = 0.33 ;
const double Q2_max = 2.79;
const double Q2_min = 0.52 ;
const double Q3_max = PI ;
const double Q3_min = -PI ;
const double offset1 = 0.33272;
const double offset2 = 1.84818;
const double offset3 = 0.88631;


// Constants for voltage2angle.
// These value define HUE and BERTS mechanical frame dimensions
const  int OG = 0.0800 * p; //m*p
const  int L1 = 0.255 * p; //m
const  int CL1 = 0.14142136 * p; //m
const  int L1m = 0.1100 * p; //m
const  int L2 = 0.2600 * p; //m
const  int CL2 = 0.28044607 * p; //m
const int L2e = 0.0800 * p; //m
const  int OL = 0.109337002 * p; //m
const  int OX = 0.1055 * p; //m

// Varialbes used to set the maximum speed and acceleration rates of HUE's stepper motors.
// Initial value set here
int alpha_speed = 150;
int beta_speed = 150;
int charlie_speed = 300;
int alpha_acc = 200;
int beta_acc = 200;
int charlie_acc = 500;

// Variable used to store the step targets of the HUE's stepper motors (alpha->1, beta->2, charlie->3)
int S1;
int S2;
int S3;

//Stepper object definitions
AccelStepper alpha(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper beta(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper charlie(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// The string used to collect recieved serial input.
String s_input = "";

//check values
bool harderror = false;

//functions
//------------------------------------------------------------------------------------------------


bool checksteppers (int rS1, int rS2, int rS3);
bool checkangles(int Q1, int Q2, int Q3);
void displaysensordata();
int homeHUE ();
void retreat2();
int maxdiff();
int printmotorposition();

int printmotortarget();
//-----------------------------------------------------





//-------------------------------------------------------------------------------------------------------------------------
void setup()
{
  //Runs once upon boot of arduino

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
  //-----------------------Serial set up
  Serial.begin(9600);
  Serial.setTimeout(100);
  s_input.reserve(20); //Reserve data up to 10 character or integers


  //define pin modes

  //These are used by stepper objects
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  //These are used by sensors
  pinMode(prA, INPUT);
  pinMode(prB, INPUT);
  pinMode(prC, INPUT);

  //These are extra ihputs
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);

  // This enables the stepper motors. Set pin 8 high to disable
  digitalWrite(8, LOW);

  //home motors
  //Set motors to home position: Make sure HUE starts from this defined position
  Serial.println("HUE--initiated ");
  homeHUE();

  Serial.println("HUE--standing by; ");

  //-----------------------------
} //setup




//***********************************MAIN PROGRAM

void loop()
{
  if (harderror == false) {
    //Serial.println("Standby");
    bool howcopy = 0;// a check variable used for return values of functions

    //displaysensordata();


    //read Serial Data if available in a string
    if (Serial.available())
    {

      //Serial.println("reading serial...");
      //s_input = Serial.readStringUntil('\n');

      //parse the input string if it has content
      //call functions and tasks here based on the first character in string
      char type = Serial.read();
      if (type == 'p')
      {
        S1 = Serial.readStringUntil('a').toInt();
        S2 = Serial.readStringUntil('b').toInt();
        S3 = Serial.readStringUntil('c').toInt();
        Serial.readStringUntil('\n');
        //      Serial.print("loc");
        //      Serial.print(alpha.currentPosition());
        //      Serial.print("a");
        //      Serial.print(beta.currentPosition());
        //      Serial.print("b");
        //      Serial.print(charlie.currentPosition());
        //      Serial.println('c');
        //
        //      Serial.print("Just Read :");
        //      Serial.print("   ");
        //      Serial.print(S1);
        //      Serial.print("   ");
        //      Serial.print(S2);
        //      Serial.print("   ");
        //      Serial.println(S3);
        howcopy = true;
      } // if (p)

      if (type == 'k') {
        printmotorposition();
      }
      Serial.flush();
    }   //if




    if (howcopy && checksteppers(S1, S2, S3))
    {
      //if the positional data was valid and converted to step value successfully set targets for steppers.

      Serial.println("HUE--setting destination...");
      // (1000);
      alpha.moveTo(S1);
      beta.moveTo(S2);
      charlie.moveTo(S3);
      //
      //        Serial.print("Current Position :");
      //        Serial.print("   ");
      //        Serial.print(alpha.currentPosition());
      //        Serial.print("   ");
      //        Serial.print(beta.currentPosition());
      //        Serial.print("   ");
      //        Serial.println(charlie.currentPosition());
      //        Serial.print("about to run to :");
      //        Serial.print("   ");
      //        Serial.print(S1);
      //        Serial.print("   ");
      //        Serial.print(S2);
      //        Serial.print("   ");
      //        Serial.println(S3);

      //run motors and check for more input
      int count = 0;
      Serial.println("HUE--running actuators");
      while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) )
      {
        //This will step motors 1 step each loop (if required by the moveTo property) until all motors have reached
        // there destination.
        //If more Serial data come in, stop moving and read the data.
        //runs motor a maximum of 1 step if required

        // Serial.println("HUE --running...");
        int motordiff = alpha.currentPosition() - beta.currentPosition();
        if (motordiff > maxdiff()) {
          Serial.println(maxdiff());
          //delay(100);
          beta.run();
          charlie.run();
          //Serial.println("Difference between S1 and S2 is to large");
          count = count + 1;
          if (beta.isRunning() == false) {
            Serial.println("HUE--failed to reach target to to motor position difference");
            retreat2();
            break;
          }
        }
        else
        {
          alpha.run();
          beta.run();
          charlie.run();
        }

      }//while

      Serial.println("HUE--standing by");

    }//if within limits

  } else {
    Serial.println("HUE--harderror, please reset me");
  }
} //loop


bool checksteppers (int rS1, int rS2, int rS3)
{
  // This function checks the sensor on HUE connected to the arduino nano on defined pins.

  // If the value of the parameters are outside defined limits,
  //   a warning will be sent through the serial port and a false returned
  // If the sensor value are within limits a value of true will be returned.

  //Serial.print("checking analog sensors...");


  if (  (rS1 <= S1_max) && (rS2 <= S2_max) && (rS3 <= S3_max) && (rS1 >= S1_min) && (rS2 >= S2_min) && (rS3 >= S3_min))
  {
    // Serial.println("Sensors indicate HUE is ready to move...");
    return true;
  }
  else
  {
    Serial.println("WARNING: HUE unable to comply....");
    return false;
  }
}//checksteppers

//----------------------------


int homeHUE ()
{
  // displaysensordata();
  // voltage2angle(analogRead(prA),analogRead(prB), 512);
  alpha.setCurrentPosition(0);
  beta.setCurrentPosition(0);
  charlie.setCurrentPosition(0);
  printmotorposition();
}

int printmotorposition() {

  Serial.print("HUE--at position:  ");
  Serial.print("p");
  Serial.print(alpha.currentPosition());
  Serial.print("a");
  Serial.print(beta.currentPosition());
  Serial.print("b");
  Serial.print(charlie.currentPosition());
  Serial.println("c");
}

int printmotortarget() {

  Serial.print("HUE--at position:  ");
  Serial.print("p");
  Serial.print(alpha.targetPosition());
  Serial.print("a");
  Serial.print(beta.targetPosition());
  Serial.print("b");
  Serial.print(charlie.targetPosition());
  Serial.println("c");
}


void retreat2()
{

  Serial.println("HUE--trying to retreat");
  //int ret = 50;
  alpha.moveTo(alpha.currentPosition());
  beta.moveTo(beta.currentPosition());
  //  int r1 = alpha.currentPosition() - ret;
  //  int r2 = beta.currentPosition() + ret;
  //  while (!checksteppers(r1, r2, 0))
  //  {
  //    ret = ret - 1;
  //    r1 = alpha.targetPosition() - ret;
  //    r2 = beta.targetPosition() + ret;
  //    if (ret < 1)
  //    {
  //      harderror == true;
  //      Serial.println("HUE-- I can't retreat...");
  //      break;
  //    }
  //
  //  }
  //  alpha.moveTo(r1);
  //  beta.moveTo(r2);
  //
  //  beta.runToPosition();
  //  alpha.runToPosition();


  printmotorposition();

}

int maxdiff()
{
  if (beta.currentPosition() < 300)
  {
    return 200 - 90 / 140 * alpha.currentPosition();
  } else
  {
    return 100;
  }

}
//------------------------------------------
