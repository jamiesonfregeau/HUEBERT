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
const int rA_max = 1023;
const int rA_min = 0;
const int rB_max = 780;
const int rB_min = 0;
const int rC_max = 1023;
const int rC_min = 0;

// These define the maximum relative angles which the HUE frame can take (in radians)
//  multiplied by the precision factor and stored as integers.
const double Q1_max = 2.41 ;
const double Q1_min = 0.33 ;
const double Q2_max = PI ;
const double Q2_min = 0.83 ;
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
int alpha_speed = 100;
int beta_speed = 100;
int charlie_speed = 100;
int alpha_acc = 300;
int beta_acc = 300;
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

//functions
//------------------------------------------------------------------------------------------------


int voltage2angle(int a_in, int b_in, int c_in)
{
  // This function converts a analog voltage value (0 to 1024) to a step position.
  // This function relies heavily on the defined dimensions of HUE and BERT.

  // This fuction will confirm the validity of the sent values against the
  //  defined limits of HUE's mechanical frame.

  // This function generally take 2ms to run.
  // Avoid the use of delays or print function to speed up compeletion.

  //Currently, doubles and long types are used to complete the mathmatical conversion.
  //For security and speed reason these should be changed in future version to ints.

  //Serial.println("converting voltage value to step angle...");

  //Convert the analog reading into the cooresponding arm angle in radians.
  double Q1 = Q1_min + a_in * (Q1_max - Q1_min) / (rA_max - rA_min); //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE*********s
  double Q2 = PI + (Q2_min + b_in * (Q2_max - Q2_min) / (rB_max - rB_min)) - Q1 - offset3 - offset2; //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE***********
  double Q3 = -PI + c_in * 2 * PI / 1024; //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE********
  //check if the desired value is valid
  //   Serial.print(" Q1= ");
  //      Serial.print(double(Q1));
  //      Serial.print("      Q2= ");
  //      Serial.println(double(Q2));;;

  if (!checkangles(Q1, Q2, Q3))
  {
    //if angle outside limits, return false and report back to main
    return 0;
  }

  //double now = micros(); //Uncomment this to start timing function


  //Convert arm angle to motor shaft angle for stepper alpha
  //See math model for more information on equation here
  double q1_theta = PI   - Q1;
  long O_L1m = sqrt(pow(L1m, 2) + pow(OL, 2) - 2 * L1m * OL * cos(q1_theta));
  double q1_beta =  acos(( pow(OG, 2)  - pow(CL1, 2) + pow(O_L1m, 2)) / (2 * OG * O_L1m));
  double q1_alpha = acos( (pow(OL, 2)  - pow(L1m, 2) + pow(O_L1m, 2)) / (2 * OL * O_L1m ));
  double qS1 = (q1_beta + q1_alpha); //Angle of OG1 referenced to the line between center of planetary gear and pivot of L1

  //Convert arm angle to motor shaft angle for stepper beta
  //See math model for more information on equation here
  double q2_theta = PI - Q2;
  long R2 = sqrt(pow(L1, 2) + pow(L2e, 2) - 2 * long(L1) * long(L2e) * cos(q2_theta));
  double q_L1_R2 = acos((-pow(L2e, 2)  + pow(R2, 2) + pow(L1, 2)) / (2 * R2 * L1));
  double q2_gamma = PI - Q1 - q_L1_R2;
  long  R1 =  sqrt(pow(OL, 2) + pow(R2, 2) - 2 * long(OL) * long(R2) * cos(q2_gamma));
  double q2_beta =  acos((pow(OG, 2) - pow(CL2, 2) + pow(R1, 2)) / (2 * OG * R1));
  double q2_alpha =  acos((pow(OL, 2) - pow(R2, 2) + pow(R1, 2)) / (2 * OL * R1));
  double qS2 = q2_alpha + q2_beta;//Angle of OG2 referenced to the line between center of planetary gear and pivot of L1

  //Convert motor arm angle to steps based on a precision of 200 steps per rotation
  int tS1 = (3.442 - qS1) / (2 * PI) * 4 * 200; //CHANGE THIS IF alpha STEPPER CHANGES*******
  int tS2 = (4.114 - qS2) / (2 * PI) * 4 * 200; //CHANGE THIS IF beta STEPPER CHANGES********
  int tS3 = (Q3 * 13) * 200 / (2 * PI);// CHANGE THIS IF charlie STEPPER CHANGES*************[

  if (checksteppers(tS1, tS2, tS3))
  {
    S1 = tS1;
    S2 = tS2;
    S3 = tS3;

  }
  else
  {
    Serial.println("INVALID POSITION!...");
  }

  // double then = micros();//Uncomment to end timing function

  //  Serial.print("            time it took = ");
  //  Serial.println(then - now);
  //  Serial.println();

  //Uncomment below to send converted values through serial connection.
  Serial.print(" Q1= ");
  Serial.print(double(Q1));
  Serial.print("      Q2= ");
  Serial.println(double(Q2));
  Serial.print(" S1= ");
  Serial.print(S1);
  Serial.print("          S2 = ");
  Serial.println(S2);
  Serial.println();
  Serial.println();

  //Serial.println("conversion complete...");

  return 1;
}//voltage2angle


//-----------------------------------------------------


bool checksteppers (int rA, int rB, int rC)
{
  // This function checks the sensor on HUE connected to the arduino nano on defined pins.

  // If the value of the parameters are outside defined limits,
  //   a warning will be sent through the serial port and a false returned
  // If the sensor value are within limits a value of true will be returned.

  //Serial.print("checking analog sensors...");


  if (  (rA < rA_max) && (rB < rB_max) && (rC < rC_max) && (rA >= rA_min) && (rB >= rB_min) && (rC >= rC_min))
  {
    //Serial.println("Sensors indicate HUE is ready to move...");
    return true;
  }
  else
  {
    Serial.println("WARNING: HUE OUTSIDE MECHANICAL LIMITS....");
    return false;
  }
}//checksensors

//----------------------------


bool checkangles(double Q1, double Q2, double Q3)
{
  //This function checks if the input angle is within the defined limits of HUE's range

  //If the angle would be outside HUE's range a warning is sent through the serial port.
  //If the angle is within limits a true value will be returned
  if (  Q1 < Q1_max && Q2 < Q2_max && Q3 < Q3_max && Q1 > Q1_min && Q2 > Q2_min && Q3 > Q3_min)
  {
    return true;
  }
  else
  {
    Serial.println("WARNING: HUE CANNOT MOVE HERE....");
    return false;
  }
}

int homeHUE ()
{
  // displaysensordata();
  // voltage2angle(analogRead(prA),analogRead(prB), 512);
  alpha.setCurrentPosition(14);
  beta.setCurrentPosition(3);
  charlie.setCurrentPosition(512);
  //    Serial.print("Current Position :");
  //        Serial.print("   ");
  //        Serial.print(alpha.currentPosition());
  //        Serial.print("   ");
  //        Serial.print(beta.currentPosition());
  //        Serial.print("   ");
  //        Serial.println(charlie.currentPosition());
}

//------------------------------------------

void displaysensordata()
{
  //This function sends current sensor data through the serial port
  int  rA = analogRead(prA);
  int  rB = analogRead(prB);
  int  rC = analogRead(prC);
  Serial.print("      A= ");
  Serial.print(rA);
  Serial.print("      B= ");
  Serial.print(rB);
  Serial.print("      C= ");
  Serial.println(rC);
}


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
  homeHUE();
  Serial.write("ready to receive instructions");

  //-----------------------------
} //setup




//***********************************MAIN PROGRAM

void loop()
{
  bool howcopy = 0;// a check variable used for return values of functions

  //displaysensordata();


  //read Serial Data if available in a string
  if (Serial.available())
  {
    //Serial.println("reading serial...");
    s_input = Serial.readStringUntil('\n');

    //parse the input string if it has content
    if (s_input.length() == 10)
    {
      //call functions and tasks here based on the first character in string

      if (s_input[0] == 'p') {
        //p indicated positional data. Seperate into 3 digit analog values
        int a_s = (s_input.substring(1, 4)).toInt();
        int b_s = (s_input.substring(4, 7)).toInt();
        int c_s = (s_input.substring(7, 10)).toInt();
        //Convert the analog voltage into step value
        howcopy = voltage2angle(a_s, b_s, c_s);//This updates S1, S2 and S3 if howcopy==1;
      }//position
    } // if (s_input)
  }   //if


  if (howcopy) // ( checksensors() && howcopy)
  {
    //if the positional data was valid and converted to step value successfully set targets for steppers.

    //  Serial.println("Setting destination...");
    //delay(1000);
    alpha.moveTo(S1);
    beta.moveTo(S2);
    charlie.moveTo(S3);

    Serial.print("Current Position :");
    Serial.print("   ");
    Serial.print(alpha.currentPosition());
    Serial.print("   ");
    Serial.print(beta.currentPosition());
    Serial.print("   ");
    Serial.println(charlie.currentPosition());
    //        Serial.print("about to run to :");
    //        Serial.print("   ");
    //        Serial.print(S1);
    //        Serial.print("   ");
    //        Serial.print(S2);
    //        Serial.print("   ");
    //     Serial.println(S3);
    //     delay(1000);
  }//if within limits
  else
  {
    //Serial.println("INVALID DESTINATION");
  }
  //run motors and check for more input

  while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) && !Serial.available())
  {
    //This will step motors 1 step each loop (if required by the moveTo property) until all motors have reached
    // there destination.
    //If more Serial data come in, stop moving and read the data.

    // Serial.println("...stepping...");
    //runs motor a maximum of 1 step if required
    alpha.run();
    beta.run();
    charlie.run();
  }//while

} //loop
