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

#define p 1000  //precision of conversion mathmatics

#define prA A7
#define prB A6
#define prC A5

//constants
const int rA_max = 999;
const int rA_min = 0;
const int rB_max = 999;
const int rB_min = 0;
const int rC_max = 999;
const int rC_min = 0;


const int Q1_max = 2.41 * p;
const int Q1_min = 0.26 * p;
const int Q2_max = 2.63 * p;
const int Q2_min = 0.35 * p;
const int Q3_max = 3.1415 * p;
const int Q3_min = -3.1415 * p;

//constants for voltage2angle
const unsigned int OG = 0.0800 * p; //m*p
const unsigned int L1 = 0.255 * p; //m
const unsigned int CL1 = 0.14142136 * p; //m
const unsigned int L1m = 0.1100 * p; //m
const unsigned int L2 = 0.2600 * p; //m
const unsigned int CL2 = 0.28044607 * p; //m
const unsigned int L2e = 0.0800 * p; //m
const unsigned int OL = 0.109337002 * p; //m
const unsigned int OX = 0.1055 * p; //m

//Variables
int alpha_speed = 150;
int beta_speed = 150;
int charlie_speed = 150;
int alpha_acc = 600;
int beta_acc = 600;
int charlie_acc = 500;

int a_s = 0;
int b_s = 0;
int c_s = 0;
int rA = rA_max;
int rB = rB_max;
int rC = 0;

int S1 = 0;
int S2 = 0;
int S3 = 0;

unsigned int Q1 = Q1_min;
unsigned int Q2 = Q2_min;
unsigned int Q3 = 0;



//serial variables
String s_input = "";
bool input_complete = false;
int check = 1;
//Steppers
AccelStepper alpha(1, MOTOR_X_STEP_PIN, MOTOR_X_DIR_PIN);
AccelStepper beta(1, MOTOR_Y_STEP_PIN, MOTOR_Y_DIR_PIN);
AccelStepper charlie(1, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);



//functions
//-----------------------------------------------------
int voltage2angle(int qin1, int qin2, int qin3)
{

  //  Serial.println(OG);
  //  Serial.println(L1);
  //  Serial.println(L2);

  qin1 = (2410 - qin1 * (2410 - 260) / 1024);
  qin2 =  (350 + qin2 * (2630 - 350) / 1024);

  //convert to decimal precision int
  Q1 = qin1;
  Q2 = qin2;


  double now = micros();
  //stepper 1
  long O_L1m = sqrt(pow(L1m, 2) + pow(OL, 2) - 2 * L1m * OL * cos(double(PI * p  - Q1) / p));
  //    int q1_beta =  acos( (pow(OG, 2)  - pow(CL1, 2) + pow(O_L1m, 2)) / (2 * OG * O_L1m)) * p;
  //    int q1_alpha = acos( (pow(OL, 2)  - pow(L1m, 2) + pow(O_L1m, 2)) / (2 * OL * O_L1m )) * p;
  int qS1 = (acos((pow(OG, 2)  - pow(CL1, 2) + pow(O_L1m, 2)) / (2 * OG * O_L1m))    +     acos((pow(OL, 2)  - pow(L1m, 2) + pow(O_L1m, 2)) / (2 * OL * O_L1m ))); //Angle of OG1 referenced to the line between center of planetary gear and pivot of L1

  //stepper 2
  long R2 = sqrt(pow(L1, 2) + pow(L2e, 2) - 2 * L1 * L2e * cos(PI - double(Q2) / p));
  int q_L1_R2 = acos((-pow(L2e, 2)  + pow(R2, 2) + pow(L1, 2)) / (2 * R2 * L1)) * p;
  long  R1 =  sqrt(pow(OL, 2) + pow(R2, 2) - 2 * OL * R2 * cos(PI - double(Q1) / p - double(q_L1_R2) / p));
  //    int q2_beta =  acos((pow(OG, 2) - pow(CL2, 2) + pow(R1, 2)) / (2 * OG * R1)) * p;
  //    int q2_alpha =  acos((pow(OL, 2) - pow(R2, 2) + pow(R1, 2)) / (2 * OL * R1)) * p;
  int qS2 = (acos((pow(OG, 2) - pow(CL2, 2) + pow(R1, 2)) / (2 * OG * R1))    +    acos((pow(OL, 2) - pow(R2, 2) + pow(R1, 2)) / (2 * OL * R1))); //Angle of OG2 referenced to the line between center of planetary gear and pivot of L1


  S1 = (3.442 - qS1) / (2 * PI) * 4 * 200;
  S2 = (4.114 - qS2) / (2 * PI) * 4 * 200;
  S3 = (S3 * 13) * 200 / (2 * PI);
  double then = micros();

  Serial.println();
  Serial.print(" Q1= ");
  Serial.print(Q1);
  Serial.print("      Q2= ");
  Serial.print(Q2);
  Serial.print("      Q3= ");
  Serial.println(Q3);
  Serial.print(" qS1= ");
  Serial.print(qS1);
  Serial.print("          qS2 = ");
  Serial.println(qS2);
  Serial.println();
  Serial.println();


  Serial.println();
  Serial.print("            time it took = ");
  Serial.println(then - now);
  Serial.println();

  return 1;
}//voltage2angle


//-----------------------------------------------------
bool checksensors ()
{
  Serial.println("checking HUE sensors");
  Serial.println(rA);
  rA = analogRead(prA);
  rB = analogRead(prB);
  rC = analogRead(prC);
  //if (  rA >rA_max || rB > rB_max || rC > rC_max || rA < rA_min || rB < rB_min || rC < rC_max)
  if (  rA > rA_max || rB > rB_max || rC > rC_max)
  {
    Serial.print("DANGER: EXCEEDING MECHANICAL LIMITS!");
    return false;
  }
  return true;
}//checksensors

//-------------------------------------------------------------------------------------------------------------------------start
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
  s_input.reserve(20);

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  pinMode(prA, INPUT);
  pinMode(prB, INPUT);
  pinMode(prC, INPUT);
  pinMode(A4, INPUT);
  pinMode(A3, INPUT);
  digitalWrite(8, LOW);
  Serial.println("ready to receive instructions");
  Serial.println(Q3_max);

  //-----------------------------
} //setup

void loop()
{
  //read Serial Data if available in a string
  if (Serial.available())
  {
    s_input = Serial.readStringUntil('\n');

    //parse the input string if it has content
    if (s_input.length() == 10)
    {
      if (s_input[0] == 'p') {
        a_s = (s_input.substring(1, 4)).toInt();
        b_s = (s_input.substring(4, 7)).toInt();
        c_s = (s_input.substring(7, 10)).toInt();

        int howcopy = voltage2angle(a_s, b_s, c_s);
        if (howcopy && Q1 < Q1_max)
        {
          Serial.print("clear to move; target within angle limits= ");
          Serial.println( Q1 <= Q1_max && Q2 <= Q2_max && Q3 <= Q3_max && Q1 >= Q1_min && Q2 >= Q2_min && Q3 >= Q3_max);
          delay(100);
          alpha.moveTo(S1);
          beta.moveTo(S2);
          charlie.moveTo(S3);

          //        Serial.print("Current Position :");
          //        Serial.print(alpha.currentPosition());
          //        Serial.print(beta.currentPosition());
          //        Serial.println(charlie.currentPosition());
                  Serial.print("about to run to :");
                  Serial.print(S1);
                  Serial.print(S2);
                  Serial.println(S3);
        }//if within limits
      }//position
    } // if (s_input)
  }   //if

  //if (  Q1 < Q1_max && Q2 < Q2_max && Q3 < Q3_max && Q1 > Q1_min && Q2 >Q2_min && Q3 > Q3_min)

  //run motors and check for more input

  int n = 0;
  while ((alpha.isRunning() || beta.isRunning() || charlie.isRunning()) && !Serial.available() && checksensors())
  {
    //    Serial.println("running");
    alpha.run();
    beta.run();
    charlie.run();
  }//while

} //loop
