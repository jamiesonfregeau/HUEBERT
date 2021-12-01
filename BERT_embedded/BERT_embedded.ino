/*
  Control program for BERT to read positional sensor data, average it and send it to
  BERT's control system
*/

// Global constants
// interval of time to average positional sensor values for
unsigned long interval = 15;
char pos_char = 'p';
char end_character = '\n';


// These define the maximum relative angles which the BERT frame can take (in radians)
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

//These define the maximum analog voltage reading of the arduino.
//THESE ARE FOR BERT
const int rA_max = 940;
const int rA_min = 0;
const int rB_max = 780;
const int rB_min = 0;
const int rC_max = 1023;
const int rC_min = 0;


// Constants for voltage2angle.
// These value define HUE and BERTS mechanical frame dimensions
const int p = 1000;
const  int OG = 0.0800 * p; //m*p
const  int L1 = 0.255 * p; //m
const  int CL1 = 0.14142136 * p; //m
const  int L1m = 0.1100 * p; //m
const  int L2 = 0.2600 * p; //m
const  int CL2 = 0.28044607 * p; //m
const int L2e = 0.0800 * p; //m
const  int OL = 0.109337002 * p; //m
const  int OX = 0.1055 * p; //m


//check variable
bool stuck = false;

//-- function list
String voltage2step(int a_in, int b_in, int c_in);
bool checkangles(double Q1, double Q2, double Q3);

void setup()
{
  // Set serial to a baud rate of 9600 and configure analog and digital pins
  // Analog pins are for the potentiometers
  // Digital pin is for the control button

  Serial.begin(9600);
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(2, INPUT);
}

void loop()
{
  // Instantiate local variables
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  int n = 0;
  unsigned long int a = 0;
  unsigned long int b = 0;
  unsigned long int c = 0;
  //
  //  // Average control values over period of time of interval
  //  while ((currentMillis - previousMillis) < interval)
  //  {
  //    a = analogRead(A7) + a;
  //    b = analogRead(A6) + b;
  //    c = analogRead(A5) + c;
  //
  //    n = n + 1;
  //    currentMillis = millis();
  //  }
  //
  //  // Set control values to average over the time interval
  //  a = a / n;
  //  b = b / n;
  //  c = c / n;


  a = analogRead(A7);
  b = analogRead(A6);
  c = analogRead(A5);
  // Reset time
  // previousMillis = currentMillis;
  //  Serial.print("\na = ");
  //  Serial.println(a);
  //  Serial.print("\nb = ");
  //  Serial.println(b);
  //  Serial.print("\nc = ");
  //  Serial.print(c);
  //  delay(1000);

  //Convert Data and Set control data value

  String control_data = voltage2steps(a, b, c);

  // Send control data to control system if BERT's control button is pressed
  if (digitalRead(2) && control_data != "INVALID")
  {
    Serial.print(control_data);
    delay(1);
    stuck = false;
    while (digitalRead(2)) {}
  }
}



//-----------------------FUNCTIONS


bool checkangles(double Q1, double Q2, double Q3)
{
  //This function checks if the input angle is within the defined limits of HUE's range

  //If the angle would be outside HUE's range a warning is sent through the serial port.
  //If the angle is within limits a true value will be returned
  if (  Q1 <= Q1_max && Q2 <= Q2_max && Q1 >= Q1_min && Q2 >= Q2_min)
  {
    return true;
  }
  else
  {
    //    Serial.print("Angles:  ");
    //    Serial.print(Q1);
    //    Serial.print(Q2);
    //    Serial.println(Q3);
    if (stuck == false) {
      Serial.println("BERT--HUE cannot move here....");
      stuck = true;
      return false;
    }
  }
}

String voltage2steps(int a_in, int b_in, int c_in)
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

  //Convert the analog reading into the cooresponding arm angle in radians.//-=--------------------------------------------------------------------------MEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
  //  double HQ1 = a_in * (Q1_max - Q1_min) / (rA_max - rA_min);
  double Q1 = Q1_min + a_in * (Q1_max - Q1_min) / (rA_max - rA_min); //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE*********s
  double Q2 =  (Q2_min + b_in * (Q2_max - Q2_min) / (rB_max - rB_min)) + PI - Q1  - offset2;// - offset3; //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE***********
  double Q3 = -PI + (c_in)  * 2 * PI / 1023; //CHANGE THIS IF DIMENSION OF POT VALUES CHANGE********

  //  double dQ2 = Q2*180/PI;
  //  double dQ1 = Q1 *180/PI;
  //  Serial.print("Q Angles: ");
  //  Serial.print(" Q1 =  ");
  //  Serial.print(Q1);
  //  Serial.print("  Q2 =  ");
  //  Serial.println(Q2);
  //Serial.println(Q3);
  //delay(500);

  if (checkangles(Q1, Q2, Q3)) {
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
    //    Serial.print("  qS2 =  ");
    //    Serial.println(qS2);
    //Convert motor arm angle to steps based on a precision of 200 steps per rotation

    int tS1 = (3.442 - qS1) / (2 * PI) * 4 * 200; //CHANGE THIS IF alpha STEPPER CHANGES*******
    int tS2 = (4.114 - qS2) / (2 * PI) * 4 * 200; //CHANGE THIS IF beta STEPPER CHANGES********
    int tS3 = (Q3 * 13) * 200 / ( 2 * PI); // CHANGE THIS IF charlie STEPPER CHANGES*************[
    //UNCOMMENTME

    //        Serial.print("  tS2 =  ");
    //  Serial.println(tS2);

    String steps = pos_char + String(tS1) + 'a' + String(tS2) + 'b' + String(tS3) + 'c' + end_character;
    //String steps = pos_char + String(tS1) + 'a' + String(tS2) + 'b' + 0 + 'c' + end_character;
    // double then = micros();//Uncomment to end timing function
    return steps;
  } else
  {
    return "INVALID";
  }
}//voltage2angle
