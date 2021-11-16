#include <math.h>

#define p 1000

const  int OG = 0.0800 * p; //m*p

const int L1 = 0.255 * p; //m
const  int CL1 = 0.14142136 * p; //m
const  int L1m = 0.1100 * p; //m

const  int L2 = 0.2600 * p; //m
const int CL2 = 0.28044607 * p; //m
const  int L2e = 0.0800 * p; //m

const  int OL = 0.109337002 * p; //m
const  int OX = 0.1055 * p; //m


double qO = 0.26681587; //rads

unsigned int interval = 15;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);

}




int voltage_convert(int qin1, int qin2) {

  //  Serial.println(OG);
  //  Serial.println(L1);
  //  Serial.println(L2);
  //
  //  qin1 = 2.41 - qin1 * 0.0023  ;
  //  qin2 =  (0.35 + qin2 * (2.63 - 0.35) / 1024) ;
  //Serial.println(qin1);
  //convert to decimal precision int
  double Q1 = 2.41 - qin1 * 0.0020996;
  double Q2 = 0.35 + qin2 * 0.002226;
  Serial.print(" Q1= ");
  Serial.print(Q1);
  Serial.print("      Q2= ");
  Serial.println(Q2);

  double now = micros();


  //stepper 1
  double q1_theta = PI   - Q1;
  //Serial.println(q1_theta);
  long O_L1m = sqrt(pow(L1m, 2) + pow(OL, 2) - 2 * L1m * OL * cos(q1_theta));
  //    Serial.print(" O_L1m = ");
  //    Serial.println(O_L1m );

  double q1_beta =  acos(( pow(OG, 2)  - pow(CL1, 2) + pow(O_L1m, 2)) / (2 * OG * O_L1m));
  double q1_alpha = acos( (pow(OL, 2)  - pow(L1m, 2) + pow(O_L1m, 2)) / (2 * OL * O_L1m ));
  double qS1 = (q1_beta + q1_alpha); //Angle of OG1 referenced to the line between center of planetary gear and pivot of L1

  //stepper 2
  double q2_theta = PI-Q2;
  long R2 = sqrt(pow(L1, 2) + pow(L2e, 2) - 2 * long(L1) * long(L2e) * cos(q2_theta));
  double q_L1_R2 = acos((-pow(L2e, 2)  + pow(R2, 2) + pow(L1, 2)) / (2 * R2 * L1));
  double q2_gamma = PI - Q1 - q_L1_R2;
  long  R1 =  sqrt(pow(OL, 2) + pow(R2, 2) - 2 * long(OL) * long(R2) * cos(q2_gamma));
  double q2_beta =  acos((pow(OG, 2) - pow(CL2, 2) + pow(R1, 2)) / (2 * OG * R1));
  double q2_alpha =  acos((pow(OL, 2) - pow(R2, 2) + pow(R1, 2)) / (2 * OL * R1));
  double qS2 = q2_alpha + q2_beta;//Angle of OG2 referenced to the line between center of planetary gear and pivot of L1
  double then = micros();



  Serial.print("  q2_alpha=  ");
  Serial.print(q2_alpha, 4);
  Serial.print("        q2_beta=  ");
  Serial.println(q2_beta, 4);
  //  Serial.print("  q2_alpha=  ");
  //  Serial.print(q2_alpha);
  //  Serial.print("        q2_beta=  ");
  //  Serial.println(q2_beta);

  Serial.print(" qS1= ");
  Serial.print(qS1, 4);
  Serial.print("          qS2 = ");
  Serial.println(qS2, 4);
  Serial.println();
  Serial.println();


    Serial.println();
  
    Serial.print("            time it took = ");
    Serial.println(then - now);
    Serial.println();
}


void loop() {
  // put your main code here, to run repeatedly:
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();
  int n = 0;
  unsigned long int a = 0;
  unsigned long int b = 0;
  unsigned long int c = 0;
  while ((currentMillis - previousMillis) < interval) {
    a = analogRead(A7) + a;
    b = analogRead(A6) + b;
    c = analogRead(A5) + c;

    n = n + 1;
    currentMillis = millis();
  }
  a = a / n;
  b = b / n;
  c = c / n;
  previousMillis = currentMillis;
  //  Serial.print("   number of samples= ");
  //  Serial.print(n);
  Serial.print("      A= ");
  Serial.print(a);
  Serial.print("      B= ");
  Serial.print(b);
  Serial.print("      C= ");
  Serial.println(c);
  voltage_convert(a, b);
  delay(1000);
}




int oldvoltage_convert(int Q1, int Q2) {


  long sec1 = cos(Q1 - 6473421825674173 / 2251799813685248);
  long sec0 = sqrt(577945169360288 - 577935636745871 * sec1);
  unsigned long qS1 = 2 * PI - acos((25 * sqrt(3) * (3467613820475226 * sec1 - 1507704304955557)) / (6442450944 * sec0)) - acos((sqrt(3) * (361209772966169375 * sec1 - 275647337930140576)) / (26843545600 * sec0));
  //takes about 600 micro seconds
  double now = micros();
  long sec4 = cos(Q2);
  long sec5 = sqrt(1632 * sec4 + 2857);
  long sec2 = acos((16 * sec4 + 51) / sec5) * sec5;
  long sec3 = 7349874591868649472 * sec4 + 197023512527001475 * sqrt(cos(Q1 + sec2 + 15021630685952347072));
  // Serial.println();
  //takes above 1000 microseconds
  // unsigned long qS2 = 7349874591868649472 * sec4 + 197023512527001475 * cos(Q1 + sec2 + 2006228090993804572) / (2147483648 * sec3);
  long qS2 = acos((7349874591868649472 * sec4 + 197023512527001475 * cos(Q1 + sec2 + 2006228090993804572) / (2147483648 * sec3))) + acos((67108864 * (7880940501080059 * cos(Q1 + sec2 + 172387724044387200)) / (7880940501080059 * sec3)));
  double then = micros();
  Serial.println();
  Serial.print(" qS2= ");
  Serial.println(qS2);
  Serial.print("   sec2 = ");
  Serial.print(sec2);
  Serial.print("   sec3 = ");
  Serial.print(sec3);
  Serial.print("  sec4 = ");
  Serial.print(sec4);
  Serial.print("   sec5 = ");
  Serial.print(sec5);
  Serial.println();

  Serial.print("            time it took = ");
  Serial.println(then - now);
  Serial.println();
}
