#include <AccelStepper.h>
#include <math.h>

#define MOTOR_X_ENABLE_PIN 8
#define MOTOR_X_STEP_PIN 3
#define MOTOR_X_DIR_PIN 4

#define MOTOR_Y_ENABLE_PIN 8
#define MOTOR_Y_STEP_PIN 3
#define MOTOR_Y_DIR_PIN 6

#define MOTOR_Z_ENABLE_PIN 8
#define MOTOR_Z_STEP_PIN 4
#define MOTOR_Z_DIR_PIN 7

//functions

//constants
int maxsteps = 1040;
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
  alpha.setAcceleration(500);
  alpha.enableOutputs();
  alpha.setMaxSpeed(150.0);
   int a = analogRead(A7);
    int am = a/2;
  alpha.setCurrentPosition(am);
  //---------------------------
  Serial.begin(9600);

  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(A3, INPUT);
  pinMode(A1, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);

}

//---------------------------------------------
void loop()
{


  while (digitalRead(A3) == 0 && check == 1)
  {
    check == 1;
    digitalWrite(8, LOW);
    int a = analogRead(A7);
    int am = a/3.25;
    //    analogWrite(11, am);

    //Serial.println(digitalRead(A3));
//    int da = abs(alpha.currentPosition() - a);
//    Serial.print("pot: ");
//    Serial.println(a);
//    Serial.print("difference: ");
//    Serial.println(da);
    //
    //    delay(100);

    alpha.moveTo(am);

    alpha.run();
  }

  if ((digitalRead(A3) == 1))
  {
    if (check == 1)
    {
      alpha.disableOutputs();
      beta.disableOutputs();
      charlie.disableOutputs();
      analogWrite(11, 0);
      analogWrite(10, 0);
      analogWrite(A1, 0);
      digitalWrite(8, HIGH);
      check = 0;
    }
    else if (check == 0)
    {
      alpha.enableOutputs();
      beta.enableOutputs();
      charlie.enableOutputs();
      digitalWrite(8, LOW);
      check = 1;
    }

    while (digitalRead(A3) == 1) {}
  }
}
//---------------------------------------------


int motorgoto(int a, int b, int c)
{
  alpha.moveTo(a);
  beta.moveTo(b);
  charlie.moveTo(c);

  Serial.print("MOVING TO: ");
  Serial.print(alpha.targetPosition());
  Serial.print(", ");
  Serial.print(beta.targetPosition());
  Serial.print(", ");
  Serial.println(charlie.targetPosition());
  Serial.print("CURRENT POSITION: ");
  Serial.print(alpha.currentPosition());
  Serial.print(", ");
  Serial.print(beta.currentPosition());
  Serial.print(", ");
  Serial.println(charlie.currentPosition());


  while (alpha.distanceToGo() != 0 || beta.distanceToGo() != 0 || charlie.distanceToGo() != 0)
  {
    alpha.run();
    beta.run();
    charlie.run();
  }
}
