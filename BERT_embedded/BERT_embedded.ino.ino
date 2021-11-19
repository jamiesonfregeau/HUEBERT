/*
Control program for BERT to read positional sensor data, average it and send it to
BERT's control system
*/

// Global constants
// interval of time to average positional sensor values for
unsigned long interval = 15;
char positional_character = "p";
char final_character = "\n";

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

  // Average control values over period of time of interval
  while ((currentMillis - previousMillis) < interval)
  {
    a = analogRead(A7) + a;
    b = analogRead(A6) + b;
    c = analogRead(A5) + c;

    n = n + 1;
    currentMillis = millis();
  }

  // Set control values to average over the time interval
  a = a / n;
  b = b / n;
  c = c / n;

  // Reset time
  previousMillis = currentMillis;

  // Set control data value
  String control_data = positional_character + String(a) + String(b) + String(c) + final_character;

  // Send control data to control system if BERT's control button is pressed
  if (digitalRead(2))
  {
    Serial.print(control_data);
  }
}
