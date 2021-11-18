unsigned long interval = 15;
void setup() {
 // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
  pinMode(2, INPUT);
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

  String control_data = "p" + String(a) + String(b) + String(c)+ "\n";
//  Serial.print(control_data);
  
 if(digitalRead(2)){
    Serial.print(control_data);
}
//  

//  a = analogRead(A7);
//  b = analogRead(A6);
//  c = analogRead(A5);
//   
//  Serial.print("      A= ");
//  Serial.print(a);
//  Serial.print("      B= ");
//  Serial.print(b);
//  Serial.print("      C= ");
//  Serial.println(c);
//  delay(10);

}
