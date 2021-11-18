unsigned long interval = 5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A7, INPUT);
  pinMode(A6, INPUT);
  pinMode(A5, INPUT);
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
  Serial.print("number of samples= ");
  Serial.println(n);
  Serial.print("input left= ");
  Serial.println(b);
  Serial.print("input right= ");
  Serial.println(c);
  Serial.print("outpute= ");
  Serial.println(a);
  delay(1000);

}
