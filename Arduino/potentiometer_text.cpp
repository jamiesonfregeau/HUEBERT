void setup()
{

    Serial.begin(9600);
}

void loop()
{

    int pot1_value = analogRead(A0);
    int pot2_value = analogRead(A1);
    int pot3_value = analogRead(A2);

    Serial.print("Pot1 Value: ");
    Serial.println(pot1_value);
    Serial.print("Pot2 Value: ");
    Serial.println(pot2_value);
    Serial.print("Pot3 Value: ");
    Serial.println(pot3_value);
    Serial.println("--------------");
    Serial.println("");

    delay(1000);
}