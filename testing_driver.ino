int enA = 9;  //rotation speed pin
int in1 = 8;  //rotation direction pin
int in2 = 7;  //rotation direction pin

void setup()
{
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void demo()
{
  for(int i = 0; i < 10; i ++){
    digitalWrite(in1, LOW);   //normal rotation
    digitalWrite(in2, HIGH);  

    analogWrite(enA, 200);    //speed rotation

    delay(2000);

    digitalWrite(in1, HIGH);  //inverse rotation
    digitalWrite(in2, LOW);
    analogWrite(enA, 20);     //inverse rotation speed

    delay(2000);

  }
}

void loop()
{
  demo();
}
