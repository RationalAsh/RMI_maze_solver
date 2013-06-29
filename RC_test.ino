int basePin1 = 13;
int basePin2 = 12;
int basePin3 = 11;
int basePin4 = 10;
int delay_time = 10;

int serial_flag=0;

void setup()
{
  pinMode(basePin1, OUTPUT);
  pinMode(basePin2, OUTPUT);
  pinMode(basePin3, OUTPUT);
  pinMode(basePin4, OUTPUT);
  Serial.begin(9600);
}

void forward()
{
  digitalWrite(basePin1, HIGH);
  digitalWrite(basePin2, LOW);
  digitalWrite(basePin3, LOW);
  digitalWrite(basePin4, LOW);
}

void back()
{
  digitalWrite(basePin1, LOW);
  digitalWrite(basePin2, HIGH);
  digitalWrite(basePin3, LOW);
  digitalWrite(basePin4, LOW);
}

void right()
{
  digitalWrite(basePin1, LOW);
  digitalWrite(basePin2, LOW);
  digitalWrite(basePin3, HIGH);
  digitalWrite(basePin4, LOW);
}

void left()
{
  digitalWrite(basePin1, LOW);
  digitalWrite(basePin2, LOW);
  digitalWrite(basePin3, LOW);
  digitalWrite(basePin4, HIGH);
}

void halt()
{
  digitalWrite(basePin1, HIGH);
  digitalWrite(basePin2, LOW);
  digitalWrite(basePin3, HIGH);
  digitalWrite(basePin4, LOW);
}

void loop()
{
  //Serial.print('H');
  delay(10);
}

void serialEvent()
{
  while(Serial.available())
  {
    char inChar = (char)Serial.read();
    if(inChar == 'W') forward();
    if(inChar == 'A') left();
    if(inChar == 'S') back();
    if(inChar == 'D') right();
    //Serial.print(inChar);
  }
}
