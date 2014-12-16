int pin_1rev = 7;
int pin_1en = 24;
int pin_1fwd = 6;

int pin_2rev = 3;
int pin_2en = 25;
int pin_2fwd = 2;

void setup()
{
  Serial.begin(9600);
  pinMode(pin_1rev, OUTPUT);
  pinMode(pin_1en, OUTPUT);
  pinMode(pin_1fwd, OUTPUT);
  pinMode(pin_2rev, OUTPUT);
  pinMode(pin_2en, OUTPUT);
  pinMode(pin_2fwd, OUTPUT);
}

void loop()
{
  digitalWrite(pin_1fwd, LOW);
  digitalWrite(pin_2fwd, LOW);
  digitalWrite(pin_1rev, LOW);
  digitalWrite(pin_2rev, LOW);
  
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    
    switch (inByte) {
      case 'w':
        digitalWrite(pin_1fwd, HIGH);
        digitalWrite(pin_2fwd, HIGH);
        break;
      case 's':
        digitalWrite(pin_1rev, HIGH);
        digitalWrite(pin_2rev, HIGH);
        break;
    }
  }
  
  delay(1000);
}
