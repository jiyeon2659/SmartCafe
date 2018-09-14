void setup() {
  // put your setup code here, to run once:
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(9, 255); // orange
  analogWrite(10, 50);
  analogWrite(11, 0);
  delay(1000);

  analogWrite(9, 255); // analog yellow
  analogWrite(10, 100);
  analogWrite(11, 0);
  delay(1000);

  analogWrite(9, 255);
  analogWrite(10, 200);
  analogWrite(11, 0);
  delay(1000);

  digitalWrite(9, HIGH); // digital yellow
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
  delay(1000);
  
  digitalWrite(9, HIGH); // white
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  delay(1000);
}
