int x;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  x = 0;

}

void loop() {
  Serial.print(x);
  Serial.print(",");
  delay(500);
  x++;

}
