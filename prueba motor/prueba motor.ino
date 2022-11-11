int digital = 2;
int vcc = 0;
int pwm_null = 9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(digital, OUTPUT);
  pinMode(vcc, OUTPUT);
  pinMode(pwm_null, OUTPUT);
  Serial.print("setuo");
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("loop");
  digitalWrite(vcc, HIGH);
  analogWrite(digital, 15);
  digitalWrite(pwm_null, LOW);
  delay(10000);
}
