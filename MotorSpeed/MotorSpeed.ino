#define LED1 3
#define LED2 4

#define BT1 14
#define BT2 15

#define MT1_PWM 5
#define MT1_DIR1 7
#define MT1_DIR2 8

#define MT2_PWM 6
#define MT2_DIR1 9
#define MT2_DIR2 10

void setup() {
  // Set LED1 as OUTPUT pin.
  pinMode(MT1_PWM, OUTPUT);
  pinMode(MT1_DIR1, OUTPUT);
  pinMode(MT1_DIR2, OUTPUT);

}

void loop() {
  analogWrite(MT1_PWM, 100);
  digitalWrite(MT1_DIR1, HIGH);
  digitalWrite(MT1_DIR2, LOW);
  delay(1000);

  analogWrite(MT1_PWM, 255);
  digitalWrite(MT1_DIR1, HIGH);
  digitalWrite(MT1_DIR2, LOW);
  delay(1000);
}
