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
  pinMode(LED1, OUTPUT);
}

void loop() {
  // Blink LED1 0.5 sec duration.
  digitalWrite(LED1, HIGH);
  delay(500);
  digitalWrite(LED1, LOW);
  delay(500);
}
