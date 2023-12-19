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

int toggle = 0;
int spd = 100;


void forward(int move_spd)
{
  analogWrite(MT1_PWM, move_spd);
  digitalWrite(MT1_DIR1, LOW);
  digitalWrite(MT1_DIR2, HIGH);
}

void backward(int move_spd)
{
  analogWrite(MT1_PWM, move_spd);
  digitalWrite(MT1_DIR1, HIGH);
  digitalWrite(MT1_DIR2, LOW);
}

void setup() {
  // Set LED1 as OUTPUT pin.
  pinMode(MT1_PWM, OUTPUT);
  pinMode(MT1_DIR1, OUTPUT);
  pinMode(MT1_DIR2, OUTPUT);
  pinMode(BT1, INPUT);

}

void loop() {
  if(toggle == 1 && digitalRead(BT1))
    toggle = 0;

  else if(toggle == 0 && digitalRead(BT1))
    toggle = 1;

  if(toggle)
    forward(spd);

  else if(!toggle)
    backward(spd);

}
