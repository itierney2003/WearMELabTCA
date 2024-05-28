//goal of this code to to determine rpm of encoder
//know that the PPR is 17
//knowing this, record the time every 17 PPR to obtain seconds per revolution

#define IN1 7
#define IN2 8
#define PWM 5
#define EN_A 2
#define EN_B 3

volatile int pulse_count = 0 ;
int current_pulse_count = 0;
int previous_pulse_count = 0;

//Pulses Per Revolution of the motor
const int PPR = 17;

long previousMillis = 0;
long currentMillis = 0;
float time_interval = 0; //
float position_interval = 0;

void setup() {
  Serial.begin(9600); //Set the band rate to your Bluetooth module.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN_A, INPUT_PULLUP);
  pinMode(EN_B,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), StartInterruptA, RISING);   // ATTACHING THE INTERRUPT TO ONE OF THE ENCODER PINS TO NEVER MISS A PULSE AND IF GOT A PULSE GOTO FUNCTION StartInterruptA() 

}

void loop() {
  
  //speed of enocoder
  float current_speed = read_speed();

  //target speed
  int target_speed = 10000; //motor rated speed in RPM

  //error
  int e = target_speed - current_speed;

  //motor power
  //float power = e;
  //if(power > 255) {
  //  power = 255;    
  //}

  //signal to motor
  motorControl(200, 1);
  
  Serial.print("Position: ");
  Serial.print(pulse_count);
  Serial.print("  Previous Position: ");
  Serial.print(previous_pulse_count);
  Serial.print("  Position Interval: ");
  Serial.print(position_interval);
  Serial.print("  Time Interval: ");
  Serial.print(time_interval);
  Serial.print("  Current Time: ");
  Serial.print(currentMillis);
  Serial.print("  Current Speed: ");
  Serial.println(current_speed);

}

//interrupt service routine that increments or decrements the position count
//when encoder A produces an interrupt because a rising edge is detected, encoder B is used to determine if the position is moving forward or backwards
void StartInterruptA(){
  int b = digitalRead(EN_B);
  if (b > 0) {
    pulse_count++;
    if(pulse_count%17 == 0) {
      currentMillis = millis();
      time_interval = currentMillis - previousMillis;
      previousMillis = currentMillis;
      position_interval = pulse_count - previous_pulse_count;
      previous_pulse_count = pulse_count;
    }
  }
  else{
    pulse_count--;
  }
}

//function with two arguments; power and dirction
//the value of the power is written to the enable pin of the motor drive using PWM
//direction is used to tell the motor which way to spin
void motorControl(int pwr, int direct) {
  analogWrite(PWM, pwr);
  if (direct == 1) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    }
  else if (direct == -1) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    }
}

//function that determines the speed of the encoder
//when the pulse count = PPR (17 in this case), records the time to complete one revolution
//calculations to determine the RPM from miliseconds per revolution
float read_speed(void) {
  //current_pulse_count = pulse_count;
  float RPM;
    //(RPM = (1/ms)*(60s/min)*(10e3ms/s)
    RPM = (1/time_interval)*60*1e3;
    return RPM;
}
