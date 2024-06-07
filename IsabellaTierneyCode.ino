//goal of this code to to determine rpm of encoder
//know that the PPR is 17
//knowing this, record the time every 17 PPR to obtain seconds per revolution
//also use a potentiometer to use PWM to control the speed

#define IN1 7
#define IN2 8
#define PWM 5
#define EN_A 2
#define EN_B 3

int potentiometer = 0; //value of analog value read from the potentiometer
int power = 0; //signal sent to the enable pin

volatile double pulse_count = 0 ;
double previous_pulse_count = 0;

//Pulses Per Revolution of the motor
const int PPR = 17;

long previousMillis = 0;
long currentMillis = 0;
long previousMillisMain = 0;
float time_interval = 0; //currentMillis - previousMillis
float position_interval = 0; //pulse_count - previous_pulse_count //should always be 17

float eprev = 0; //previous error
float eintegral = 0; //error integral

void setup() {
  Serial.begin(115200); //Set the band rate to your Bluetooth module.
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(EN_A, INPUT_PULLUP);
  pinMode(EN_B,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EN_A), StartInterruptA, RISING);   // ATTACHING THE INTERRUPT TO ONE OF THE ENCODER PINS TO NEVER MISS A PULSE AND IF GOT A PULSE GOTO FUNCTION StartInterruptA() 

}

void loop() {

  //start timer
  currentMillis = millis();

  //power
  potentiometer = analogRead(A0);
  power = map(potentiometer, 0, 1023, 0, 255);

  //target speed
  float target_speed = desired_speed();

  //speed of enocoder
  float current_speed = read_speed();

  //PID Constants
  float kp = 1;
  float kd = 0;
  float ki = 0;

  float deltaT = ((float)(currentMillis-previousMillisMain))/1.0e6;
  previousMillisMain = currentMillis;

  //error
  int e = target_speed - current_speed;

  //derivative
  float dedt = (e-eprev)/(deltaT);

  //integral
  eintegral = eintegral + e*deltaT;

  //control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  power = fabs(u);
    if( power > 255 ) {
    power = 255;
  }

  //signal to motor
  motorControl(power, 1);

  Serial.print(currentMillis);
  Serial.print(",");
  Serial.print(current_speed);
  Serial.print(",");
  Serial.print(target_speed);
  Serial.print("  PWM Power ");
  Serial.println("power");

}

//interrupt service routine that increments or decrements the position count
//when encoder A produces an interrupt because a rising edge is detected, encoder B is used to determine if the position is moving forward or backwards
void StartInterruptA(){

  int b = digitalRead(EN_B);

  if (b > 0) {
    pulse_count++;
    if((int)pulse_count%17 == 0) {
      time_interval = currentMillis - previousMillis;
      previousMillis = currentMillis;
    }
  }
  else{
    pulse_count--;
  }
}

//function with two arguments; power and dirction
//the value of the power is written to the enable pin of the motor drive using PWM
//direction is used to tell the motor which way to spin
void motorControl(int power, int direct) {

  analogWrite(PWM, power);

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
  float RPM;
    //(RPM = (1/ms)*(60s/min)*(10e3ms/s)
    RPM = (1/time_interval)*60*1e3;
    return RPM;
}

//function to calculate the desired speed based on the set potentiometer value 
float desired_speed(void) {
  float speed;
  //max PWM is 255
  //whatever the power is (0-255)*rated speed of motor is the speed
  speed = (power/255)*10000;
  return speed;
}
