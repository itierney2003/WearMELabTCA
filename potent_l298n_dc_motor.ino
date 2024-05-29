
#define IenableTheThing 9
#define imTheForwardIN1 6
#define imTheReverseIN2 5
#define imPotNSpeedControl A0 
int potentiometer  = 0;
int IjustObey = 0;

void setup(){
  pinMode(IenableTheThing, OUTPUT);
  pinMode( imTheReverseIN2, OUTPUT);
  pinMode(imTheForwardIN1, OUTPUT);
 Serial.begin(9600);
  
   
}
 
void loop() {
   potentiometer = analogRead(imPotNSpeedControl);
 
    digitalWrite(imTheReverseIN2, HIGH);
    digitalWrite(imTheForwardIN1, LOW);
    IjustObey = map(potentiometer, 0, 1023, 0, 255);
    Serial.print("Pot:");
    Serial.print(potentiometer);
    Serial.print("Obey:");
    Serial.print(IjustObey);
    delay(1000);
  
  analogWrite(IenableTheThing, IjustObey);
  Serial.println("  " );
 
  }
