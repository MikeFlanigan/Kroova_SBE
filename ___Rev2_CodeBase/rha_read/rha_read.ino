int sensorPin = A0;
int sensorValue = 0;
int RH_setP_init = 1200 + 650; // mm !! plus half the throw of the RHA pot
int RH_setP = RH_setP_init;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = analogRead(sensorPin);
  RH_setP = RH_setP_init - map(sensorValue,140,600,0,1000);
  Serial.println(RH_setP);
}
