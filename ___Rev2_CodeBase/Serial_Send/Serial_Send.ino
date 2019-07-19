
// note Max3232 chip runs on 3.3V power from the mega
int count = 0;
int dist = 0;

void setup() {
  Serial.begin(9600); // programming console
  Serial1.begin(9600); // BeagleBone comms
  Serial3.begin(9600); // ultrasonic
}

void loop() {
  if (Serial3.available()) {
    int incomingByte = Serial3.read();
    Serial.print(incomingByte);
    Serial.print(' ');
    int inByte = Serial3.parseInt();
    dist = inByte * 25.4 * 0.003384; // data coming out in mm
    Serial.println(dist);
    count++;

    Serial3.flush(); // is this important?
  }
  else {
    //    dist = random(1200);
    dist = 9999;
  }
  Serial1.println(dist); // send to BB
}
