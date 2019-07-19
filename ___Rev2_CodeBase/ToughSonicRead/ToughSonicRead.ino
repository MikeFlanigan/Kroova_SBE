
// note Max3232 chip runs on 3.3V power from the mega
int count = 0;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600); // programming console
  Serial3.begin(9600); // ultrasonic
}

void loop() {
  if (Serial3.available()) {
    int incomingByte = Serial3.read();
    Serial.print(incomingByte);
    Serial.print(' ');
    int inByte = Serial3.parseInt();
    int dist = inByte * 25.4 * 0.003384; // data coming out in mm 
    Serial.println(dist);
    count++;

    Serial1.flush(); // is this important?
  }
}
