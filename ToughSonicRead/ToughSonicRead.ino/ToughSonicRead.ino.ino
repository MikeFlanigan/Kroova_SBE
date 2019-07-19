/*
  Multiple Serial test

  Receives from the main serial port, sends to the others.
  Receives from serial port 1, sends to the main serial (Serial 0).

  This example works only with boards with more than one serial like Arduino Mega, Due, Zero etc.

  The circuit:
  - any serial device attached to Serial port 1
  - Serial Monitor open on Serial port 0

  created 30 Dec 2008
  modified 20 May 2012
  by Tom Igoe & Jed Roach
  modified 27 Nov 2015
  by Arturo Guadalupi

  This example code is in the public domain.
*/

// note Max3232 chip runs on 3.3V power from the mega
int count = 0;

void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial3.begin(9600);
//  Serial3.setTimeout(200);
}

void loop() {
  //  Serial.println("loop...");
  // read from port 1, send to port 0:
  if (Serial3.available()) {
    //    Serial.println("am i here??");
    int incomingByte = Serial3.read();
    //    Serial.println(incomingByte);
        int inByte = Serial3.parseInt();
    //    Serial.println("or am i here??");
//    int dist = incomingByte * 25.4 * 0.003384;
        int dist = inByte*25.4*0.003384;
    //    char inByte = (char)Serial1.read();
    //    int inByte = Serial1.read();
    Serial.println(dist);
    count++;

    //    Serial1.flush();
  }
  //  else{
  //    Serial.print(count);
  //    Serial.println("no data");
  //  }

  // read from port 0, send to port 1:
  //  if (Serial.available()) {
  //    int inByte = Serial.read();
  //    Serial3.write(inByte);
  //  }
}
