#include <Servo.h>

Servo tiltServo;
Servo panServo;

String incomingByte; // for incoming serial data

int minPanPos = 1050;
int maxPanPos = 1800;
int minTiltPos = 1500;
int maxTiltPos = 1900;

int panPos = minPanPos;    // variable to store the servo position
int tiltPos = minTiltPos;    // variable to store the servo position


bool keepScanning = true;
bool panForward = true;


void printPoint(double pan, double tilt, int dis) { //Java software will read these printed texts

  Serial.print(addLeadingZeros(pan));
  Serial.print(pan);
  Serial.print(",");
  Serial.print(addLeadingZeros(tilt));
  Serial.print(tilt);
  Serial.print(",");
  Serial.print(addLeadingZeros(dis));
  Serial.print(dis);
  Serial.println();

}




String addLeadingZeros(int number) //Add leading zeros so every number sent will always be 3 digits long for consistency
{
  String leadingZeros = "";

  if (number < 10)
  {
    leadingZeros = "00";
  }
  else if (number >= 10 && number < 100)
  {
    leadingZeros = "0";
  }
  else
  {
    leadingZeros = "";

  }

  return leadingZeros;
}




void updatePos() //move servos to next position
{
  if (panPos >= maxPanPos)
  {
    panPos = minPanPos;
    tiltPos += 2;
    panServo.writeMicroseconds(panPos);
    tiltServo.writeMicroseconds(tiltPos);
    delay(2000);
    keepScanning = true;
  }
  else if (panPos < maxPanPos)
  {
    panPos += 2;
    panServo.writeMicroseconds(panPos);
    delay(15);

    keepScanning = true;
  }

  if (tiltPos > maxTiltPos)
  {
    keepScanning = false;
  }
}

void getTFminiData(int* distance, int* strength)
{
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];
  if (Serial1.available())
  {
    rx[i] = Serial1.read();
    if (rx[0] != 0x59)
    {
      i = 0;
    }
    else if (i == 1 && rx[1] != 0x59)
    {
      i = 0;
    }
    else if (i == 8)
    {
      for (j = 0; j < 8; j++)
      {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256))
      {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    }
    else
    {
      i++;
    }
  }
}


void takeMeasurement(int panPos, int tiltPos) //get a reading from the TFMini
{

  int distance = 0;
  int strength = 0;

  uint16_t dist1 = -1;

  while (dist1 < 0 || dist1 > 1500)
  {
    getTFminiData(&distance, &strength);
    if(distance) {
      dist1 = distance;
    }
  }


  uint16_t dist2 = -1;

  while (dist2 < 0 || dist2 > 1500)
  {
    getTFminiData(&distance, &strength);
    if(distance) {
      dist2 = distance;
    }
  }

  uint16_t dist3 = -1;

  while (dist3 < 0 || dist3 > 1500)
  {
    getTFminiData(&distance, &strength);
    if(distance) {
      dist3 = distance;
    }
  }



  uint16_t dist = dist1 + dist2 + dist3;

  double distAv = dist / 3;



  // Display the measurement
  printPoint(panPos/11.11111111111111111, tiltPos/11.11111111111111111, distAv);
  updatePos();
}





void setup() {

  // Step 1: Initialize hardware serial port 
  Serial.begin(115200); //For usb/bt data?

  // Step 2: Initialize the data rate for TFMini
  Serial1.begin(115200);

  panServo.attach(10);  // attaches the servo on pin
  tiltServo.attach(11);  // attaches the servo on pin
  tiltServo.writeMicroseconds(tiltPos);
  panServo.writeMicroseconds(panPos);
  delay(2000);


  //start first measurement
  takeMeasurement(panPos, tiltPos);

}



void loop() {
  if (tiltPos > maxTiltPos) //end program succesfully
  {
    keepScanning = false;
  }

  // send data only when you receive data:
  if (Serial1.available() > 0 && keepScanning) {
    keepScanning = false; //need this to make the loop not go so fast
    takeMeasurement(panPos, tiltPos);
  }
}