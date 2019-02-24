/**
 * A people detecting & targeting robot
 *
 * http://wiki.wiring.co/wiki/Connecting_Infrared_Thermometer_MLX90614_to_Wiring#Address_Changing
 */

#include <i2cmaster.h>
#include <Servo.h>

// eyes
int device1Address = 0x5A<<1;   // 0x5A is the assigned address for I²C
int device2Address = 0x55<<1;   // 0x55 is the re-assigned address for I²C

float tempLeft = 0;             // Variable to hold temperature in Celcius for left eye
float tempRight = 0;             // Variable to hold temperature in Celcius for right eye

// neck
Servo rotateServo;
int rotateServoPin = 9;
int rotateAngle = 0;   // servo position in degrees
int newRotateAngle;
int sweepUnit = 0.2;
int sweepDelay = 15;
int sweep;
int rotateLeftLimit;  // how far left and right
int rotateRightLimit;
int rotateCentre;  // centre point

int rotateTempMargin = 10;

Servo tiltServo;
int tiltServoPin = 8;
int tiltAngle = 0;   // servo position in degrees
int newTiltAngle;
int tiltSweepUnit = 0.2;
int tiltSweepDelay = 15;
int tiltSweep;
int tiltFrontLimit; // how far front and back
int tiltBackLimit;
int tiltCentre; // centre point

int aveTemp; // the average of the two eyes

void setup()
{
  Serial.begin(9600);           // Start serial communication at 9600bps.
  Serial.print("initialising\n");
  i2c_init();                               // Initialise the i2c bus.
  PORTC = (1 << PORTC4) | (1 << PORTC5);    // Enable pullups.

  rotateServo.attach(rotateServoPin);
  tiltServo.attach(tiltServoPin);

  calibrateServos();
}

void calibrateServos()
{
  Serial.print("calibbrating\n");
  Serial.print("rotating\n");
  for (int x = rotateLeftLimit; x <= rotateRightLimit; x += 5){
    rotateServo.write(x);
    Serial.print(x);
    Serial.print("\n");
    delay(1000);
  }
  rotateServo.write(rotateCentre);

  Serial.print("tilting\n");
  for (int y = tiltFrontLimit; y <= tiltBackLimit; y += 10){
    tiltServo.write(y);
    Serial.print(y);
    Serial.print("\n");
    delay(1000);
  }
  tiltServo.write(tiltCentre);
}

void loop()
{
  // temperature
  tempLeft = temperatureCelcius(device1Address);  // Read's data from MLX90614
  tempRight = temperatureCelcius(device2Address); // with the given address,

  Serial.print("Left: ");
  Serial.print(tempLeft);
  Serial.print(" Right: ");
  Serial.println(tempRight);
  Serial.print("\n");

  // for testing purposes
  return;

  // rotate
  int tempDifference = tempLeft - tempRight;          //  VERIFY this
  if(tempDifference < 0) tempDifference = -tempDifference;
  
  if(tempDifference >= rotateTempMargin){          //  VERIFY this
    if(tempLeft > tempRight){          //  VERIFY this
      newRotateAngle = rotateAngle - sweepUnit;          //  VERIFY this
      if(newRotateAngle < rotateLeftLimit){          //  VERIFY this
        newRotateAngle = rotateLeftLimit;
      }else{
        Serial.println("look left\n");
      }
    }else if(tempLeft < tempRight){          //  VERIFY this
      newRotateAngle = rotateAngle + sweepUnit;          //  VERIFY this
      if(newRotateAngle > rotateLeftLimit){          //  VERIFY this
        newRotateAngle = rotateRightLimit;
      }else{
        Serial.println("look right\n");
      }
    }
  }
 
  Serial.print("rotateAngle: ");
  Serial.println(newRotateAngle);
  Serial.print("\n");
  
  if(newRotateAngle != rotateAngle){
    rotateServo.write(newRotateAngle);
    rotateAngle = newRotateAngle;
  }

  // tilt
  int newAveTemp = (tempLeft + tempRight) / 2;
  if(newAveTemp > aveTemp){ // look up
    newTiltAngle = tiltAngle + tiltSweepUnit;
    if(newTiltAngle < tiltFrontLimit){          //  VERIFY this
      newTiltAngle = tiltBackLimit;
    }else{
      Serial.println("look up");
    }
  }else if(newAveTemp < aveTemp){          //  VERIFY this
    newTiltAngle = tiltAngle - sweepUnit;
    if(newTiltAngle > tiltBackLimit){
      newTiltAngle = tiltBackLimit;
    }else{
      Serial.println("look down");
    }
  }
  
  Serial.print("tileAngle: ");
  Serial.println(newTiltAngle);

  if(newTiltAngle != tiltAngle){
    tiltServo.write(newTiltAngle);
    tiltAngle = newTiltAngle;
    aveTemp = newAveTemp;
  }

  delay(1000);
}

float temperatureCelcius(int address) {
  int dev = address;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;

  // Write
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);

  // Read
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck();       // Read 1 byte and then send ack.
  data_high = i2c_readAck();      // Read 1 byte and then send ack.
  pec = i2c_readNak();
  i2c_stop();

  // This converts high and low bytes together and processes temperature,
  // MSB is a error bit and is ignored for temps.
  double tempFactor = 0.02;       // 0.02 degrees per LSB (measurement
                                  // resolution of the MLX90614).
  double tempData = 0x0000;       // Zero out the data
  int frac;                       // Data past the decimal point

  // This masks off the error bit of the high byte, then moves it left
  // 8 bits and adds the low byte.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  float celcius = tempData - 273.15;

  // Returns temperature un Celcius.
  return celcius;
}
