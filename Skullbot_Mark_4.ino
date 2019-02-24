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
int neckServoPin = 6;
Servo servo;
int angle = 0;   // servo position in degrees
int newangle;
int sweepUnit = 2;
int sweepDelay = 15;
int sweep;

void setup()
{
  Serial.begin(9600);           // Start serial communication at 9600bps.
  Serial.print("initialising\n");
  i2c_init();                               // Initialise the i2c bus.
  PORTC = (1 << PORTC4) | (1 << PORTC5);    // Enable pullups.

  servo.attach(neckServoPin);
}

void loop()
{
  tempLeft = temperatureCelcius(device1Address);// Read's data from MLX90614
  tempRight = temperatureCelcius(device2Address);// with the given address,

  Serial.print("Left: ");
  Serial.print(tempLeft);
  Serial.print(" Right: ");
  Serial.println(tempRight);

  if(tempLeft > tempRight){
    newangle = angle - sweepUnit;
    if(newangle < 0){
      newangle = 0;
    }else{
      Serial.println("look left");
    }
  }else{
    newangle = angle + sweepUnit;
    if(newangle > 180){
      newangle = 180;
    }else{
      Serial.println("look right");
    }
  }
  // Serial.print("angle: ");
  // Serial.println(newangle);

  if(newangle != angle){
    servo.write(newangle);
    angle = newangle;
  }

  delay(1000);                         // Wait before printing again.
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
