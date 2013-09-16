/*============================================================================*/
/* adxl345.ino - Lawrence Cushman and Andrew Simpson, 2013.
/*              - github: lawrencecushman
/*  
/*  This arduino sketch reads x, y, z-axis output data from the adxl345
/*  accelerometer. There are two functions defined that provide updates to this
/*  data: 
/*    - updateAccellValues(), which iteratively reads the 6 output registers
/*    AND
/*    - updateAccellValuesWithRepeatedStart(), takes advantage of the repeated
/*      start feature of I2C and allows for 3x faster read times, on average.
/*  
/*  Thus, updateAccellValuesWithRepeatedStart() should be used, if possible.
/*  
/*============================================================================*/
#include <Wire.h>

#define DEVICE_ADDRESS 0x53  // Slave Address

/*----------------------------------------------------------------------------*/
/*  REGISTER MAPPING
/*   The registers are further defined in the datasheet. The names are exactly 
/*   the same. Addresses with two stars (**) in the comment are used in this
/*   code. In the case of control registers, ** means they are modified from 
/*   their default values.
/*----------------------------------------------------------------------------*/
#define WHO_AM_I       0x00 // Holds the value of DEVICE_ADDRESS (default: 0x53)
#define THRESH_TAP     0x1D // Tap threshold
#define OFSX           0x1E // X-axis offset - offset added to accel data
#define OFSY           0x1F // Y-axis offset - offset added to accel data
#define OFSZ           0x20 // Z-axis offset - offset added to accel data
#define DUR            0x21 // Tap duration - maximum time to be considered a tap
#define LATENT         0x22 // Tap latency - the time between tap and window
#define WINDOW         0x23 // Window - window of opportunity for 2nd tap event
#define THRESH_ACT     0x24 // Threshold value for activity 
#define THRESH_INACT   0x25 // Threshold value for inactivity
#define TIME_INACT     0x26 // Amount of time required to be considered inactive
#define ACT_INACT_CTL  0x27 // Control register for activity/inactivity options
#define THRESH_FF      0x28 // Freefall threshold
#define TIME_FF        0x29 // Minimum time to be considered freefalling
#define TAP_AXES       0x2A // Axis control for single tap/double tap
#define ACT_TAP_STATUS 0x2B // Status register for individual axis tap involvement
#define BW_RATE        0x2C // Control for output data rate and bandwidth
#define POWER_CTL      0x2D // ** Control for sleep/measure/wake-up modes
#define INT_ENABLE     0x2E // Control for various functions to be interrupts
#define INT_MAP        0x2F // Maps the functions from INT_ENABLE to INT1 or INT2
#define INT_SOURCE     0x30 // Status register for various function events
#define DATA_FORMAT    0x31 // Control for presentation of DATA
#define DATAX0         0x32 // ** X-Axis LSB
#define DATAX1         0x33 // ** X-Axis MSB
#define DATAY0         0x34 // ** Y-Axis LSB
#define DATAY1         0x35 // ** Y-Axis MSB
#define DATAZ0         0x36 // ** Z-Axis LSB
#define DATAZ1         0x37 // ** Z-Axis MSB
#define FIFO_CTL       0x38 // FIFO Control register
#define FIFO_STATUS    0x39 // Status register for various FIFO 

byte x_l, y_l, z_l, x_h, y_h, z_h;
long x,y,z;

void setup(){
  Wire.begin();  // set up I2C
  Serial.begin(9600);
  setupAccel();
}

void loop() {
  updateAccelValuesWithRepeatedStart(); // set x, y and z.
  print_CSV();                          // print values in CSV format
  delay(50);
}


// Update Accelerometer Values
//  Reads the 6 accelerometer output registers using six separate calls to 
//  readRegister(). Readings are stored in the x, y, and z integer variables. 
//  The performance of this function is crippled compared to the Repeated 
//  Start implementation.
void updateAccelValues(){
  x_l = readRegister(DATAX0);
  x_h = readRegister(DATAX1);
  y_l = readRegister(DATAY0);
  y_h = readRegister(DATAY1);
  z_l = readRegister(DATAZ0);
  z_h = readRegister(DATAZ1);
  
  x = x_l | (x_h << 8);
  y = y_l | (y_h << 8);
  z = z_l | (z_h << 8);
}


// Update Accelerometer Values with Repeated Start
//  Reads the 6 accelerometer output registers with a single call to 
//  readSequentialRegisters(). Readings are stored in the x, y, and z integer 
//  variables.  
void updateAccelValuesWithRepeatedStart(){
  byte byteArray[6];
  readSequentialRegisters(DATAX0, byteArray, 6);
  
  x = byteArray[0] | (byteArray[1] << 8);
  y = byteArray[2] | (byteArray[3] << 8);
  z = byteArray[4] | (byteArray[5] << 8);
}

// Read Sequential Registers
//  Reads multiple registers with adjacent addresses. This function takes 
//  advantage of I2C's repeated start mechanism, which avoids unnecessary start
//  conditions and acknowklegements. 
// Arguments:
//  firstReg  - the address of the first register to be read.
//  byteArray - a pointer to the array the read values will be stored to
//  n         - the size of byteArray
void readSequentialRegisters(byte firstReg, byte* byteArray, int n){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(firstReg); // read First Register
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, n);
  while(Wire.available() < n){}
  for (int i=0; i<n; i++){
    byteArray[i] = Wire.read();
  }
}


// Write to Register
//  Writes a byte to a single register.
// Arguments:
//  reg   - the register's 7 bit address
//  value - the value to be stored in the register
// Returns:
//  indicates the status of the transmission via endTransmission():
//    0:success
//    1:data too long to fit in transmit buffer
//    2:received NACK on transmit of address
//    3:received NACK on transmit of data
//    4:other error
int writeRegister(int reg, byte value) {
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission();
}

// Read Register
//  Reads data from a single register
// Arguments:
//  reg - the register to read from
// Returns:
//  The data stored in reg
byte readRegister(int reg){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(DEVICE_ADDRESS,1);
  while(Wire.available() < 1){}
  return Wire.read();
}


// Setup Accelerometer
//   By default, the ADXL345 starts in sleep mode, and needs to be placed
//   in measure mode to start taking readings. The POWER_CTL register is used
//   to enable measure mode, but as reccomended by the documentation, we
//   place the chip in standby mode first.
void setupAccel() {
  //            REG     bit# 76543210
  writeRegister(POWER_CTL, 0b00000000); // Ensure it has default settings
  writeRegister(POWER_CTL, 0b00000100); // Enable sleep mode
  writeRegister(POWER_CTL, 0b00001000); // Enable measure mode
}


// Print CSV
//  Prints x, y and z in CSV (Comma-Separated Values) format.
void print_CSV(){
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(z);
}
