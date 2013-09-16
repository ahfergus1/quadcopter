/*============================================================================*/
/* l3g4200d.ino - Lawrence Cushman and Andrew Simpson, 2013.
/*              - github: lawrencecushman
/*  
/*  This arduino sketch reads x, y, z-axis output data from the l3g4200d
/*  gyroscope. There are two functions defined that provide updates to this
/*  data: 
/*    - updateGyroValues(), which iteratively reads the 6 output registers
/*    AND
/*    - updateGyroValuesWithRepeatedStart(), takes advantage of the repeated
/*      start feature of I2C and allows for 3x faster read times, on average.
/*  
/*  Thus, updateGyroValuesWithRepeatedStart() should be used, if possible.
/*  
/*============================================================================*/

#include <Wire.h>

// Slave Address. In the documentation, this address is called SAD
#define DEVICE_ADDRESS 0x69

/*----------------------------------------------------------------------------*/
/*  REGISTER MAPPING
/*   The registers are further defined in the datasheet. The names are exactly 
/*   the same. Addresses with two stars (**) in the comment are used in this
/*   code. In the case of control registers, ** means they are modified from 
/*   their default values.
/*----------------------------------------------------------------------------*/
#define WHO_AM_I      0x0F // Holds the address DEVICE_ADDRESS (default 0x69)
#define CTRL_REG1     0x20 // ** Control Register 1
#define CTRL_REG2     0x21 // Control Register 2
#define CTRL_REG3     0x22 // Control Register 3
#define CTRL_REG4     0x23 // ** Control Register 4
#define CTRL_REG5     0x24 // Control Register 5
#define REFERENCE_REG 0x25 // Reference
#define OUT_TEMP      0x26 // Temperature Data
#define STATUS_REG    0x27 // Contains 8 various status bits
#define OUT_X_L       0x28 // ** X-Axis LSB
#define OUT_X_H       0x29 // ** X-Axis MSB
#define OUT_Y_L       0x2A // ** Y-Axis LSB
#define OUT_Y_H       0x2B // ** Y-Axis MSB 
#define OUT_Z_L       0x2C // ** Z-Axis LSB
#define OUT_Z_H       0x2D // ** Z-Axis MSB
#define FIFO_CTRL_REG 0x2E // FIFO Control Register
#define FIFO_SRC_REG  0x2F // Contains various FIFO Status bits
#define INT1_CFG      0x30 // Interrupt control register
#define INT1_SRC      0x31 // Contains various interrupt status bits
#define INT1_TSH_XH   0x32 // X-Axis Interrupt threshold MSB
#define INT1_TSH_XL   0x33 // X-Axis Interrupt threshold LSB
#define INT1_TSH_YH   0x34 // Y-Axis Interrupt threshold MSB
#define INT1_TSH_YL   0x35 // Y-Axis Interrupt threshold LSB
#define INT1_TSH_ZH   0x36 // Z-Axis Interrupt threshold MSB
#define INT1_TSH_ZL   0x37 // Z-Axis Interrupt threshold LSB
#define INT1_DURATION 0x38 // Interrupt duration configuration

byte x_l, y_l, z_l, x_h, y_h, z_h; // LSB and MSB of 8-bit output readings
long x,y,z;                        // 16-bit gyro output values


void setup(){
  Wire.begin();       // set up I2C
  Serial.begin(9600);
  setupGyro();
}

void loop() {
  updateGyroValuesWithRepeatedStart(); // set x, y and z.
  print_CSV();                         // print values in CSV format
  delay(50);
}


// Update Gyro Values
//  Reads the 6 gyro output registers using six separate calls to readRegister().
//  Readings are stored in the x, y, and z integer variables. The performance of
//  this function is crippled compared to the Repeated Start implementation.
void updateGyroValues(){
  x_l = readRegister(OUT_X_L);
  x_h = readRegister(OUT_X_H);
  y_l = readRegister(OUT_Y_L);
  y_h = readRegister(OUT_Y_H);
  z_l = readRegister(OUT_Z_L);
  z_h = readRegister(OUT_Z_H);
  
  x = x_l | (x_h << 8); 
  y = y_l | (y_h << 8);
  z = z_l | (z_h << 8);
}


// Update Gyro Values with Repeated Start
//  Reads the 6 gyro output registers with a single call to 
//  readSequentialRegisters(). Readings are stored in the x, y, and z integer 
//  variables.                                                                
void updateGyroValuesWithRepeatedStart(){
  byte byteArray[6];
  readSequentialRegisters(OUT_X_L, byteArray, 6); // 
  
  x = byteArray[0] | (byteArray[1] << 8); // 
  y = byteArray[2] | (byteArray[3] << 8);
  z = byteArray[4] | (byteArray[5] << 8); 
}


// Read Sequential Registers
//  Reads multiple registers with adjacent addresses. This function takes 
//  advantage of I2C's repeated start mechanism, which avoids unnecessary start
//  conditions and acknowklegements. The L3G4200D needs to be explicitly told to
//  increment to the next register after each read. If the MSb of the address is
//  1, the register address is automatically incremented.
//  For more information, see pg. 22 of the L3G4200D documentation.
// Arguments:
//  firstReg  - the address of the first register to be read.
//  byteArray - a pointer to the array the read values will be stored to
//  n         - the size of byteArray
void readSequentialRegisters(byte firstReg, byte* byteArray, int n){
  Wire.beginTransmission(DEVICE_ADDRESS);
  Wire.write(firstReg | ( 1 << 7 )); // read First Register | Auto-Increment
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


// Setup Gyro
//  Sets up the gyros control registers.
//  To take advantage of this function, toggle the bits based on your needs. The 
//  default values for each control register are 0b00000000. 
//  For more information, see pg.29 of the documentation.
void setupGyro() {
  //            REG     bit# 76543210
  writeRegister(CTRL_REG1, 0b00001111);  // default, but with 200Hz bandwidth and 50Hz cut-off freq.
  writeRegister(CTRL_REG2, 0b00000000);  // TODO: Configure this HPF, currently default
  writeRegister(CTRL_REG3, 0b00000000);  // default
  writeRegister(CTRL_REG4, 0b00110000);  // default, but with 2000dps Full scale, default was 250dps full scale
  writeRegister(CTRL_REG5, 0b00000000);  // default
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
