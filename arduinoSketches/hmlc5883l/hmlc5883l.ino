/*============================================================================*/
/* hmc5883l.ino - Lawrence Cushman and Andrew Simpson, 2013.
/*              - github: lawrencecushman
/*  
/*  This arduino sketch reads x, y, z-axis output data from the hmc5883l
/*  magnetometer. There are two functions defined that provide updates to this
/*  data: 
/*    - updateMagValues(), which iteratively reads the 6 output registers
/*    AND
/*    - updateMagValuesWithRepeatedStart(), takes advantage of the repeated
/*      start feature of I2C and allows for 3x faster read times, on average.
/*  
/*  Thus, updateMagValuesWithRepeatedStart() should be used, if possible.
/*  
/*============================================================================*/

#include <Wire.h>

// Slave Address. In the documentation, this address is called SAD
#define DEVICE_ADDRESS 0x1E

/*----------------------------------------------------------------------------*/
/*  REGISTER MAPPING
/*   The registers are further defined in the datasheet. Addresses with two 
/*   stars (**) in the comment are used in this code. In the case of control
/*   registers, ** means they are modified from their default values.
/*----------------------------------------------------------------------------*/
#define CTRL_REG_A 0x00 // ** Configuration Register A
#define CTRL_REG_B 0x01 // Configuration Register B
#define MODE_REG   0x02 // ** Control Register for the data mode
#define DATA_XH    0x03 // ** X-Axis MSB
#define DATA_XL    0x04 // ** X-Axis LSB
#define DATA_YH    0x05 // ** Y-Axis MSB
#define DATA_YL    0x06 // ** Y-Axis LSB
#define DATA_ZH    0x07 // ** Z-Axis MSB
#define DATA_ZL    0x08 // ** Z-Axis LSB
#define STATUS_REG 0x09 // Contains status bits LSb:Ready, 2LBSb:Locked
#define ID_REG_A   0x0A // Identification Register A
#define ID_REG_B   0x0B // Identification Register B 
#define ID_REG_C   0x0C // Identification Register C

byte x_l, y_l, z_l, x_h, y_h, z_h; // LSB and MSB of 8-bit output readings
long x,y,z;                        // 16-bit magnetometer output values


void setup(){
  Wire.begin();        // set up I2C
  Serial.begin(9600);
  delay(500);
  setupMagnetometer();
  updateMagValuesWithRepeatedStart();
}

void loop() {
  updateMagValues();
   // updateMagValuesWithRepeatedStart(); // set x, y and z.
    print_CSV();                        // print values in CSV format
  delay(80); // Reccomended minimum delay = 67ms(pg.18 of documentation)
}


// Update Magnetometer Values
//  Reads the 6 magnetometer output registers using six separate calls to 
//  readRegister().  Readings are stored in the x, y, and z integer variables. 
//  The performance of this function is crippled compared to the Repeated Start
//  implementation.
void updateMagValues(){
  x_l = readRegister(DATA_XL);
  x_h = readRegister(DATA_XH);
  y_l = readRegister(DATA_YL);
  y_h = readRegister(DATA_YH);
  z_l = readRegister(DATA_ZL);
  z_h = readRegister(DATA_ZH);
  
  x = x_l | (x_h << 8); 
  y = y_l | (y_h << 8);
  z = z_l | (z_h << 8);
}


// Update Magnetometer Values with Repeated Start
//  Reads the 6 magnetometer output registers with a single call to 
//  readSequentialRegisters(). Readings are stored in the x, y, and z integer 
//  variables.                                                                
void updateMagValuesWithRepeatedStart(){
  byte byteArray[6];
  readSequentialRegisters(DATA_XH, byteArray, 6); // 
  
  x = byteArray[1] | (byteArray[0] << 8); // 
  y = byteArray[3] | (byteArray[2] << 8);
  z = byteArray[5] | (byteArray[4] << 8); 
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
  Wire.write(firstReg); // read First Register | Auto-Increment
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


// Setup Magnetometer
//  Sets up the magnetometer's control registers.
//  To take advantage of this function, toggle the bits based on your needs. The 
//  default values for each control register are 0b00000000. 
//  For more information, see pg.29 of the documentation.
void setupMagnetometer() {
  //            REG      bit# 76543210
  writeRegister(CTRL_REG_A, 0b00010100);  // set data output rate to 30Hz
  writeRegister(CTRL_REG_B, 0b00000000);  // set as default
  writeRegister(MODE_REG  , 0b00000000);  // set to continuous putput mode
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
