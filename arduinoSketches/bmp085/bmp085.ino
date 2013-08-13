/*============================================================================*/
/* bmp085.ino - Lawrence Cushman and Andrew Simpson, 2013.
/*              - github: lawrencecushman
/*  
/*  This arduino sketch reads and displays temperature and pressure data from
/*  the bmp085 barometer.
/*
/*============================================================================*/

#include <Wire.h>

#define DEVICE_ADDRESS 0x77 // Slave Address

#define READ_TEMP        0x2E // Temperature option
#define READ_PRESSURE    0x34 // Pressure option

/*----------------------------------------------------------------------------*/
//  REGISTER MAPPING
#define CTRL_REG      0xF4 // Control register for output (temp or pressure)
#define OUTPUT_REG_H  0xF6 // Data output MSB
#define OUTPUT_REG_L  0xF7 // Data output LSB
#define OUTPUT_REG_XL 0xF8 // Data output XLSB (used in high-res mode)

//  Calibration Coefficients - The following 22 registers hold the 11 16-bit
//    coefficients that are specific to your individual chip. The coefficients
//    are used to help calculate pressure and temperature. They need only be 
//    read once at the beginning of operation
#define AC_1H   0xAA
#define AC_1L   0xAB
#define AC_2H   0xAC
#define AC_2L   0xAD
#define AC_3H   0xAE
#define AC_3L   0xAF
#define AC_4H   0xB0
#define AC_4L   0xB1
#define AC_5H   0xB2
#define AC_5L   0xB3
#define AC_6H   0xB4
#define AC_6L   0xB5
#define B_1H    0xB6
#define B_1L    0xB7
#define B_2H    0xB8
#define B_2L    0xB9
#define MB_H    0xBA
#define MB_L    0xBB
#define MC_H    0xBC
#define MC_L    0xBD
#define MC_H    0xBE
#define MC_L    0xBF

const unsigned char oss = 3; // I2C slave address for bmp085

// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

void setup(){
  Wire.begin();          // set up I2C
  Serial.begin(9600);
  getCalibrationCoefficients();
  printCalibrationData();
}

void loop() {
  int temperature;
  long pressure;
  
  updateBaroValues(&temperature, &pressure); // set pressure and temperature
  Serial.print ("T:");
  Serial.print (temperature);
  Serial.print (" P:");
  Serial.println (pressure);
  
  delay(80);
}

void updateBaroValues(int* t, long* p){
  long x1, x2, x3, b3, b5, b6;
  unsigned long b4, b7;
  
  // Read uncompensated temperature value
  writeRegister(0xF4,0x2E);
  delay(5);
  int ut = (readRegister(0xF6) << 8) | readRegister(0xF7);
  Serial.print("UT:");
  Serial.print(ut);

  // Read uncompensated puessure data
  writeRegister(0xF4,0x2E);
  delay(5);
  long up = ((readRegister(0xF6) << 16) | (readRegister(0xF7) << 8) |
                  readRegister(0xF8)) >> (8 - oss);
  Serial.print(" UP:");
  Serial.println(up);  
  
  // Calculate true temperature
  x1 = (ut - ac6) * ac5;
  x1 >>= 15;
  b5 = x1 + x2;
  *t = b5 + 8;
  *t >>= 4;
  
  // Calculate true pressure
  b6 = b5-4000;
  x1 = (b6 * b6) >> 12;
  x1 *= b2;
  x1 >>= 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << oss) + 2) >> 2;
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * (b6 * b6) >> 12) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = ac4 * (unsigned long)(x3 + 32768) >> 15;
  b7 = ((unsigned long)up - b3)*(50000 >> oss);
  if(b7 < 0x80000000){
    *p = (b7 * 2) / b4;
  }
  else{
    *p = (b7 / b4) *2;
  }
  x1 = (*p >> 8);
  x1 *= x1;
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * *p) >> 16;
  *p += (x1 + x2 + 3791) >> 4;
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


// Setup Barometer
//  Sets up the barometer's control registers.
//  To take advantage of this function, toggle the bits based on your needs. The 
//  default values for each control register are 0b00000000. 
//  For more information, see pg.29 of the documentation.
void setupBarometer() {
  getCalibrationCoefficients();
}


// Get Calibration Coefficients
//  Reads the calibration coefficiets form the bmp085's EEPROM and sets their
//  respective variables
void getCalibrationCoefficients(){
  byte byteArray[22];

  // store value inside AC_1H and the next 21 register's values inside byteArray
  readSequentialRegisters(AC_1H, byteArray, 22);
  
  ac1 = (byteArray[0]  << 8 ) | byteArray[1];
  ac2 = (byteArray[2]  << 8 ) | byteArray[3];
  ac3 = (byteArray[4]  << 8 ) | byteArray[5];
  ac4 = (byteArray[6]  << 8 ) | byteArray[7];
  ac5 = (byteArray[8]  << 8 ) | byteArray[9];
  ac6 = (byteArray[10] << 8 ) | byteArray[11];
  b1  = (byteArray[12] << 8 ) | byteArray[13];
  b2  = (byteArray[14] << 8 ) | byteArray[15];
  mb  = (byteArray[16] << 8 ) | byteArray[17];
  mc  = (byteArray[18] << 8 ) | byteArray[19];
  md  = (byteArray[20] << 8 ) | byteArray[21];  
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
  Wire.write(firstReg); // read First Register | Auto-Increment
  Wire.endTransmission();

  Wire.requestFrom(DEVICE_ADDRESS, n);
  while(Wire.available() < n){}
  for (int i=0; i<n; i++){
    byteArray[i] = Wire.read();
  }
}

// Print Calibration Data
//  Prints the Calibration data stored in the bmp085's EEPROM
void printCalibrationData(){
  Serial.println("========== Calibration Data ==========");
  Serial.print("AC1:"); Serial.print(ac1); Serial.print(", ");
  Serial.print("AC2:"); Serial.print(ac2); Serial.print(", ");
  Serial.print("AC3:"); Serial.print(ac3); Serial.print(", ");
  Serial.print("AC4:"); Serial.print(ac4); Serial.print(", ");
  Serial.print("AC5:"); Serial.print(ac5); Serial.print(", ");
  Serial.print("AC6:"); Serial.println(ac6);
  
  Serial.print("B1:"); Serial.print(b1); Serial.print(", ");
  Serial.print("B2:"); Serial.print(b2); Serial.print(", ");
  Serial.print("MB:"); Serial.print(mb); Serial.print(", ");
  Serial.print("MC:"); Serial.print(mc); Serial.print(", ");
  Serial.print("MD:"); Serial.println(md);  
}
