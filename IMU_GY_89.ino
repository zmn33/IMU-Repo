#include <Wire.h>
#define CTRL0_LSM303D 0x1F
#define CTRL1_LSM303D 0x20
#define CTRL2_LSM303D 0x21
#define CTRL3_LSM303D 0x22
#define CTRL4_LSM303D 0x23
#define CTRL5_LSM303D 0x24
#define CTRL6_LSM303D 0x25
#define CTRL7_LSM303D 0x26

#define RegMagStart 0x28

const int NrBytes = 6;
byte I2C_Addr_LSM303D = 0x1D;
byte RawDatabuffer[NrBytes]; // Array of Bytes. Each byte in the array will hold one byte of data for MSB and LSB
int Magn_rawX, Magn_rawY, Magn_rawZ, MagnX,MagnY,MagnZ;
float a_sensitivityConst = 0.244; //mg/LSB
void setup()
{
  Serial.begin(9600);
  Wire.begin();

  SetSensor(I2C_Addr_LSM303D, CTRL0_LSM303D, 0b00000000); // CTRL0(1Fh):
  SetSensor(I2C_Addr_LSM303D, CTRL1_LSM303D, 0b01010111); // CTRL1(20h): Register: acceleration data-rate is 0001 i.e. 50 Hz and all axis are enabled with continous update
  SetSensor(I2C_Addr_LSM303D, CTRL2_LSM303D, 0b00011000); // CTRL2(21h): Acceleration Anti-alias filter, full-scale selection(+- 8g), self-test, SPI mode selection
  SetSensor(I2C_Addr_LSM303D, CTRL3_LSM303D, 0b00000000); // CTRL3(22h):
  SetSensor(I2C_Addr_LSM303D, CTRL4_LSM303D, 0b00000000); // CTRL4(23h):
  SetSensor(I2C_Addr_LSM303D, CTRL5_LSM303D, 0b11110000); // CTRL5(24h): Magnetometer config
  SetSensor(I2C_Addr_LSM303D, CTRL6_LSM303D, 0b00100000); // CTRL6(25h): Magnetometer: +-4 gauss magnetic full-scale
  SetSensor(I2C_Addr_LSM303D, CTRL7_LSM303D, 0b10000000); // CTRL7(26h): MD[1:0] == [0:0] i.e. continuous conversion mode;; High-Pass Filter:Normal mode
}
//*************************************************************
void loop() {
  // put your main code here, to run repeatedly:
  ReadRawData(I2C_Addr_LSM303D, RegMagStart, 6);
  
  Serial.print(Magn_rawX*a_sensitivityConst*0.001);  // Yaw : around Z-Axis
  Serial.print("  ,  ");
  Serial.print(Magn_rawY*a_sensitivityConst*0.001); // Yaw : around Z-Axis
  Serial.print("  ,  ");
  Serial.println(Magn_rawZ*a_sensitivityConst*0.001);// Yaw : around Z-Axis
  delay(1000);
}
//*************************************************************

byte ReadBytes(byte I2C_DevAddress, byte FromRegAddress, byte NrBytes)
{
  Wire.beginTransmission(I2C_DevAddress);
  Wire.write(FromRegAddress | (1 << 7));
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_DevAddress, (byte)NrBytes, true);
  for (int i = 0; i < NrBytes; i++)
  {
    RawDatabuffer[i] = Wire.read();
  }
  return RawDatabuffer;
}
//********************************************
void ReadRawData(byte I2C_DevAddress, byte FromRegAddress, byte NrBytes)
{
  ReadBytes(I2C_DevAddress, FromRegAddress, NrBytes); // reading the registers and creating a byte pointer to the bytes of array "RawDatabuffer"
  Magn_rawX = RawDatabuffer[0] | RawDatabuffer[1] << 8;
  Magn_rawY = RawDatabuffer[2] | RawDatabuffer[3] << 8;
  Magn_rawZ = RawDatabuffer[4] | RawDatabuffer[5] << 8;
}
void SetSensor(byte I2C_DevAddress, byte RegAddress, byte SetValBin)
{
  Wire.beginTransmission(I2C_DevAddress);
  Wire.write(RegAddress);             // CTRL7 Register: Selection mode for High-pass filter AHPM1 and AHPM0 10 and Magnetic Sensor
  Wire.write(SetValBin);       // Magnetic Sensor selection mode is continoues
  Wire.endTransmission(true);
}
