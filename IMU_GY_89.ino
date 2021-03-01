/* Created by zmn33: This program code reads the 3-Axis Accelerometer, Magnetometer
,Temperature Sensor measurements from GY-89 breakout board */

#include <Wire.h>

#define CTRL0_LSM303D 0x1F
#define CTRL1_LSM303D 0x20
#define CTRL2_LSM303D 0x21
#define CTRL3_LSM303D 0x22
#define CTRL4_LSM303D 0x23
#define CTRL5_LSM303D 0x24
#define CTRL6_LSM303D 0x25
#define CTRL7_LSM303D 0x26

#define RegMagStart 0x08
#define RegAccStart 0x28
#define RegTempStart 0x05

const int NrBytes = 6;
byte I2C_Addr_LSM303D = 0x1D;
byte RawDatabuffer[NrBytes]; // Array of Bytes. Each byte in the array will hold one byte of data for MSB and LSB
float rawX, rawY, rawZ;
float Acc_X, Acc_Y, Acc_Z, MagnX, MagnY, MagnZ;
float T_raw, Temp;

float a_sensitivityConst, m_sensitivityConst, T_Const;
//******************** Setup section **************************
void setup()
{
  Serial.begin(9600);
  Wire.begin();
  GY89_Setup();
}
//********************* Main loop *******************************
void loop() {
  // put your main code here, to run repeatedly:
  getAcc();
  getMag();
  Temp = getTemp();

  Serial.print(Acc_X);
  Serial.print("  ,  ");
  Serial.print(Acc_Y);
  Serial.print("  ,  ");
  Serial.print(Acc_Z);
  Serial.print("  ,  ");

  Serial.print(MagnX);
  Serial.print("  ,  ");
  Serial.print(MagnY);
  Serial.print("  ,  ");
  Serial.print(MagnZ);
  Serial.print("  ,  ");

  Serial.print(  a_sensitivityConst);
  Serial.print("  ,  ");
  Serial.print(  m_sensitivityConst);
  Serial.print("  ,  ");
  Serial.print(  T_Const);
  Serial.print("  ,  ");

  Serial.println(Temp);


  delay(1000);
}
//*************************************************************

byte ReadBytes(byte I2C_DevAddress, byte FromRegAddress, byte NrBytes)
{
  Wire.beginTransmission(I2C_DevAddress);
  Wire.write(FromRegAddress | (1 << 7));// inserting 1 to MSB of the register to indicate reading, otherwise it is zero and means writing
  Wire.endTransmission(false);
  Wire.requestFrom(I2C_DevAddress, (byte)NrBytes, true);
  for (int i = 0; i < NrBytes; i++)
  {
    RawDatabuffer[i] = Wire.read();
  }
  return RawDatabuffer;
}
//********************************************
void readRawData(byte I2C_DevAddress, byte FromRegAddress, byte NrBytes)
{
  ReadBytes(I2C_DevAddress, FromRegAddress, NrBytes); // reading the registers and creating a byte pointer to the bytes of array "RawDatabuffer"
  rawX = RawDatabuffer[0] | RawDatabuffer[1] << 8;
  rawY = RawDatabuffer[2] | RawDatabuffer[3] << 8;
  rawZ = RawDatabuffer[4] | RawDatabuffer[5] << 8;
}
void getAcc()
{
  readRawData(I2C_Addr_LSM303D, RegAccStart, 6);
  Acc_X = rawX * a_sensitivityConst * 0.001 * 9.81; // 0.001 converts from mg to g and 9.81 from g to m/s^2
  Acc_Y = rawY * a_sensitivityConst * 0.001 * 9.81;
  Acc_Z = rawZ * a_sensitivityConst * 0.001 * 9.81;
}
void getMag()
{
  readRawData(I2C_Addr_LSM303D, RegMagStart, 6);
  MagnX = rawX * m_sensitivityConst * 0.001; // 0.001 converts from mgauss to gauss
  MagnY = rawY * m_sensitivityConst * 0.001;
  MagnZ = rawZ * m_sensitivityConst * 0.001;
}
float getTemp()
{
  float Temperature;
  readRawData(I2C_Addr_LSM303D, RegTempStart, 2);
  T_raw = rawX;
  Temperature = T_raw / T_Const;
  return Temperature;
}
void SetSensor(byte I2C_DevAddress, byte RegAddress, byte SetValBin)
{
  Wire.beginTransmission(I2C_DevAddress);
  Wire.write(RegAddress);             // CTRL7 Register: Selection mode for High-pass filter AHPM1 and AHPM0 10 and Magnetic Sensor
  Wire.write(SetValBin);       // Magnetic Sensor selection mode is continoues
  Wire.endTransmission(true);
}
void GY89_Setup()
{
  SetSensor(I2C_Addr_LSM303D, CTRL0_LSM303D, 0b00000000); // CTRL0(1Fh):
  SetSensor(I2C_Addr_LSM303D, CTRL1_LSM303D, 0b01010111); // CTRL1(20h): Register: acceleration data-rate is 0001 i.e. 50 Hz and all axis are enabled with continous update
  SetSensor(I2C_Addr_LSM303D, CTRL2_LSM303D, 0b00011000); // CTRL2(21h): Acceleration Anti-alias filter, full-scale selection(+- 8g), self-test, SPI mode selection
  SetSensor(I2C_Addr_LSM303D, CTRL3_LSM303D, 0b00000000); // CTRL3(22h):
  SetSensor(I2C_Addr_LSM303D, CTRL4_LSM303D, 0b00000000); // CTRL4(23h):
  SetSensor(I2C_Addr_LSM303D, CTRL5_LSM303D, 0b11110000); // CTRL5(24h): Magnetometer config
  SetSensor(I2C_Addr_LSM303D, CTRL6_LSM303D, 0b00100000); // CTRL6(25h): Magnetometer: +-4 gauss magnetic full-scale
  SetSensor(I2C_Addr_LSM303D, CTRL7_LSM303D, 0b10000000); // CTRL7(26h): MD[1:0] == [0:0] i.e. continuous conversion mode;; High-Pass Filter:Normal mode

  // Sensitivity settings
  getSensConst('a', 8);
  getSensConst('m', 4);
  getSensConst('T', 0);

}
float getSensConst(char SensType, int FullScaleSens)
{
  if (SensType == 'a') // a for accelerometer
  {
    if (FullScaleSens == 2)// +-2g
    {
      a_sensitivityConst = 0.061; // 0.061 mg/LSB
    }
    else if (FullScaleSens == 4)
    {
      a_sensitivityConst = 0.122;
    }
    else if (FullScaleSens == 6)
    {
      a_sensitivityConst = 0.183;
    }
    else if (FullScaleSens == 8)
    {
      a_sensitivityConst = 0.244;
    }
    else if (FullScaleSens == 16)
    {
      a_sensitivityConst = 0.732;
    }
  }
  else if (SensType == 'm')// m for magnetometer
  {
    if (FullScaleSens == 2)
    {
      m_sensitivityConst = 0.080; // 0.061 mg/LSB
    }
    else if (FullScaleSens == 4)
    {
      m_sensitivityConst = 0.160; // 0.061 mg/LSB
    }
    else if (FullScaleSens == 8)
    {
      m_sensitivityConst = 0.320; // 0.061 mg/LSB
    }
    else if (FullScaleSens == 12)
    {
      m_sensitivityConst = 0.479; // 0.061 mg/LSB
    }
  }
  else if (SensType == 'T' && FullScaleSens == 0)
  {
    T_Const = 8; // 8 LSB/C°
  }
  else
  {
    a_sensitivityConst = 0.061;
    m_sensitivityConst = 0.160;
    T_Const = 8;
  }
}
