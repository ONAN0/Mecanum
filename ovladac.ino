#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>

#define p1 7

const int MPU = 0x68;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AccUholX, AccUholY, GyroUholX, GyroUholY;
float uholX, uholY;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;
float Prejdcas, Teraz, Predcas;
int c = 0;

RF24 radio(5, 6);

const byte address[6] = "00001";

struct Data_Package
{
  byte j1PotX;
  byte j1PotY;
  byte j2PotX;
  byte pot1;
};

Data_Package data;

void setup()
{
  Serial.begin(9600);

  initialize_MPU6050();
  
  //calculate_IMU_error();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);

  pinMode(p1, INPUT_PULLUP);
  
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.pot1 = 1;
}

void loop()
{
  data.j1PotY = map(analogRead(A0), 0, 1023, 0, 255);
  data.j1PotX = map(analogRead(A1), 0, 1023, 255, 0);
  data.j2PotX = map(analogRead(A2), 0, 1023, 0, 255);
  data.pot1 = map(analogRead(A3), 0, 1023, 255, 0);
    
  if (digitalRead(p1) == 0)
  {
    read_IMU();
  }
  
  radio.write(&data, sizeof(Data_Package));
}

void initialize_MPU6050()
{
  Wire.begin();                      
  Wire.beginTransmission(MPU);       
  Wire.write(0x6B);                  
  Wire.write(0x00);                  
  Wire.endTransmission(true);        

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  
  Wire.write(0x10);                  
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                  
  Wire.write(0x10);                  
  Wire.endTransmission(true);
}

void calculate_IMU_error()
{
  while (c < 200)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
   
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;

    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }

  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;

  while (c < 200)
  {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();

    GyroErrorX = GyroErrorX + (GyroX / 32.8);
    GyroErrorY = GyroErrorY + (GyroY / 32.8);
    c++;
  }

  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;

  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}

void read_IMU()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; 
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; 
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; 
  
  AccUholX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 1.46;
  AccUholY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 4.72;

  Predcas = Teraz;        
  Teraz = millis();            
  Prejdcas = (Teraz - Predcas) / 1000;   

  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); 

  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  
  GyroX = GyroX - 0.49; 
  GyroY = GyroY - 0.44;

  GyroUholX = GyroX * Prejdcas;
  GyroUholY = GyroY * Prejdcas;

  uholX = 0.98 * (uholX + GyroUholX) + 0.02 * AccUholX;
  uholY = 0.98 * (uholY + GyroUholY) + 0.02 * AccUholY;

  data.j1PotX = map(uholX, -90, +90, 0, 255);
  data.j1PotY = map(uholY, -90, +90, 0, 255);
}
