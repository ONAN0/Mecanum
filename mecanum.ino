#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <AccelStepper.h>

RF24 radio(48,49);

const byte address[6] = "00001";
unsigned long CasPoslPrij = 0;
unsigned long CasTeraz = 0;

AccelStepper PravePredne(1, 32, 33);  
AccelStepper PraveZadne(1, 34, 35);   
AccelStepper LaveZadne(1, 36, 37);  
AccelStepper LavePredne(1, 38, 39); 

int rychlost = 1500;

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
  LavePredne.setMaxSpeed(3000);
  LaveZadne.setMaxSpeed(3000);
  PraveZadne.setMaxSpeed(3000);
  PravePredne.setMaxSpeed(3000);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate (RF24_250KBPS);
  radio.setPALevel (RF24_PA_LOW);
  radio.startListening(); //Nastavuje túto jednotku ako prijímač

  pinMode(14, OUTPUT);
  
  Serial.begin(115200);
}

void loop()
{
  CasTeraz = millis();
  if (CasTeraz - CasPoslPrij > 1000 )
  {
    resetData();
  }

  if (radio.available())
  {
    radio.read(&data, sizeof(Data_Package));
    CasPoslPrij = millis();
  }

  rychlost = map(data.pot1, 0, 255, 100, 3000);

  if (data.j1PotY > 140 & data.j1PotX == 129 )
  {
    vpred();
  }

  else if (data.j1PotY < 121 & data.j1PotX == 129 )
  {
    vzad();
  }
  
  else if (data.j1PotX > 140 & data.j1PotY == 129 )
  {
    vpravo();
  }

  else if (data.j1PotX < 121 & data.j1PotY == 129 )
  {
    vlavo();
  }

  else if (data.j1PotX < 121 & data.j1PotY > 140)
  {
    vlavoVpred();
  }

  else if (data.j1PotX > 140 & data.j1PotY > 140)
  {
    vpravoVpred();
  }

  else if (data.j1PotX < 121 & data.j1PotY < 121 )
  {
    vlavoVzad();
  }

  else if (data.j1PotX > 140 & data.j1PotY < 121 )
  {
    vpravoVzad();
  }

  else if (data.j2PotX < 121)
  {
    otocVlavo();
  }

  else if (data.j2PotX > 140)
  {
    otocVpravo();
  }

  else
  {
    stoj();
  }

  LavePredne.runSpeed();
  LaveZadne.runSpeed();
  PraveZadne.runSpeed();
  PravePredne.runSpeed();

  //Kontrola stavu batérie
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.00) * 3;

  //Ak klesňe napätie pod 11V rozsvieti sa Ledka
  if (voltage < 11)
  {
    digitalWrite(14, HIGH);
  }

  else
  {
    digitalWrite(14, LOW);
  }
}

void vpred() 
{
  LavePredne.setSpeed(-rychlost);
  LaveZadne.setSpeed(-rychlost);
  PraveZadne.setSpeed(rychlost);
  PravePredne.setSpeed(rychlost);
}

void vzad() 
{
  LavePredne.setSpeed(rychlost);
  LaveZadne.setSpeed(rychlost);
  PraveZadne.setSpeed(-rychlost);
  PravePredne.setSpeed(-rychlost);
}

void vpravo() 
{
  LavePredne.setSpeed(-rychlost);
  LaveZadne.setSpeed(rychlost);
  PraveZadne.setSpeed(rychlost);
  PravePredne.setSpeed(-rychlost);
}

void vlavo() 
{
  LavePredne.setSpeed(rychlost);
  LaveZadne.setSpeed(-rychlost);
  PraveZadne.setSpeed(-rychlost);
  PravePredne.setSpeed(rychlost);
}

void vpravoVpred() 
{
  LavePredne.setSpeed(-rychlost);
  LaveZadne.setSpeed(0);
  PraveZadne.setSpeed(rychlost);
  PravePredne.setSpeed(0);
}

void vpravoVzad() 
{
  LavePredne.setSpeed(0);
  LaveZadne.setSpeed(rychlost);
  PraveZadne.setSpeed(0);
  PravePredne.setSpeed(-rychlost);
}

void vlavoVpred() 
{
  LavePredne.setSpeed(0);
  LaveZadne.setSpeed(-rychlost);
  PraveZadne.setSpeed(0);
  PravePredne.setSpeed(rychlost);
}

void vlavoVzad() 
{
  LavePredne.setSpeed(rychlost);
  LaveZadne.setSpeed(0);
  PraveZadne.setSpeed(-rychlost);
  PravePredne.setSpeed(0);
}

void otocVlavo() 
{
  LavePredne.setSpeed(rychlost);
  LaveZadne.setSpeed(rychlost);
  PraveZadne.setSpeed(rychlost);
  PravePredne.setSpeed(rychlost);
}

void otocVpravo() 
{
  LavePredne.setSpeed(-rychlost);
  LaveZadne.setSpeed(-rychlost);
  PraveZadne.setSpeed(-rychlost);
  PravePredne.setSpeed(-rychlost);
}

void stoj() 
{
  LavePredne.setSpeed(0);
  LaveZadne.setSpeed(0);
  PraveZadne.setSpeed(0);
  PravePredne.setSpeed(0);
}

// Resetuje hodnoty ak nieje žiadne rádiové pripojenie
void resetData() 
{
  data.j1PotX = 129;
  data.j1PotY = 129;
  data.j2PotX = 129;
  data.pot1 = 1;
}
