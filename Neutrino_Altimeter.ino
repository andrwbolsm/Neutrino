/*
  At Doom's Gate (H1M1)
  Connect a piezo buzzer or speaker to pin 11 or select a new pin.
  More songs available at https://github.com/robsoncouto/arduino-songs

                                              Robson Couto, 2019
*/

#define NOTE_AS4 466
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_DS6 1245
#define NOTE_F6 1397
#define REST 0

/*
The contents of this code and instructions are the intellectual property of Carbon Aeronautics.
The text and figures in this code and instructions are licensed under a Creative Commons Attribution - Noncommercial - ShareAlike 4.0 International Public Licence.
This license lets you remix, adapt, and build upon your work non-commercially, as long as you credit Carbon Aeronautics
(but not in any way that suggests that we endorse you or your use of the work) and license your new creations under the identical terms.
This code and instruction is provided "As Is” without any further warranty. Neither Carbon Aeronautics or the author has any liability to any person or entity
with respect to any loss or damage caused or declared to be caused directly or indirectly by the instructions contained in this code or by
the software and hardware described in it. As Carbon Aeronautics has no control over the use, setup, assembly, modification or misuse of the hardware,
software and information described in this manual, no liability shall be assumed nor accepted for any resulting damage or injury.
By the act of copying, use, setup or assembly, the user accepts all resulting liability.

1.0  18 February 2023 -  initial release
*/
#include <EEPROM.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Servo.h>
float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
float LoopTimer;
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t dig_P6, dig_P7, dig_P8, dig_P9;
float AltitudeBarometer, AltitudeBarometerStartUp;
int RateCalibrationNumber;
using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2, 2> F;
BLA::Matrix<2, 1> G;
BLA::Matrix<2, 2> P;
BLA::Matrix<2, 2> Q;
BLA::Matrix<2, 1> S;
BLA::Matrix<1, 2> H;
BLA::Matrix<2, 2> I;
BLA::Matrix<1, 1> Acc;
BLA::Matrix<2, 1> K;
BLA::Matrix<1, 1> R;
BLA::Matrix<1, 1> L;
BLA::Matrix<1, 1> M;

Servo meuServo;
int count = 1, count2 = 1;
int tempo = 108;
int buzzer = 9;
float ultimaAlturaSalva = 0; // Variável para armazenar a última altura salva
int enderecoEEPROM = 0;      // Endereço de início da EEPROM para gravação
float deltaAltura = 1.0;     // Diferença de 1 metro para começar a gravar

uint8_t altiByte[501];        // altitude in a byte
unsigned long nextSampleTime; // time when next sample has to be taken
int ringBufferPosition = 0;
boolean apogee = false;
int apogeeCount = 0; // number of samples after apogee;
int EEaddr = -1;
float altitude = 0;
float maxAltitude = 0;

const int melody[] PROGMEM = {

    NOTE_AS4, 8, NOTE_AS4, 8, NOTE_AS4, 8, // 1
    NOTE_F5, 2, NOTE_C6, 2,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8, NOTE_G5, 2, NOTE_C5, 8, NOTE_C5, 8, NOTE_C5, 8,
    NOTE_F5, 2, NOTE_C6, 2,
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4,

    NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F6, 2, NOTE_C6, 4, // 8
    NOTE_AS5, 8, NOTE_A5, 8, NOTE_AS5, 8, NOTE_G5, 2, NOTE_C5, -8, NOTE_C5, 16,
    NOTE_D5, -4, NOTE_D5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F5, 8,
    NOTE_F5, 8, NOTE_G5, 8, NOTE_A5, 8, NOTE_G5, 4, NOTE_D5, 8, NOTE_E5, 4, NOTE_C5, -8, NOTE_C5, 16,
    NOTE_D5, -4, NOTE_D5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F5, 8,

    NOTE_C6, -8, NOTE_G5, 16, NOTE_G5, 2, REST, 8, NOTE_C5, 8, // 13
    NOTE_D5, -4, NOTE_D5, 8, NOTE_AS5, 8, NOTE_A5, 8, NOTE_G5, 8, NOTE_F5, 8,
    NOTE_F5, 8, NOTE_G5, 8, NOTE_A5, 8, NOTE_G5, 4, NOTE_D5, 8, NOTE_E5, 4, NOTE_C6, -8, NOTE_C6, 16,
    NOTE_F6, 4, NOTE_DS6, 8, NOTE_CS6, 4, NOTE_C6, 8, NOTE_AS5, 4, NOTE_GS5, 8, NOTE_G5, 4, NOTE_F5, 8,
    NOTE_C6, 1};

// Quantidade de elementos no array da melodia
int notes = sizeof(melody) / sizeof(melody[0]);

// Função para tocar a melodia
void tocaMusica()
{
  int wholenote = (60000 * 4) / tempo; // cálculo da duração de uma nota inteira (em milissegundos)
  int divider = 0, noteDuration = 0;

  // Toca cada nota
  for (int thisNote = 0; thisNote < notes; thisNote += 2)
  {
    int note = pgm_read_word_near(melody + thisNote);
    int duration = pgm_read_word_near(melody + thisNote + 1);

    // Calcula a duração da nota
    if (duration > 0)
    {
      // Duração normal (por exemplo, 8 = semínima)
      divider = duration;
    }
    else
    {
      // Nota pontuada (por exemplo, -4 = mínima pontuada)
      divider = abs(duration);
      noteDuration = (wholenote * 1.5) / divider;
    }
    noteDuration = wholenote / divider;

    // Toca a nota no buzzer
    if (note != REST)
    {
      tone(buzzer, note, noteDuration * 0.9);
    }

    // Pausa entre notas
    delay(noteDuration);

    // Silêncio para a próxima nota
    noTone(buzzer);
  }
}

void gravaNaEEPROM(float altura)
{
  // recording has ended - save values in EEPROM
  digitalWrite(13, HIGH);
  byte value = EEPROM.read(0);
  if (value >= 199)
  {
    value = 0;
  }
  value++;
  EEPROM.write(0, value); // write number of recording in address 0

  int IVin = analogRead(A6); // read battery voltage
  IVin = IVin / 4;
  uint8_t Vwrite = IVin;
  if (Vwrite == 255)
  {
    Vwrite = 254;
  }
  EEPROM.write(1, Vwrite); // write battery voltage in address 1

  for (int i = 2; i < 502; i++)
  { // write 500 samples in addresses 2...501
    ringBufferPosition++;
    if (ringBufferPosition == 500)
    {
      ringBufferPosition = 0;
    }
    delay(1);
    uint8_t writeVal = altiByte[ringBufferPosition];
    if (writeVal == 255)
    {
      writeVal = 254;
    }
    EEPROM.write(i, writeVal);

    tocaMusica();
  }

  while (1)
  {
    for (int i = 0; i < 10; i++)
    {
      digitalWrite(13, HIGH);
      delay(50);
      digitalWrite(13, LOW);
      delay(50);
    }
    delay(1000);
  }
}

void leDadosEEPROM()
{
  int address = 0;
  byte value;

  while (address <= EEPROM.length())
  {
    // read a byte from the current address of the EEPROM
    value = EEPROM.read(address);

    value = value / 2 - 10;

    Serial.print(address);
    Serial.print("\t");
    Serial.print(value, DEC);
    Serial.println();
    address = address + 1;
  }
}

void kalman_2d(void)
{
  Acc = {AccZInertial};
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Invert(L);
  M = {AltitudeBarometer};
  S = S + K * (M - H * S);
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  P = (I - K * H) * P;
}

void barometer_signals(void)
{
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76, 6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int)dig_T1 << 1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T >> 4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int)t_fine) >> 1) - (signed long int)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((signed long int)dig_P6);
  var2 = var2 + ((var1 * ((signed long int)dig_P5)) << 1);
  var2 = (var2 >> 2) + (((signed long int)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((signed long int)dig_P2) * var1) >> 1)) >> 18;
  var1 = ((((32768 + var1)) * ((signed long int)dig_P1)) >> 15);
  if (var1 == 0)
  {
    p = 0;
  }
  p = (((unsigned long int)(((signed long int)1048576) - adc_P) - (var2 >> 12))) * 3125;
  if (p < 0x80000000)
  {
    p = (p << 1) / ((unsigned long int)var1);
  }
  else
  {
    p = (p / (unsigned long int)var1) * 2;
  }
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
  var2 = (((signed long int)(p >> 2)) * ((signed long int)dig_P8)) >> 13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4));
  double pressure = (double)p / 100;
  AltitudeBarometer = 44330 * (1 - pow(pressure / 1030, 1 / 5.255)) * 100;
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096 - 0.07;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096 + 0.07;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

void blink1Hz()
{
  unsigned long tm = millis();
  tm = tm / 500;
  digitalWrite(13, tm % 2);

  // colocar musica 1
  if (tm % 2 == 0)
  {
    tone(buzzer, NOTE_AS4, 500); // toca por 500ms
  }
  else
  {
    noTone(buzzer); // desliga o buzzer nos outros 500ms
  }
}

void blink10Hz()
{
  unsigned long tm = millis();
  tm = tm / 50;
  digitalWrite(13, tm % 2);

  // musica 2
  if (tm % 2 == 0)
  {
    tone(buzzer, NOTE_AS4, 500); // toca por 50ms
  }
  else
  {
    noTone(buzzer); // desliga o buzzer nos outros 500ms
  }
}

void setup()
{
  Serial.begin(57600);
  meuServo.attach(3);
  pinMode(13, OUTPUT);
  Wire.setClock(400000);
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  tone(buzzer, NOTE_AS4, 250);
  delay(250);
  noTone(buzzer);
  delay(1000);
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();
  Wire.beginTransmission(0x76);
  Wire.write(0xF5);
  Wire.write(0x14);
  Wire.endTransmission();
  uint8_t data[24], i = 0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  tone(buzzer, NOTE_AS4, 250);
  delay(250);
  noTone(buzzer);
  delay(1000);
  Wire.requestFrom(0x76, 24);
  while (Wire.available())
  {
    data[i] = Wire.read();
    i++;
  }

  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
  delay(250);

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++)
  {
    barometer_signals();
    AltitudeBarometerStartUp += AltitudeBarometer;
    delay(1);
  }

  AltitudeBarometerStartUp /= 2000;
  F = {1, 0.004,
       0, 1};
  G = {0.5 * 0.004 * 0.004,
       0.004};
  H = {1, 0};
  I = {1, 0,
       0, 1};
  Q = G * ~G * 10 * 10;
  R = {30 * 30};
  P = {0, 0,
       0, 0};
  S = {0,
       0};
  LoopTimer = micros();

  digitalWrite(13, HIGH);
  // Inicializa última altura salva como a altitude inicial
  ultimaAlturaSalva = AltitudeKalman;

  // Chama a função para ler e exibir os dados da EEPROM
  leDadosEEPROM();
}

void loop()
{
  while (apogeeCount <= 300)
  { // after apogee: write still 300 samples into ringbuffer
    if (!apogee)
    {
      blink1Hz(); // blink with 1Hz in order to indicate that device is armed but apogee has not been detected
    }
    else
    {
      blink10Hz(); // blink with 10Hz in order to indicate that device is armed and apogee has been detected
    }

    if (millis() >= nextSampleTime)
    {
      gyro_signals();
      AccZInertial = -sin(AnglePitch * (3.142 / 180)) * AccX + cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY + cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
      AccZInertial = (AccZInertial - 1) * 9.81 * 100;

      barometer_signals();
      AltitudeBarometer -= AltitudeBarometerStartUp;

      kalman_2d();
      altitude = AltitudeKalman / 100;

      Serial.print("Altitude [m]:");
      Serial.print(altitude);
      Serial.print(" Vertical velocity [m/s]:");
      Serial.println(VelocityVerticalKalman / 100);

      nextSampleTime = nextSampleTime + 50; // take altitude samples with 20Hz

      // detect apogee
      if (altitude > maxAltitude)
      {
        maxAltitude = altitude; // save highest altitude
      }

      if (abs(maxAltitude - altitude) > 1.5 || VelocityVerticalKalman / 100 < 0)
      { // colocar velocidade pra redundancia
        apogee = true;
      }

      if (apogee)
      {
        apogeeCount++;       // keep track of how many values have been sampled since apogee
        meuServo.write(180); // aciona o servo motor
      }

      // convert altitude in a byte value
      altitude = altitude + 10;
      altitude = altitude * 2;
      if (altitude < 0)
      {
        altitude = 0;
      }
      if (altitude > 254)
      {
        altitude = 254;
      }
      altiByte[ringBufferPosition] = int(altitude); // write altitude samples into ringbuffer
      ringBufferPosition++;
      if (ringBufferPosition == 500)
      {
        ringBufferPosition = 0;
      }
    }
  }
  gravaNaEEPROM(altitude);
}
