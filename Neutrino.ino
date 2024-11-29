#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <BasicLinearAlgebra.h>
#include <EEPROM.h>
#include <Servo.h>

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor(); 

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

const float Ai = -0.0065, R0 = 287.0530, g0 = 9.80665;
int i = 0;
float AltitudeHistory[2], AltitudeStartup = 0, PressureStartup = 1013.25, TemperatureStartup = 25;

// Setar a sensitividade do acelerômetro
const int ACEL_SENS = 0x08; // 2g -> 0x00, 4g -> 0x08, 8g -> 0x10, 16g -> 0x18
const float fs_a = 8192.0; // fator de escala p aceleração: 2g -> 16374; 4g -> 8192; 8g -> 4096; 16g -> 2048

// Setar a sensibilidade do giroscópio
// const int GIRO_SENS = 0x08; // 250 deg/s -> 0x00, 500 deg/s -> 0x08, 1000 deg/s -> 0x10, 2000 deg/s -> 0x18
// const float fs_w = 65.5; // fator de escala p giro: 250 deg/s -> 131; 500 deg/s -> 65.5; 1000 deg/s -> 32.8; 2000 deg/s -> 16.4

unsigned long prevTime = 0; // Tempo para o loop principal
unsigned long sampleTime = 0; // Tempo para o loop de 51 Hz

const int frequency_filter = 512, frequency_sample_save = 51;

const unsigned long microT = 1000000 / frequency_filter; 
const unsigned long sampleInterval = 1000000 / frequency_sample_save; // Intervalo para 51 Hz
const float deltaT = microT / 1000000.0;  // Convertido para segundos

struct {
  float Temp, Pressure, Altitude;
}BMPData;

float accel;

struct{
  const int Pin = 9, frequency = 10;
  unsigned long lastToggleTime = 0, period = 1000000 / frequency;
  bool State = LOW; 
}Buzzer;

bool Apogee, savedOnEEPROM = false;
byte RingBuffer[512]; // 512 é o tamanho da EEPROM do Arduino Nano
int cnt = 1;
int R1 = 620; //ohm
int R2 = 470; //ohm
float battery_level = 0.0;

Servo servo;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  servo.attach(3);
  servo.write(0);

  readFromEEPROM(sizeof(RingBuffer));
  delay(1000);

  BMP280Config();
  MPUConfig();
  pinMode(Buzzer.Pin, OUTPUT);
  battery_level = analogRead(A6)*(5.0/1023.0)*(R1+R2)/R2;
  RingBuffer[0] = floatToByte(battery_level, 12.0);

  F = {1, deltaT,
       0, 1};
  G = {0.5 * deltaT * deltaT,
       deltaT};
  H = {1, 0};
  I = {1, 0,
       0, 1};
  Q = G * ~G * 100 * 100; //  (matriz de covariância do processo)
  R = {50 * 50}; // (matriz de covariância da medição)
  P = {0, 0,
       0, 0};
  S = {0,
       0};

  digitalWrite(Buzzer.Pin, HIGH);
  delay(1000);
  digitalWrite(Buzzer.Pin,LOW);
}

void loop() {
  unsigned long currentTime = micros();

  if (currentTime - prevTime >= microT) {
    prevTime = currentTime;
    read_BMP280();
    read_MPU6050();
    kalman_filter();

    if(!Apogee){
      Apogee = ApogeeDetect();
    }
    else{
      BuzzerToggle(currentTime);
    }
    // printData();
  }

  if (currentTime - sampleTime >= sampleInterval && cnt < 511) {
    sampleTime = currentTime;
    RingBuffer[cnt] = floatToByte(AltitudeKalman, 110.0);
    Serial.println("OK");


    cnt += 1;

    if(!Apogee && cnt == 200){
      cnt = 1;
    }
  }

  if(cnt >= 511 && !savedOnEEPROM){
    saveToEEPROM(RingBuffer, sizeof(RingBuffer));
    savedOnEEPROM = true;
  }

  if(savedOnEEPROM){
    star_wars_theme();
  }
}

void printData(){
  Serial.print("Accel. X: ");
  Serial.print(accel);
  Serial.print(" || Altitude: ");
  Serial.print(BMPData.Altitude);
  Serial.print(" || Altitude Kalman: ");
  Serial.print(AltitudeKalman);
  Serial.print(" || Vertical Velocity: ");
  Serial.print(VelocityVerticalKalman);
  Serial.print(" || Battery: ");
  Serial.println(battery_level);
}

void kalman_filter(void){
  Acc = accel;
  S = F * S + G * Acc;
  P = F * P * ~F + Q;
  L = H * P * ~H + R;
  K = P * ~H * Invert(L);
  M = {BMPData.Altitude};
  S = S + K * (M - H * S);
  AltitudeKalman = S(0, 0);
  VelocityVerticalKalman = S(1, 0);
  P = (I - K * H) * P;
}

bool ApogeeDetect(){
  if (AltitudeKalman > 1.0 && VelocityVerticalKalman < -0.5){
      servo.write(180);
    return true;
  }
  else{
    return false;
  }
}

void BuzzerToggle(unsigned long currentTime){
  if (currentTime - Buzzer.lastToggleTime >= Buzzer.period / 2) { 
    Buzzer.lastToggleTime = currentTime;
    Buzzer.State = !Buzzer.State;
    digitalWrite(Buzzer.Pin, Buzzer.State);
  }
}

void BMP280Config(){
  // Configura o BMP280 e seta as variáveis iniciais
  bmp.begin(0x76);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_63); /* Standby time. */

  while(i<2){
    delay(1000);
    read_BMP280();
    AltitudeHistory[i] = BMPData.Altitude;

    if (i>0 && AltitudeHistory[i] - AltitudeHistory[i-1] <= 0.01){
      i = 2;
    }
    else{
      i++;
      if (i>1){
        i = 0;
      }
    }
  }

  read_BMP280();
  AltitudeStartup = BMPData.Altitude;
  PressureStartup = BMPData.Pressure;
  TemperatureStartup = BMPData.Temp;
}

void read_BMP280() {
  // Lê as medições de pressão e temperatura, e aplica a estimativa da altitude utilizando o modelo ISA
  sensors_event_t temp_event, pressure_event;

  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  BMPData.Temp = temp_event.temperature;
  BMPData.Pressure = pressure_event.pressure;
  BMPData.Altitude = ((TemperatureStartup + 273.15)/Ai)*(pow((BMPData.Pressure/PressureStartup), -(Ai*R0/g0)) - 1);
}

void MPUConfig(){
    // Inicializa e configura o MPU6050
  Wire.beginTransmission(0x68); //Endereço do dispositivo
  Wire.write(0x6B); //Registro ao qual será enviado a mensagem
  Wire.write(0x00); //Valor a ser escrito para acordar o MPU6050
  Wire.endTransmission(); 

  // Configura acelerômetro
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(ACEL_SENS);
  Wire.endTransmission();

  // Configura o filtro passa-baixa digital
  // Só precisa mudar os três últimos digitos:

  /*0b...... signifies a binary representation of a number
  0x...... signifies a hexadecimal representation of a number
  0...... signifies an octal representation of a number (base 8 )
  valid decimal digits not beginning with zero = decimal number
  */

  byte DLPF_CFG = 0b00000000; 
  /* TABELA PARA DLPF_CFG
            |   ACCELEROMETER    |           GYROSCOPE
   DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
   ---------+-----------+--------+-----------+--------+-------------
   000      | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
   001      | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
   010      | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
   011      | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
   100      | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
   101      | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
   110      | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
   111      |   -- Reserved --   |   -- Reserved --   | Reserved
  */

  /* OBs: Se usar DLPF_CFG > 0, o sample rate do giroscópio sera 1khz ao inves de 8khz porém, isso não alterará nada no sample rate 
    do acelerômetro que continua sendo 1khz a única mudança de aumentar DLPF_CFG é que teremos dados mais suaves com algumas medidas
   de aceleração se repetindo para manter 1khz de amostragem
  */

  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(DLPF_CFG);
  Wire.endTransmission();
  
  byte PWR_MGMT_2 = 0b00011111; // Coloca em standby todos os eixos exceto medições no eixo X
  Wire.beginTransmission(0x68);
  Wire.write(0x6C);
  Wire.write(PWR_MGMT_2);
  Wire.endTransmission();
}

void read_MPU6050(){
  //Lê os dados de aceleração, giro e temperatura do MPU-6050

  // Seta o registrador no qual queremos ler
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  // Requisita 14 bytes do MPU6050
  Wire.requestFrom(0x68, 14);

  // Lê 6 bytes da aceleração
  int16_t acc_x = (Wire.read() << 8 | Wire.read());

  // Neste momento, é necessário fazer a conversão correta dos dados crus lidos.

  // Conversão da aceleração: a_correto = a_cru / fator_de_escala (fs_a)
  float mpu_g = (acc_x/fs_a - 0.34)*0.97;
  accel = (mpu_g - mpu_g/abs(mpu_g))*g0;
}

float byteToFloat(uint8_t byteValue, float max) {
  // Converte o byte de volta para o valor no intervalo [0, max]
  return (float)byteValue * max / 255.0;  // Mapeia de [0, 255] para [0, max]
}

uint8_t floatToByte(float value, float max) {
  // Função para mapear um valor float (0 a max) para um uint8_t (0 a 255)
  // Constrói o valor no intervalo [0, max] para o intervalo [0, 255]
  value = constrain(value, 0.0, max);  // Garante que o valor está dentro do intervalo permitido
  return (uint8_t)((value / max) * 255);  // Mapeia para [0, 255]
}

void saveToEEPROM(const byte* data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    EEPROM.update(i, data[i]);
  }
}

void readFromEEPROM(size_t size) {
  // Lê e imprime os dados da EEPROM
  Serial.println(byteToFloat(EEPROM.read(0),12.0));
  for (size_t i = 1; i < size-1; i++) {
    Serial.println(byteToFloat(EEPROM.read(i), 150.0));
  }
}

void star_wars_theme(){
  //Essa função serve para tocar o tema do Star Wars
  //Quando essa música toca, significa que o sistema está pronto para ser desligado e plugado em um
  //computador para leitura

  // change this to make the song slower or faster
  int tempo = 108;

  // notes of the moledy followed by the duration.
  // a 4 means a quarter note, 8 an eighteenth , 16 sixteenth, so on
  // !!negative numbers are used to represent dotted notes,
  // so -4 means a dotted quarter note, that is, a quarter plus an eighteenth!!
  int melody[] = {
      // Dart Vader theme (Imperial March) - Star wars
      // Substituindo as macros pelas frequências diretamente
      
      466, 8, 466, 8, 466, 8,  // NOTE_AS4
      698, 2, 1047, 2,         // NOTE_F5, NOTE_C6
      932, 8, 880, 8, 784, 8, 1568, 2, 1047, 4,  // NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6
      932, 8, 880, 8, 784, 8, 1568, 2, 1047, 4,  // NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6
      932, 8, 880, 8, 932, 8, 784, 2, 523, 8, 523, 8, 523, 8,  // NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5
      698, 2, 1047, 2,         // NOTE_F5, NOTE_C6
      932, 8, 880, 8, 784, 8, 1568, 2, 1047, 4,  // NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6
      
      932, 8, 880, 8, 784, 8, 1568, 2, 1047, 4,  // NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6
      932, 8, 880, 8, 932, 8, 784, 2, 523, -8, 523, 16,  // NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5
      587, -4, 587, 8, 932, 8, 880, 8, 784, 8, 698, 8,  // NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5
      698, 8, 784, 8, 880, 8, 784, 4, 587, 8, 659, 4, 523, -8, 523, 16,  // NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C5
      587, -4, 587, 8, 932, 8, 880, 8, 784, 8, 698, 8,  // NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5
      
      1047, -8, 784, 16, 784, 2, 0, 8, 523, 8,  // NOTE_C6, NOTE_G5, REST, NOTE_C5
      587, -4, 587, 8, 932, 8, 880, 8, 784, 8, 698, 8,  // NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5
      698, 8, 784, 8, 880, 8, 784, 4, 587, 8, 659, 4, 1047, -8, 1047, 16,  // NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C6
      1568, 4, 1245, 8, 1109, 4, 1047, 8, 932, 4, 831, 8, 784, 4, 698, 8,  // NOTE_F6, NOTE_DS6, NOTE_CS6, NOTE_C6, NOTE_AS5, NOTE_GS5, NOTE_G5, NOTE_F5
      1047, 1  // NOTE_C6
  };


  // sizeof gives the number of bytes, each int value is composed of two bytes (16 bits)
  // there are two values per note (pitch and duration), so for each note there are four bytes
  int notes = sizeof(melody) / sizeof(melody[0]) / 2;

  // this calculates the duration of a whole note in ms
  int wholenote = (60000 * 4) / tempo;

  int divider = 0, noteDuration = 0;

    // iterate over the notes of the melody. 
    // Remember, the array is twice the number of notes (notes + durations)
    for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

        // calculates the duration of each note
        divider = melody[thisNote + 1];
        if (divider > 0) {
          // regular note, just proceed
          noteDuration = (wholenote) / divider;
        } else if (divider < 0) {
          // dotted notes are represented with negative durations!!
          noteDuration = (wholenote) / abs(divider);
          noteDuration *= 1.5; // increases the duration in half for dotted notes
        }

        // we only play the note for 90% of the duration, leaving 10% as a pause
        tone(Buzzer.Pin, melody[thisNote], noteDuration*0.9);

        // Wait for the specief duration before playing the next note.
        delay(noteDuration);
        
        // stop the waveform generation before the next note.
        noTone(Buzzer.Pin);

        if (thisNote >= (notes * 2 - 2)) {
            thisNote = -2; // Voltar para o início
      }
    }
}