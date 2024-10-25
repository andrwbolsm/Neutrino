#include <Wire.h>

// Setar a sensitividade do acelerômetro
const int ACEL_SENS = 0x10; // 2g -> 0x00, 4g -> 0x08, 8g -> 0x10, 16g -> 0x18
const float fs_a = 4096.0; // fator de escala p aceleração: 2g -> 16374; 4g -> 8192; 8g -> 4096; 16g -> 2048

// Setar a sensibilidade do giroscópio
const int GIRO_SENS = 0x08; // 250 deg/s -> 0x00, 500 deg/s -> 0x08, 1000 deg/s -> 0x10, 2000 deg/s -> 0x18
const float fs_w = 65.5; // fator de escala p giro: 250 deg/s -> 131; 500 deg/s -> 65.5; 1000 deg/s -> 32.8; 2000 deg/s -> 16.4

struct {
  double aX, aY, aZ, gX, gY, gZ, Temp;
}MPUdata;

void setup() {
  Wire.begin(); // Inicializa a biblioteca Wire.h
  Serial.begin(115200);

  MPUconfig();
}

void loop() {
  read_MPU6050();

  Serial.println(MPUdata.aX);
}

void MPUconfig(){
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

  // Configura MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(GIRO_SENS);
  Wire.endTransmission();

  // Configura o filtro passa-baixa digital
  // Só precisa mudar os três últimos digitos:

  /*0b...... signifies a binary representation of a number
  0x...... signifies a hexadecimal representation of a number
  0...... signifies an octal representation of a number (base 8 )
  valid decimal digits not beginning with zero = decimal number
  */

  byte DLPF_CFG = 0b00000110; 
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
  int16_t acc_y = Wire.read() << 8 | Wire.read();
  int16_t acc_z = Wire.read() << 8 | Wire.read();

  // Lê 2 bytes da temperatura
  int16_t temperature = Wire.read() <<8 | Wire.read();

  // Lê 6 bytes do giro
  int16_t gyro_x = Wire.read()<<8 | Wire.read();
  int16_t gyro_y = Wire.read()<<8 | Wire.read();
  int16_t gyro_z = Wire.read()<<8 | Wire.read();

  // Neste momento, é necessário fazer a conversão correta dos dados crus lidos.

  // Conversão da aceleração: a_correto = a_cru / fator_de_escala (fs_a)
  MPUdata.aX = acc_x/fs_a;
  MPUdata.aY = acc_y/fs_a;
  MPUdata.aZ = acc_z/fs_a;

  // Conversão da temperatura (°C): T_correto = T_cru/340.0 + 36.53
  MPUdata.Temp = (temperature/340.0) + 36.53;

  // Conversão do giro: w_correto = w_cru / fator_de_escala (fs_w)
  MPUdata.gX = gyro_x/fs_w;
  MPUdata.gY = gyro_y/fs_w;
  MPUdata.gZ = gyro_z/fs_w;
}
