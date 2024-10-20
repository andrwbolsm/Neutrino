#include <Wire.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor(); 

const float Ai = -0.0065, R = 287.0530, g0 = 9.81;
int i = 0;
float AltitudeHistory[2], AltitudeStartup = 0, PressureStartup = 1013.25, TemperatureStartup = 15;

struct {
  float Temp, Pressure, Altitude;
}BMPData;

void setup() {
  Serial.begin(115200);
  BMP280Config();
}

void loop() {
  BMP280Sampling();
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
    BMP280Sampling();
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

  AltitudeStartup = AltitudeHistory[1];
  PressureStartup = BMPData.Pressure;
  TemperatureStartup = BMPData.Temp;
}

void BMP280Sampling() {
  // Lê as medições de pressão e temperatura, e aplica a estimativa da altitude utilizando o modelo ISA
  sensors_event_t temp_event, pressure_event;

  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  BMPData.Temp = temp_event.temperature;
  BMPData.Pressure = pressure_event.pressure;
  BMPData.Altitude = ((TemperatureStartup + 273.15)/Ai)*(pow((BMPData.Pressure/PressureStartup), -(Ai*R/g0)) - 1);

  Serial.print(BMPData.Altitude);
  Serial.println(" m");
}
