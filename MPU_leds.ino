// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

//int leds[] = {19, 18, 5, 4};
int leds[] = {26, 25, 33, 32};

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G); //RANGOS DEL ACELERÓMETRO: 2G 4G 8G 16G
  
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);      //RANGOS DEL GIROSCOPO: 250DEG 500DEG 1000DEG 2000DEG


  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);    //FILTRO DE PRESICIÓN: 260HZ 184HZ 94HZ 44HZ 21HZ 10HZ 5HZ

  
  //LEDS BORRAR////////////////////////////////////////////////////////
    for (int i = 0; i < 4; i++) {
    pinMode(leds[i], OUTPUT);
  }
  ///////////////////////////////////////////////////////////////////

  Serial.println("");
  delay(100);

}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //////////////////////INCLINACIÓN EN EL EJE X y Y//////////////////////////////////////////////////
  float inc_y = atan(a.acceleration.y/sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.z,2)))*(180.0/PI) + 0.25;
  float inc_x = atan(a.acceleration.x/sqrt(pow(a.acceleration.y,2) + pow(a.acceleration.z,2)))*(180.0/PI);
  float inc_z = atan(a.acceleration.z/sqrt(pow(a.acceleration.x,2) + pow(a.acceleration.y,2)))*(180.0/PI);
  
  //Serial.print("Inclinación en el eje y: ");
  Serial.print(inc_y); Serial.print(",");
  //Serial.println(" °");
  //Serial.print("Inclinación en el eje x: ");
  Serial.print(inc_x); //Serial.print(",");
  //Serial.println(" °");
  //Serial.print("Inclinación en el eje z: ");
  //Serial.print(inc_z);
  //Serial.println(" °");


// Apagar todos los LEDs primero
  for (int i = 0; i < 4; i++) {
    digitalWrite(leds[i], LOW);
  }

  // Encender los LEDs correspondientes al rango de inclinación
  for (int i = 0; i < 4; i++) {
    float minAngulo = -90 + (i * 22.5);
    float maxAngulo = -90 + ((i - 1) * 22.5);
    if (inc_y >= minAngulo && inc_y < maxAngulo) {
      for (int j = i; j < 4; j++) {
        digitalWrite(leds[j], HIGH);
      }
      break; // Salir del bucle una vez que se hayan encendido los LEDs correspondientes
    }
  }


  float inclinacionX = atan2(-a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float inclinacionY = atan2(a.acceleration.x, a.acceleration.z) * 180 / PI;

  /*Serial.print("Inclinación en el eje y: ");
  Serial.print(inclinacionY);
  Serial.println(" °");
  Serial.print("Inclinación en el eje x: ");
  Serial.print(inclinacionX);
  Serial.println(" °");*/



  /* Print out the values */
  /*Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");*/

  /*Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");*/

  Serial.println("");
  delay(50);
}