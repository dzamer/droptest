#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

Pytałem o przycisk, na piny ustawiasz przycis i tranzystor i piszesz że jak tranzysotr ma stan niski to coś, a jak stan wysoki to coś. Koniec

#define LIS3DH_CLK 13
#define LIS3DH_MOSI 12
#define LIS3DH_MISO 11
#define LIS3DH_CS 10

Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

int reflection_sensor_connection_lower = 2;
int reflection_sensor_connection_upper = 3;
int reflection_sensor_out_lower;
int reflection_sensor_out_upper;
int switch_value;
boolean waiting = 0;


void setup() {

  Serial.begin(2000000);
  delay(100);
  Serial.println("Starting...");
  if (! accelerometer.begin(0x18)) {
    Serial.println("Couldnt start");
    while (1) yield();
  }
  Serial.println("LIS3DH found!");

  delay(100);
  waiting = 0;
  int accelerometer_setRange;
  Serial.println("Choose accelerometer range: ");
  while (!waiting) {
    accelerometer_setRange = Serial.parseInt();
    if (accelerometer_setRange != 0) {
      switch (accelerometer_setRange) {
        case 12:
          accelerometer.setRange(LIS3DH_RANGE_2_G);
          Serial.println("2G choosed");
          waiting = 1;
          break;
        case 13:
          accelerometer.setRange(LIS3DH_RANGE_4_G);
          Serial.println("4G choosed");
          waiting = 1;
          break;
        case 14:
          accelerometer.setRange(LIS3DH_RANGE_8_G);
          Serial.println("8G choosed");
          waiting = 1;
          break;
        case 15:
          accelerometer.setRange(LIS3DH_RANGE_16_G);
          Serial.println("16G choosed");
          waiting = 1;
          break;
      }
    }
  }
  delay (100);

  Serial.println("Choose your data rate: ");
  waiting = 0;
  while (!waiting) {
    int dataRate = Serial.parseInt();
    if (dataRate != 0) {
      switch (dataRate) {
        case 21:
          accelerometer.setDataRate(LIS3DH_DATARATE_1_HZ);
          Serial.println("1 Hz");
          waiting = 1;
          break;
        case 22:
          accelerometer.setDataRate(LIS3DH_DATARATE_10_HZ);
          Serial.println("10 Hz");
          waiting = 1;
          break;
        case 23:
          accelerometer.setDataRate(LIS3DH_DATARATE_25_HZ);
          Serial.println("25 Hz");
          waiting = 1;
          break;
        case 24:
          accelerometer.setDataRate(LIS3DH_DATARATE_50_HZ);
          Serial.println("50 Hz");
          waiting = 1;
          break;
        case 25:
          accelerometer.setDataRate(LIS3DH_DATARATE_100_HZ);
          Serial.println("100 Hz");
          waiting = 1;
          break;
        case 26:
          accelerometer.setDataRate(LIS3DH_DATARATE_200_HZ);
          Serial.println("200 Hz");
          waiting = 1;
          break;
        case 27:
          accelerometer.setDataRate(LIS3DH_DATARATE_400_HZ);
          Serial.println("400 Hz");
          waiting = 1;
          break;
        case 28:
          accelerometer.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);
          Serial.println("5 Khz Low Power");
          waiting = 1;
          break;
        case 29:
          accelerometer.setDataRate(LIS3DH_DATARATE_LOWPOWER_1K6HZ);
          Serial.println("16 Khz Low Power");
          waiting = 1;
          break;
        case 30:
          accelerometer.setDataRate(LIS3DH_DATARATE_POWERDOWN);
          Serial.println("Powered Down");
          waiting = 1;
          break;
      }
    }
  }
  delay(100);

  Serial.println ("Press start button");
  boolean entrance = 0;
  while (entrance == 0) {
    if (Serial.available() > 0) {
      char entranceKey = Serial.parseInt();
      if (entranceKey == 41) {
        Serial.println("Droptest started..");
        entrance = 1;


      }
    }
  }
}


void loop() {

  reflection_sensor_out_lower = digitalRead(reflection_sensor_connection_lower);
  reflection_sensor_out_upper = digitalRead(reflection_sensor_connection_upper);
  Serial.print(millis()); // first print time value
  Serial.print(",");
  Serial.print(reflection_sensor_out_upper); // 2nd print upper slotted reflection sensor
  Serial.print(",");
  Serial.print(reflection_sensor_out_lower); // 3rd print lower slotted reflection sensor
  sensors_event_t event;
  accelerometer.getEvent(&event);
  Serial.print(",");
  Serial.print(event.acceleration.x); // 4th print, reading in m/s2
  Serial.print(",");
  Serial.print(event.acceleration.y); // 5th print, reading in m/s2
  Serial.print(",");
  Serial.print(event.acceleration.z); // 6th print, reading in m/s2
  Serial.println(";");
}
