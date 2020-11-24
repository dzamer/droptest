#include <HX711_ADC.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

HX711_ADC LoadCell(4, 5);

#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
#define LIS3DH_CS 10

Adafruit_LIS3DH accelerometer = Adafruit_LIS3DH();


boolean load_sensor_calibration_status = false;
float calibration_scale_factor = 0.0;
int reflection_sensor_connection_lower = 2;
int reflection_sensor_connection_upper = 3;
int reflection_sensor_out_lower;
int reflection_sensor_out_upper;
float load_sensor_out;
float calValue;
float knownMass;
long t;
long myTime;
int switch_value;
const int eepromAdress = 0;
boolean waiting = 0;

void Load_Sensor_Calibration() {
  Serial.println("***");
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  } else {
    LoadCell.setCalFactor(1.0);
    Serial.println("Load sensor tare complete");
  }
  delay(100);
  Serial.println("Start calibration:");

  boolean f = 0;
  Serial.print("Enter calibration mass in kg, use ''.'' as separator: ");
  while (f == 0) {
    LoadCell.update();
    if (Serial.available() > 0) {
      knownMass = Serial.parseFloat();
      if (knownMass > 0) {
        Serial.print("Known mass is: ");
        Serial.println(knownMass);
        f = 1;
      }
    }
  }
  calibration_scale_factor = LoadCell.getData() / knownMass;
  LoadCell.setCalFactor(calibration_scale_factor);
  Serial.print("your scale factor is: ");
  Serial.println(calibration_scale_factor);
  LoadCell.update();
  f = 0;
  Serial.print("value saved to EEPROM adress ");
  Serial.print(eepromAdress);
  while (f == 0) {
#if defined(ESP8266)
    EEPROM.begin(512);
#endif
    EEPROM.put(eepromAdress, calibration_scale_factor);
#if defined(ESP8266)
    EEPROM.commit();
#endif
    EEPROM.get(eepromAdress, calibration_scale_factor);
    Serial.print("Value ");
    Serial.print(calibration_scale_factor);
    Serial.print(" saved to EEPROM address: ");
    Serial.println(eepromAdress);
    f = 1;
  }
  waiting = 1;
}

void Load_Sensor_Run () {
  float calValue; // calibration value
  calValue = 2000.0; // uncomment this if you want to set this value in the sketch
#if defined(ESP8266)
  EEPROM.begin(512);
#endif
  EEPROM.get(eepromAdress, calValue);

  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Tare timeout, check MCU>HX711 wiring and pin designations");
  }
  else {
    LoadCell.setCalFactor(calValue); // set calibration value (float)
    Serial.println("Startup + tare is complete");
  }
  waiting = 1;
}

void setup() {

  Serial.begin(9600);
  delay(100);
  //  pinMode(2, INPUT);
  LoadCell.begin();
  long stabilisingtime = 2000;
  LoadCell.start(stabilisingtime);

  Serial.println("Choose load sensor option: ");
  waiting = 0;
  while (!waiting) {
    int load_sensor_choosed = Serial.parseInt();
    if (load_sensor_choosed != 0) {
      switch (load_sensor_choosed) {
        case 1:
          Load_Sensor_Calibration();
          break;
        case 2:
          Load_Sensor_Run();
          break;
      }
    }
  }
  delay(100);
  Serial.println("Load sensor prepaired.");
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
      char entranceKey = Serial.read();
      if (entranceKey == 'e') {
        Serial.println("Droptest started..");
        entrance = 1;
        myTime = millis() + 3000;

      }
    }
  }
}


void loop() {

  LoadCell.update();

  while (myTime > millis()) {
    if (millis() > 250) {
      reflection_sensor_out_lower = digitalRead(reflection_sensor_connection_lower);
      reflection_sensor_out_upper = digitalRead(reflection_sensor_connection_upper);
      float mass = LoadCell.getData();
      Serial.print(millis()); // first print time value
      Serial.print(",");
      Serial.print(mass);  // second print load sensor value
      Serial.print(",");
      Serial.print(reflection_sensor_out_upper); // 3rd print upper slotted reflection sensor
      Serial.print(",");
      Serial.print(reflection_sensor_out_lower); // 4th print lower slotted reflection sensor
      Serial.print(",");
      accelerometer.read(); // read data from all 3 axies
      Serial.print(accelerometer.x); // read the axis-X
      Serial.print(",");
      Serial.print(accelerometer.y); // read the axis-Y
      Serial.print(",");
      Serial.print(accelerometer.z); // read the axis-Z
      sensors_event_t event;
      accelerometer.getEvent(&event);
      Serial.print(",");
      Serial.print(event.acceleration.x);
      Serial.print(",");
      Serial.print(event.acceleration.y);
      Serial.print(",");
      Serial.print(event.acceleration.z);
      Serial.println(";");
    }
  }

  delay(200);
}
