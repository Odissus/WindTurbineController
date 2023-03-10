#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <SD.h>

#define current_measurement_pin 34
#define temperature_measurement_pin 35
#define rail_voltage_measurement_pin 33
#define current_correction_factor 1.0
#define ADC_steps 4095
#define Digital_Voltage_Reference 3.3
#define Celsius_Offset 273.15
#define V_Ref 3.3
#define temperature_correction_factor 1.0
#define max_low_reading 700
#define min_high_reading 1500
#define number_of_slots 24
#define tachometer_correction_factor 1.2
#define SD_channel_select_pin 5
#define voltage_tap_pin 35
#define amplifier_gain 3.84
#define resistance_ohm 100

/* WiFi network name and password */
const char * ssid = "TurbineMonitor";
const char * pwd = "0000000000";
const int udpPort = 44444;
WiFiUDP udp;

float rail_voltage = 0;
float voltage = 0;
float current = 0;
float temperature = 0;
float rpm = 0;
unsigned long old_time = 0;
unsigned long time_now = 0;
bool hole = true;
float photoresistor_reading = 0;
int holes = 0;
float oscilloscope_reading = 0;
bool SD_successful = false;
float temp_voltage = 0;
String data_to_save;

float get_voltage_reading(){
  return 1;
}

float get_current_reading(){
  // V=IR -> I = V/R
  //temp_voltage = (float) analogRead(current_measurement_pin) / 1240.9;
  //current = -9.9 * temp_voltage + 31.581;
  float this_current = 0;
  for (int i = 0; i < 10; i++){ 
    this_current += (float) analogRead(current_measurement_pin) / 10.0;
  }
  current = -0.005914 * this_current + 22.5623;
  current = 0.02506 * current * current + 0.69493 * current;
  if ((current>-0.2) and (current<0)){
    current = 0;
  }
  return current;
}

float get_temperature_reading(){
  float this_temperature = 0;
  const float R1 = 10000; // value of R1 on board in ohm
  float logR2, R2, Temperature, Therm_V;
  const float c1 = 0.001129148, c2 = 0.000234125, c3 = 0.0000000876741;
  for (int i = 0; i < 10; i++){ 
    this_temperature += (float) analogRead(temperature_measurement_pin) / 10.0;
  }
  Therm_V = this_temperature * V_Ref * temperature_correction_factor/ (float) ADC_steps;
  R2 = R1 * (Therm_V/(Digital_Voltage_Reference-Therm_V));
  logR2 = log(R2);
  Temperature = (1 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Temperature -= Celsius_Offset;
  return Temperature;
}

void update_rail_voltage_reading(){
  rail_voltage = 2 * (float) analogRead(rail_voltage_measurement_pin) / (float) ADC_steps;
}

void IRAM_ATTR hole_detected_interrupt()
{
  holes += 1;
}

void SaveData(String DataString, bool initial=false){
  // Save data to SD card
  File file = SD.open("/Data.csv", FILE_WRITE);
  if (!file){
    SD_successful = false;
    return;
  }
  if (initial == true){
    file.println("Time (ms), Voltage (V), Current (A), Angular velocity (rpm)");
    Serial.println("Data file initialised");
  }
  if (DataString != "") {
    file.println(DataString);
    Serial.println("Data added");
  }
  file.flush();
  file.close();
}

void setup(){
  Serial.begin(9600);
  
  //Connect to the WiFi network
  // WiFi.begin(ssid, pwd);
  Serial.println("");

  // Wait for connection
  // while (WiFi.status() != WL_CONNECTED) {
    // delay(500);
    // Serial.print(".");
  // }
  Serial.println("");
  Serial.print("Connected to ");
  // Serial.println(ssid);
  Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  // SD_successful = SD.begin(SD_channel_select_pin);
  // if (SD_successful){
  //   SaveData(String(), true);
  // }
  // Serial.print("SD card connection successfull: ");
  // Serial.println(SD_successful);
}

bool send_packet(){
  udp.beginPacket(WiFi.gatewayIP(), udpPort);
  // udp.printf("Voltage %2.10f,", voltage);
  udp.printf("Current %2.10f,", current);
  // udp.printf("rpm %2.10f,", rpm);
  udp.printf("time %lu,", time_now);
  udp.endPacket();
  return true; // In the future this can return true or false depending on if it succeeds or fails
}

void loop(){
  update_rail_voltage_reading();
  voltage = get_voltage_reading();
  current = get_current_reading();
  temperature = get_temperature_reading();
  Serial.print("I: ");
  Serial.print(current);
  Serial.print(" A; T: ");
  Serial.print(temperature);
  Serial.print(" C; V: ");
  Serial.print(rail_voltage);
  Serial.println(" V");
  time_now = millis();
  // send_packet();  
  if (SD_successful){
    data_to_save = "Voltage " + (String)(voltage) + ", Current " + (String)(voltage) + ",rpm " + (String)(rpm) + ",time " + (String)(time_now);
    SaveData(data_to_save);
  }
  delay(100);
}