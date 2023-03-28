#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>
#include <SD.h>
#include <esp32-hal-dac.h>

#define current_measurement_pin 34
#define temperature_measurement_pin 35
#define rail_voltage_measurement_pin 33
#define hall_effect_sensor_pin 13
#define cut_off_relay_pin 14
#define fan_pin 27
#define dac_small_value_pin 25
#define dac_large_value_pin 26
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
#define number_of_motor_pole_pairs 7
#define max_dac_read 3.28
#define min_dac_read 0.004
#define DAC_bits 12

/* WiFi network name and password */
const char * ssid = "TurbineMonitor";
const char * pwd = "0000000000";
const int udpPort = 44444;
WiFiUDP udp;

int hall_effect_event_counts = 0;
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
unsigned long time_since_last_rpm_reading = 0;
bool fan_relay_on = false;
int i = 0;
String data_to_save;
String command;

float get_voltage(uint8_t pin){
  uint16_t value = analogRead(pin);
  return 0.0007963 * (float) value + 0.13;
}

float get_voltage_reading(){
  return 1;
}

float get_current_reading(){
  // V=IR -> I = V/R
  //temp_voltage = (float) analogRead(current_measurement_pin) / 1240.9;
  //current = -9.9 * temp_voltage + 31.581;
  float this_current = 0;
  float correction_1 = 1.24, correction_2 = 0.33;
  for (int i = 0; i < 10; i++){ 
    this_current += get_voltage(current_measurement_pin) / 10.0;
  }
  float zero_current_point =  (1.0 + (220.0/1000.0)) * (rail_voltage / 2);
  current = ((20 - 0) / (0 - zero_current_point)) * this_current + 20;
  current = correction_1 * current + correction_2;
  if ((current>-0.7) and (current<0)){
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
    this_temperature += get_voltage(temperature_measurement_pin) / 10.0;
  }
  Therm_V = this_temperature;
  R2 = R1 * (Therm_V/(rail_voltage-Therm_V));
  logR2 = log(R2);
  Temperature = (1 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Temperature -= Celsius_Offset;
  return Temperature;
}

float get_rail_voltage_reading(){
  rail_voltage = 2 * get_voltage(rail_voltage_measurement_pin);
  return rail_voltage;
}

float get_motor_rpm(){
  unsigned long time_now = micros();
  float dt = (float) (time_now - time_since_last_rpm_reading) / (1000.0 * 1000.0);
  if (dt > 1){ // limit to only work every one second
    rpm = 60.0 * (((float) hall_effect_event_counts / (float) number_of_motor_pole_pairs) / dt);
    // update the time and reset the hall effect event counts
    time_since_last_rpm_reading=micros();
    hall_effect_event_counts = 0;
  }
  return rpm;
}

void read_reference(){

}

void IRAM_ATTR hole_detected_interrupt()
{
  holes += 1;
}

void IRAM_ATTR hall_effect_change_interrupt()
{
  hall_effect_event_counts += 1;
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

  //attachInterrupt(hall_effect_sensor_pin, hall_effect_change_interrupt, RISING);
  attachInterrupt(hall_effect_sensor_pin, hall_effect_change_interrupt, FALLING);
  // Serial.println(WiFi.localIP());

  // SD_successful = SD.begin(SD_channel_select_pin);
  // if (SD_successful){
  //   SaveData(String(), true);
  // }
  // Serial.print("SD card connection successfull: ");
  // Serial.println(SD_successful);
  // dacAttachPin(25);
  // dacAttachPin(25);
  pinMode(cut_off_relay_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  digitalWrite(cut_off_relay_pin, LOW);
  digitalWrite(fan_pin, LOW);
  randomSeed(0);
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

void send_dac_value(uint16_t value){
  if (value > 4607){
    value = 4607;
  }
  int smaller_value = value % 18;
  int larger_value = value / 18;
  Serial.print(smaller_value);
  Serial.print(" -> ");
  Serial.print((float ) smaller_value / (18.0 * 255.0) * max_dac_read);
  Serial.print(" , ");
  Serial.print(larger_value); 
  Serial.print(" -> ");
  Serial.print((float ) larger_value / 255.0 * max_dac_read);
  Serial.print(" ; ");
  Serial.print(((((float) smaller_value) + (float) larger_value * 18))); 
  Serial.print(" -> ");
  Serial.print(((((float) smaller_value / 18.0) + (float) larger_value) / 255.0) * max_dac_read);

  //Serial.print(smaller_value);
  //Serial.print(" : ");
  //Serial.println(larger_value);
  dacWrite(dac_small_value_pin, smaller_value);
  dacWrite(dac_large_value_pin, larger_value);
}

void send_dac_voltage(float voltage){
  uint16_t bits = min(1.0, (double)(voltage / (float) (max_dac_read - min_dac_read))) * 4607;
  Serial.print(voltage);
  Serial.print(" -> ");
  Serial.print(bits);
  Serial.print(" ; ");
  send_dac_value(bits);
}

void update_cut_off_relay_state(bool on){
  digitalWrite(cut_off_relay_pin, on);
}

void update_cooling_fan_state(bool on){
  digitalWrite(fan_pin, on);
}

void read_command(){
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("HELP")){
      Serial.println("AVAILABLE COMMANDS INCLUDE: \n\tPOWER ON/OFF\n\tFAN ON/OFF\n\tBRAKE ON/OFF\n\tGET RPM/CURRENT/TEMPERATURE/RAIL VOLTAGE");
    } else if (command.substring(0,5) == "POWER"){
      update_cut_off_relay_state(command.equals("POWER ON"));
    } else if (command.substring(0,3) == "FAN"){
      update_cooling_fan_state(command.equals("FAN ON"));
    } else if (command.substring(0,5) == "BRAKE"){
      float value = command.substring(6, command.length()-1).toFloat();
      send_dac_voltage(value);
    } else if (command.substring(0,3) == "GET"){
      if (command.equals("GET RPM")){
        float value = get_motor_rpm();
        Serial.print(value);
        Serial.println(" RPM");
      } else if (command.equals("GET CURRENT")){
        float value = get_current_reading();
        Serial.print(value);
        Serial.println(" A");
      } else if (command.equals("GET TEMPERATURE")){
        float value = get_temperature_reading();
        Serial.print(value);
        Serial.println(" C");
      } else if (command.equals("GET RAIL VOLTAGE")){
        float value = get_rail_voltage_reading();
        Serial.print(value);
        Serial.println(" V");
      }
    }
  }
}

void loop(){
  get_rail_voltage_reading();
  // https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html
  // 8 channels: GPIO32 - GPIO39
  // 10 channels: GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
  // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/dac.html
  //delay(5000);
  //float new_voltage = 3.5 * (float) random(0, 100000) / 100000.0;
  //send_dac_voltage(new_voltage);
  read_command();
  //send_dac_value(i);
}