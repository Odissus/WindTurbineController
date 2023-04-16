#include <WiFi.h>
#include <Arduino.h>
#include <esp32-hal-dac.h>

#include <NetworkHandler.h>
#include <PID.h>

// Pin layout
#define high_voltage_measurement_pin 34
#define current_measurement_pin 35
#define temperature_measurement_pin 32
#define rail_voltage_measurement_pin 33
#define hall_effect_sensor_pin 13
#define cut_off_relay_pin 14
#define fan_pin 27
#define dac_small_value_pin 25
#define dac_large_value_pin 26

#define current_correction_factor 1.0
#define Celsius_Offset 273.15
#define number_of_motor_pole_pairs 7
#define max_dac_read 3.28
#define min_dac_read 0.004

#define min_rpm_to_start 20
#define max_rpm_to_stop 20

#define RPMPIDControllerProportionalConstant 0.2
#define RPMPIDControllerDifferentialConstant 0.0
#define RPMPIDControllerIntegralConstant 0.0008
#define RPMPIDControllerMinValue 0.0
#define RPMPIDControllerMaxValue 100000.0
#define RPMPIDControllerDefaultValue 0.0

/* WiFi network name and password */
const char * ssid = "TurbineMonitor";
const char * pwd = "0000000000";
const char* serverName = "http://oracle.odissus.com:8080/turbine_statuses";

int hall_effect_event_counts = 0;
float rail_voltage = 0;
float voltage = 0;
float current = 0;
float temperature = 0;
float rpm = 0;
bool fan_on = false;
float brake_value = 0;
unsigned long time_since_last_rpm_reading = 0;
bool relay_on = false;
bool WiFi_connected = false;
time_t datetime_on_boot;

float longitude = -1.395043;
float latitude = 50.9352279;

String data_to_save;
String command;

PIDController RPMPIDController;

float get_voltage(uint8_t pin){
  // Gets voltage on a pin
  uint16_t value = analogRead(pin);
  return 0.0007963 * (float) value + 0.13;
}

float get_high_voltage(){
  float this_voltage = 0;
  for (int i = 0; i < 10; i++){ 
    this_voltage += get_voltage(high_voltage_measurement_pin) / 10.0;
  }
  voltage = this_voltage * 100.0 / 3.3;
  return voltage;
}

float get_voltage_reading(){
  return 1;
}

float get_current_reading(){
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

float get_motor_rpm2(){
  
  hall_effect_event_counts = 0;
  delay(6000);
  rpm = ((((float) hall_effect_event_counts / (float) number_of_motor_pole_pairs)) / 6.0) * 60;
  hall_effect_event_counts = 0;
  return rpm;
}

void IRAM_ATTR hall_effect_change_interrupt()
{
  hall_effect_event_counts += 1;
}

void send_dac_value(uint16_t value){
  if (value > 4607){
    value = 4607;
  }
  int smaller_value = value % 18;
  int larger_value = value / 18;
  dacWrite(dac_small_value_pin, smaller_value);
  dacWrite(dac_large_value_pin, larger_value);
}

void send_dac_voltage(float voltage){
  brake_value = voltage;
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
  fan_on = on;
  digitalWrite(fan_pin, on);
}

void read_command(){
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("HELP")){
      Serial.println("AVAILABLE COMMANDS INCLUDE: \n\tPOWER ON/OFF\n\tFAN ON/OFF\n\tBRAKE [digits...]VGE\n\tGET RPM/CURRENT/TEMPERATURE/VOLTAGE/RAIL VOLTAGE");
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
      } else if (command.equals("GET VOLTAGE")){
        float value = get_high_voltage();
        Serial.print(value);
        Serial.println(" V");
      }
    }
  }
}

void control_relay_state(){
  if (rpm > min_rpm_to_start){
    update_cut_off_relay_state(true);
  } else if (rpm < max_rpm_to_stop){
    update_cut_off_relay_state(false);
  }
}

void setup(){
  Serial.begin(9600);

  WiFi_connected = connect_to_wifi(5, true);

  attachInterrupt(hall_effect_sensor_pin, hall_effect_change_interrupt, FALLING);
  pinMode(cut_off_relay_pin, OUTPUT);
  pinMode(fan_pin, OUTPUT);
  digitalWrite(cut_off_relay_pin, LOW);
  digitalWrite(fan_pin, LOW);
  // PID Controller setting
  RPMPIDController.Init(RPMPIDControllerProportionalConstant, RPMPIDControllerIntegralConstant, RPMPIDControllerDifferentialConstant, RPMPIDControllerMinValue, RPMPIDControllerMaxValue, RPMPIDControllerDefaultValue);
  RPMPIDController.Update_target(1);

  if (WiFi_connected){
    datetime_on_boot = request_current_time();
    Serial.println(datetime_on_boot);
    if (datetime_on_boot == 0){
      ESP.restart();
    }
  }
}

void loop(){
  rail_voltage = get_rail_voltage_reading();
  voltage = get_high_voltage();
  current = get_current_reading();
  rpm = get_motor_rpm();
  temperature = get_temperature_reading();
  read_command();
  send_update_to_server(serverName, voltage, rail_voltage, current, rpm, temperature, relay_on, fan_on, brake_value, longitude, latitude);
  delay(1000);
  //send_dac_value(i);
  
  // float brake_value = - RPMPIDController.Update_and_Return(rpm);
  // Serial.println(brake_value);
  // WiFi stuff
  
  // delay(100);

}