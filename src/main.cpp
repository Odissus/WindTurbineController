#include <WiFi.h>
//#include <WiFiUdp.h>
#include <Arduino.h>
#include <SD.h>
#include <esp32-hal-dac.h>

#include <WebServer.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#include <WiFi_credentials.h>
#include <ServerHandler.h>
#include <server_authorisation_token.h>

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
bool relay_on = false;
bool WiFi_connected = false;
bool stop_sending_requests = false;
time_t datetime_on_boot;

float longitude = -1.395043;
float latitude = 50.9352279;

String data_to_save;
String command;

class PIDController{
  public:
    float target_value;
    float Kp;
    float Ki;
    float Kd;

    float output_min;
    float output_max;

    unsigned long minimum_time_difference = 0;

    float default_output;

  private:
    unsigned long time_since_last_ran = 0;
    unsigned long time_now = 0;
    float total_error = 0;
    float last_error = 0;
    float control_signal;

  public:
    void Init(float proportional_constant, float integral_constant, float differential_constant, float min_output, float max_output, float default_output, unsigned long minimum_time_difference=0, float target_value=0) {  // Method/function defined inside the class
      this->Kp = proportional_constant;
      this->Ki = integral_constant;
      this->Kd = differential_constant;
      this->output_min = min_output;
      this->output_max = max_output;
      this->minimum_time_difference = minimum_time_difference;
      this->target_value = target_value;
      this->default_output = default_output;
      this->control_signal = default_output;
    }
  
  public:
    void Update_target(float target_value) {
      this->target_value = target_value;
    }

  public:
    double Update_and_Return(float sensed_output) {
      this->time_now = millis();
      unsigned long delta_time = this->time_now - this->time_since_last_ran; //delta time interval 
      if (delta_time >= minimum_time_difference){
        float error = this->target_value - sensed_output;

        this->total_error += error; //accumalates the error - integral term
        if (total_error >= this->output_max) {
          this->total_error = this->output_max;
        } else if (total_error <= this->output_min) {
          this->total_error = this->output_min;
        }

        float delta_error = error - this->last_error; //difference of error for derivative term

        float control_signal = this->Kp * error + (this->Ki * delta_time) * total_error + (this->Kd / delta_time) * delta_error; //PID control compute
        if (control_signal >= this->output_max){
          control_signal = this->output_max;
        } else if (control_signal <= this->output_min){
          control_signal = this->output_min;
        }

        this->last_error = error;
        this->time_since_last_ran = this->time_now;
        this->control_signal = control_signal;
        return control_signal;
      }
    // If the controller hasn't ran for long enough, return last value 
    return this->control_signal;
    }
};

// PID
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

float get_motor_rpm2(){
  
  hall_effect_event_counts = 0;
  delay(6000);
  rpm = ((((float) hall_effect_event_counts / (float) number_of_motor_pole_pairs)) / 6.0) * 60;
  hall_effect_event_counts = 0;
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

bool connect_to_wifi(int max_wait_time_seconds, bool turn_radio_off_if_failed){
  // Connections to Wifi and returns a bool specifying if a connection was succesfull
  const char * SSID = WiFiSSID;
  const char * password = WiFiPasswd;
  const char * backup_SSID = WiFiSSID2;
  const char * backup_password = WiFiPasswd2;

  // Try default credentials
  unsigned long start_time = millis();
  WiFi.begin(SSID, password);
  Serial.println("Using default network credentials");

  // Try default connection first
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - start_time) < (max_wait_time_seconds * 1000 / 2))) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED){
    Serial.print("Connected to ");
    Serial.println(SSID);
    Serial.println(WiFi.gatewayIP());
    return true;
  }

  Serial.println("Changing over to backup network");
  WiFi.disconnect(false, true);
  WiFi.begin(backup_SSID, backup_password);

  // Change over to backup connection
  while ((WiFi.status() != WL_CONNECTED) && ((millis() - start_time) < (max_wait_time_seconds * 1000))) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED){
    Serial.print("Connected to ");
    Serial.println(backup_SSID);
    Serial.println(WiFi.gatewayIP());
    return true;
  }

  WiFi.disconnect(turn_radio_off_if_failed, true);
  return false;
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
  get_rail_voltage_reading();
  rpm = get_motor_rpm();
  // https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html
  // 8 channels: GPIO32 - GPIO39
  // 10 channels: GPIO0, GPIO2, GPIO4, GPIO12 - GPIO15, GOIO25 - GPIO27
  // https://docs.espressif.com/projects/arduino-esp32/en/latest/api/dac.html
  //delay(5000);
  //float new_voltage = 3.5 * (float) random(0, 100000) / 100000.0;
  //send_dac_voltage(new_voltage);
  read_command();
  send_update_to_server(serverName, voltage, current, rpm, temperature, longitude, latitude);
  delay(1000);
  //send_dac_value(i);
  
  // float brake_value = - RPMPIDController.Update_and_Return(rpm);
  // Serial.println(brake_value);
  // WiFi stuff
  
  // delay(100);

}