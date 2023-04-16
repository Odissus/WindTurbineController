#include <Arduino.h>
// Header guard
#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

time_t request_current_time();

String send_update_to_server(const char* serverName, float voltage, float rail_voltage, float current, int rpm, float temperature, bool relay_on, bool fan_on, float brake_value, float longitude, float latitude);

bool connect_to_wifi(int max_wait_time_seconds, bool turn_radio_off_if_failed);

#endif
