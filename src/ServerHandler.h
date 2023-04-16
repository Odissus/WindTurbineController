#include <Arduino.h>
// Header guard
#ifndef SERVER_HANDLER_H
#define SERVER_HANDLER_H

time_t request_current_time();

String send_update_to_server(const char* serverName, float voltage, float current, int rpm, float temperature, float longitude, float latitude);

#endif
