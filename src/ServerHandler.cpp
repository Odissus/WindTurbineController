#include <Arduino.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>

#include <server_authorisation_token.h>
#include <ServerHandler.h>


time_t request_current_time(){
  StaticJsonDocument<256> doc;
  HTTPClient http;
  http.begin("http://oracle.odissus.com:8080/turbine_statuses/time_now");
  String server_authorisation = (String) server_authorisation_token;
  http.addHeader("Authorization", "Bearer " + server_authorisation); // Add the token to the header
  int httpResponseCode = http.GET();
  String payload = "";
  time_t default_time = 0;
  if (httpResponseCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.println(payload);
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
      return default_time;
    }
        // Extract the current time
    const char* timeString = doc["time"];
    Serial.print("Current time: ");
    Serial.println(timeString);

    // Convert the time string to a time_t variable
    struct tm timeStruct;
    strptime(timeString, "%Y-%m-%dT%H:%M:%SZ", &timeStruct);
    time_t currentTime = mktime(&timeStruct);
    Serial.print("Current time in seconds since epoch: ");
    Serial.println(currentTime);
    return currentTime;

  } else {
    Serial.print("Error getting time: ");
    Serial.println(httpResponseCode);
  }
  http.end();
  return default_time;
}

String send_update_to_server(const char* serverName, float voltage, float current, int rpm, float temperature, float longitude, float latitude) {
  WiFiClient client;
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  http.addHeader("Content-Type", "application/json");
  String server_authorisation = (String) server_authorisation_token;
  http.addHeader("Authorization", "Bearer " + server_authorisation); // Add the token to the header
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  
  // Send HTTP POST request

  StaticJsonDocument<256> doc;

  // Add data to the document
  doc["id"] = 0;
  doc["Name"] = "Demo Turbine Monitor";
  doc["Voltage"] = voltage;
  doc["Current"] = current;
  doc["Rpm"] = rpm;
  doc["Temperature"] = temperature;
  doc["Longitude"] = longitude;
  doc["Latitude"] = latitude;

  // Serialize the JSON document to a string
  String payload;
  serializeJson(doc, payload);

  int httpResponseCode = http.POST(payload);
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}