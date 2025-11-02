#include <WiFi.h>

const char *ssid = "Team02_ESP32_Server";    // network name
const char *password = "12345678";           // must be at least 8 characters

WiFiServer server(80);

void setup() {
  Serial.begin(115200);

  // Start Wi-Fi in Access Point mode
  WiFi.softAP(ssid, password);

  // print IP address in serial monitor
  // type http://[IP address] in browser to access web server
  Serial.print("Access Point started. Connect to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // Start web server
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected.");

    // Wait for data from client
    while (client.connected()) {
      if (client.available()) {
        client.read(); // just read and ignore request
        break;
      }
    }

    // Send webpage
    float temp = random(200, 300) / 10.0;
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
    client.println("<!DOCTYPE html><html><body>");
    client.println("<h2>ESP32 Local Server</h2>");
    
    //print data
    client.print("<p>Temperature: ");
    client.print(temp);
    client.println(" °C</p>");
    client.println("</body></html>");
    client.println();

    //disconnect at end of session
    client.stop();
    Serial.println("Client disconnected.");
  }
}
