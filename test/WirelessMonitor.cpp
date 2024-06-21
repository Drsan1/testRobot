#include <WiFi.h>
#include <WiFiClientSecure.h>

const char* ssid     = "ESP32";
const char* password = "123456789";

WiFiServer server(23);

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  server.begin();

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

int i = 0;

void loop() {
  WiFiClient client = server.available();

  if (client) {                               
    while (client.connected()) {        
      if (client.available()) {            
        char c = client.read();            
        Serial.write(c);                   
      }

      i++;
      client.print("i = ");
      client.println(i);
      delay(100);
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}