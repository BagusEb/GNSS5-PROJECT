#include <ArduinoJson.h>
#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

// WiFi Configuration
const char* ssid = "";         
const char* password = ""; 

// MQTT Configuration
const char* mqtt_server = "";  
const int mqtt_port = 1883;                  
const char* mqtt_topic = "";   

// Pin definitions
#define RXD2 16
#define TXD2 17
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Clients initialization
WiFiClient espClient;
PubSubClient client(espClient);

// Buffer for NMEA data
String nmeaBuffer = "";

// Storage for parsed values
struct GPSData {
  String timeNow5 = "";    // Combined date and time in yyyy/mm/dd hh:mm:ss format
  String LAT05 = "";       // Latitude
  String LAD05 = "";       // Latitude Direction
  int LAD05_deg = 0;       // Latitude Direction (degrees)
  String LON05 = "";       // Longitude
  String LOD05 = "";       // Longitude Direction
  int LOD05_deg = 0;       // Longitude Direction (degrees)
  String SAT05 = "";       // Total Satellites
  String ALT05 = "";       // Altitude
  String GSE05 = ""; 
  String PDO05 = "";       // Position Dilution of Precision
  String HDO05 = "";       // Horizontal Dilution of Precision
  String VDO05 = "";       // Vertical Dilution of Precision
  String SCO05 = "";       // Satellite Count
  String SNR05 = "";       // Signal Noise Ratio
  String UTC05 = "";       // Time
  String DAT05 = "";       // Date
  String SPD05 = "";       // Speed (from RMC message)
  String HAD05 = "";       // Heading/Course (from RMC message)
  String MVR05 = "";       // Magnetic Variation
  int MVA05_deg = 0;       // Magnetic Value (degrees)
};

GPSData gpsData;

// Timing variables
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 15000; // 15 seconds
unsigned long lastWifiCheckTime = 0;
const unsigned long wifiCheckInterval = 30000; // 30 seconds

// Direction conversion helper
int directionToDegrees(String direction) {
  if (direction == "N") return 90;
  if (direction == "E") return 180;
  if (direction == "S") return 270;
  if (direction == "W") return 360;
  return 0; 
}

void setup_wifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected - IP: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi connection FAILED");
  }
}

void reconnect() {
  if (client.connected()) return;

  Serial.print("Connecting to MQTT...");
  
  if (client.connect("")) { // connect without client ID
    Serial.println("connected");
  } else {
    Serial.print("failed, rc=");
    Serial.println(client.state());
  }
}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  randomSeed(micros());
  dht.begin();
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setBufferSize(1024); // Larger buffer
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Check WiFi connection periodically
  if (currentMillis - lastWifiCheckTime >= wifiCheckInterval) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi reconnecting...");
      setup_wifi();
    }
    lastWifiCheckTime = currentMillis;
  }
  
  // Check MQTT connection
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Read GPS data
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (nmeaBuffer.length() > 0) {
        parseNMEA(nmeaBuffer);
        nmeaBuffer = "";
      }
    } else if (c != '\r') {
      nmeaBuffer += c;
    }
  }
  
  // Send JSON data every 15 seconds
  if (currentMillis - lastSendTime >= sendInterval) {
    // Update combined date and time before sending
    updateCombinedDateTime();
    sendJSONData();
    lastSendTime = currentMillis;
  }
}

void parseNMEA(String nmea) {
  if (nmea.length() < 6) return;
  
  // Parse different NMEA sentences
  if (nmea.startsWith("$GPGGA") || nmea.startsWith("$GNGGA")) {
    parseGPGGA(nmea);
  } else if (nmea.startsWith("$GPGSA") || nmea.startsWith("$GNGSA")) {
    parseGPGSA(nmea);
  } else if (nmea.startsWith("$GPGSV") || nmea.startsWith("$GNGSV")) {
    parseGPGSV(nmea);
  } else if (nmea.startsWith("$GPRMC") || nmea.startsWith("$GNRMC")) {
    parseGPRMC(nmea);
    // After parsing GPRMC, we can update the combined date and time
    updateCombinedDateTime();
  }
}

// New function to update the combined date and time
void updateCombinedDateTime() {
  // Only update if we have both date and time
  if (gpsData.DAT05.length() > 0 && gpsData.UTC05.length() > 0) {
    // Format: YYYY/MM/DD HH:MM:SS
    String year = gpsData.DAT05.substring(0, 4);
    String month = gpsData.DAT05.substring(4, 6);
    String day = gpsData.DAT05.substring(6, 8);
    
    // Format time with colons for timeNow5
    String formattedTime = "";
    if (gpsData.UTC05.length() >= 6) {
      formattedTime = gpsData.UTC05.substring(0, 2) + ":" + 
                     gpsData.UTC05.substring(2, 4) + ":" + 
                     gpsData.UTC05.substring(4, 6);
    } else {
      formattedTime = gpsData.UTC05;
    }
    
    gpsData.timeNow5 = year + "/" + month + "/" + day + " " + formattedTime;
  }
}

void parseGPGGA(String nmea) {
  String parts[15];
  splitString(nmea, parts, 15);
  
  if (parts[1].length() > 0) {
    String formattedTime = formatTime(parts[1]);
    // UTC05 will be used later to build the combined date/time
    gpsData.UTC05 = formattedTime;
  }
  if (parts[2].length() > 0) gpsData.LAT05 = parts[2];
  if (parts[3].length() > 0) {
    gpsData.LAD05 = parts[3];
    gpsData.LAD05_deg = directionToDegrees(parts[3]);
  }
  if (parts[4].length() > 0) gpsData.LON05 = parts[4];
  if (parts[5].length() > 0) {
    gpsData.LOD05 = parts[5];
    gpsData.LOD05_deg = directionToDegrees(parts[5]);
  }
  if (parts[7].length() > 0) gpsData.SAT05 = parts[7];
  if (parts[9].length() > 0) gpsData.ALT05 = parts[9];

  if (parts[11].length() > 0) gpsData.GSE05 = parts[11];
}

void parseGPGSA(String nmea) {
  String parts[20];
  splitString(nmea, parts, 20);
  
  if (parts[15].length() > 0) gpsData.PDO05 = parts[15];
  if (parts[16].length() > 0) gpsData.HDO05 = parts[16];
  if (parts[17].length() > 0) gpsData.VDO05 = parts[17];
}

void parseGPGSV(String nmea) {
  String parts[20];
  splitString(nmea, parts, 20);
  
  if (parts[3].length() > 0) gpsData.SCO05 = parts[3];
  if (parts[7].length() > 0) gpsData.SNR05 = parts[7];
}

void parseGPRMC(String nmea) {
  String parts[13];
  splitString(nmea, parts, 13);
  
  if (parts[1].length() > 0) {
    String formattedTime = formatTime(parts[1]);
    gpsData.UTC05 = formattedTime;
  }
  if (parts[7].length() > 0) {
    // Extract speed from RMC message (field 7)
    gpsData.SPD05 = parts[7];
  }
  if (parts[8].length() > 0) {
    // Extract heading/course from RMC message (field 8)
    gpsData.HAD05 = parts[8];
  }
  if (parts[9].length() > 0) {
    String formattedDate = formatDate(parts[9]);
    gpsData.DAT05 = formattedDate;
  }
  if (parts[10].length() > 0) gpsData.MVR05 = parts[10];
  if (parts[11].length() > 0) {
    gpsData.MVA05_deg = directionToDegrees(parts[11]);
  }
}

// Split NMEA string by commas
void splitString(String str, String parts[], int maxParts) {
  int partIndex = 0;
  int startPos = 0;
  
  for (int i = 0; i < str.length() && partIndex < maxParts; i++) {
    if (str.charAt(i) == ',' || str.charAt(i) == '*') {
      parts[partIndex] = str.substring(startPos, i);
      startPos = i + 1;
      partIndex++;
      if (str.charAt(i) == '*') break;
    }
  }
}

// Format time from HHMMSS.SSS to HHMMSS (without colons)
String formatTime(String utcTime) {
  if (utcTime.length() >= 6) {
    return utcTime.substring(0, 6); 
  }
  return utcTime;
}

// Format date from DDMMYY to YYYY/MM/DD (with slashes for timeNow5 but without slashes for DAT05)
String formatDate(String date) {
  if (date.length() >= 6) {
    // For DAT05, no slashes (YYYYMMDD format)
    return "20" + date.substring(4, 6) + 
           date.substring(2, 4) + 
           date.substring(0, 2);
  }
  return date;
}

void sendJSONData() {
  if (!client.connected()) {
    reconnect();
    if (!client.connected()) return;
  }

  StaticJsonDocument<1024> doc;
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Add all GPS data to JSON document with original field names
  doc["timeNow5"] = gpsData.timeNow5;
  doc["LAT05"] = gpsData.LAT05;
  doc["LAD05"] = gpsData.LAD05_deg;
  doc["LON05"] = gpsData.LON05;
  doc["LOD05"] = gpsData.LOD05_deg;
  doc["SAT05"] = gpsData.SAT05;
  doc["ALT05"] = gpsData.ALT05;
  doc["GSE05"] = gpsData.GSE05;
  doc["PDO05"] = gpsData.PDO05;
  doc["HDO05"] = gpsData.HDO05;
  doc["VDO05"] = gpsData.VDO05;
  doc["SCO05"] = gpsData.SCO05;
  doc["SNR05"] = gpsData.SNR05;
  doc["UTC05"] = gpsData.UTC05;
  doc["DAT05"] = gpsData.DAT05;
  doc["SPD05"] = gpsData.SPD05;  
  doc["HAD05"] = gpsData.HAD05;  
  doc["MVR05"] = gpsData.MVR05;
  doc["MVA05"] = gpsData.MVA05_deg;
  
  // Add temperature and humidity with original field names
  doc["TEM05"] = isnan(temperature) ? "N/A" : String(temperature, 1);
  doc["HUM05"] = isnan(humidity) ? "N/A" : String(humidity, 1);
  
  String jsonOutput;
  serializeJson(doc, jsonOutput);
  
  bool published = client.publish(mqtt_topic, jsonOutput.c_str(), true);
  Serial.println(published ? "Published to MQTT" : "Failed to publish");
  Serial.println(jsonOutput); // Print JSON for debugging
}
