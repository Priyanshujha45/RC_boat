#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>
#include <DHT.h>

// Configuration
#define CONFIG_FILE "/config.json"
#define MAX_DISTANCE 400 // cm
#define GPS_BAUD 9600
#define DHTPIN 5         // GPIO4 for DHT11
#define DHTTYPE DHT11

// Hardware pins
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_EN = 14;
const int MOTOR_B_IN1 = 33;
const int MOTOR_B_IN2 = 25;
const int MOTOR_B_EN = 32;
const int TRIG_PIN = 12;
const int ECHO_PIN = 13;
const int GPS_RX = 16;
const int GPS_TX = 17;

// Default settings
struct Config {
  String ssid = "BoatController";
  String password = "securepass123";
  String loginPassword = "admin123";
  int defaultSpeed = 150;
  int maxSpeed = 255;
};

// Global objects
Config config;
WebServer server(80);
WebSocketsServer webSocket(81);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
DHT dht(DHTPIN, DHTTYPE);


// Runtime variables
bool isAuthenticated = false;
int currentSpeed;
unsigned long lastSensorUpdate = 0;
unsigned long lastGPSCheck = 0;
float distance = 0;
String gpsData = "No GPS data";
float temperature = 0;
float humidity = 0;
unsigned long lastTempCheck = 0;
bool obstacleDetected = false;
const int OBSTACLE_DISTANCE = 15; // cm

void setup() {
  Serial.begin(115200);
   // Initialize DHT11
 
  dht.begin();
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }

  // Initialize hardware pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Initialize PWM channels
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(MOTOR_A_EN, 0);
  ledcAttachPin(MOTOR_B_EN, 1);

  // Load configuration
  loadConfig();
  currentSpeed = config.defaultSpeed;
  stopMotors();

  // Initialize GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  // Start WiFi
  WiFi.softAP(config.ssid.c_str(), config.password.c_str());
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Initialize WebSocket
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Setup server routes
  server.on("/", HTTP_GET, []() {
    if (!isAuthenticated) {
      server.sendHeader("Location", "/login.html");
      server.send(302);
    } else {
      server.sendHeader("Location", "/control.html");
      server.send(302);
    }
  });

  server.on("/login.html", HTTP_GET, []() {
    if (!SPIFFS.exists("/www/login.html")) {
      server.send(404, "text/plain", "login.html not found");
      return;
    }
    File file = SPIFFS.open("/www/login.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });

  server.on("/control.html", HTTP_GET, []() {
    if (!isAuthenticated) {
      server.sendHeader("Location", "/login.html");
      server.send(302);
      return;
    }
    if (!SPIFFS.exists("/www/control.html")) {
      server.send(404, "text/plain", "control.html not found");
      return;
    }
    File file = SPIFFS.open("/www/control.html", "r");
    server.streamFile(file, "text/html");
    file.close();
  });

  server.on("/css/style.css", HTTP_GET, []() {
    if (!SPIFFS.exists("/www/css/style.css")) {
      server.send(404, "text/plain", "style.css not found");
      return;
    }
    File file = SPIFFS.open("/www/css/style.css", "r");
    server.streamFile(file, "text/css");
    file.close();
  });

  server.on("/js/script.js", HTTP_GET, []() {
    if (!SPIFFS.exists("/www/js/script.js")) {
      server.send(404, "text/plain", "script.js not found");
      return;
    }
    File file = SPIFFS.open("/www/js/script.js", "r");
    server.streamFile(file, "application/javascript");
    file.close();
  });

  // API endpoints
  server.on("/api/login", HTTP_POST, handleLogin);
  server.on("/api/logout", HTTP_GET, handleLogout);
  server.on("/api/status", HTTP_GET, handleGetStatus);
  
  server.onNotFound([]() {
    server.send(404, "text/plain", "404: Not Found");
  });

  // Start server
  server.begin();
}

void loop() {
  server.handleClient();
  webSocket.loop();
  updateSensors();
  updateGPS();
  updateDHT();
  
}

void updateDHT() {
  if (millis() - lastTempCheck > 2000) { // Update every 2 seconds
    float newTemp = dht.readTemperature();
    float newHumidity = dht.readHumidity();
    
    if (!isnan(newTemp)) temperature = newTemp;
    if (!isnan(newHumidity)) humidity = newHumidity;
    
    lastTempCheck = millis();
    
    // Broadcast update
    String status = "{\"type\":\"status\",\"temperature\":" + String(temperature, 1) + 
                   ",\"humidity\":" + String(humidity, 1) + "}";
    webSocket.broadcastTXT(status);
  }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    // ========================
    // 1. When Client Disconnects
    // ========================
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", num);
      break;

    // ========================
    // 2. When New Client Connects
    // ========================
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("New client %u from %s\n", num, ip.toString().c_str());
      
      // Send current status to the new client
      String statusJson = 
        "{\"type\":\"status\","
        "\"speed\":" + String(currentSpeed) + "," +
        "\"distance\":" + String(distance) + "," +
        "\"obstacle\":" + String(obstacleDetected ? "true" : "false") + "," +
        "\"temp\":" + String(temperature, 1) + "," +
        "\"humidity\":" + String(humidity, 1) + "," +
        "\"gps\":\"" + gpsData + "\"}";
      
      webSocket.sendTXT(num, statusJson);
      break;
    }

    // ========================
    // 3. When Message Received
    // ========================
    case WStype_TEXT: {
      String cmd = String((char*)payload);
      cmd.trim();
      
      Serial.printf("[%u] Received: %s\n", num, cmd.c_str());

      // Safety Check 1: Require login
      if (!isAuthenticated) {
        Serial.println("  ! Rejected: Not authenticated");
        webSocket.sendTXT(num, "{\"error\":\"Please login first\"}");
        return;
      }

      // Safety Check 2: Stop if obstacle detected
      if (obstacleDetected && cmd != "STOP") {
        Serial.println("  ! Blocked: Obstacle detected");
        webSocket.sendTXT(num, "{\"error\":\"OBSTACLE! Motors locked\"}");
        return;
      }

      // ==============================
      // ACTUAL MOTOR CONTROL LOGIC
      // ==============================
      if (cmd == "FORWARD") {
        Serial.println("  > Moving FORWARD");
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
        ledcWrite(0, currentSpeed);
        ledcWrite(1, currentSpeed);
      }
      else if (cmd == "REVERSE") {
        Serial.println("  > Moving REVERSE");
        digitalWrite(MOTOR_A_IN1, HIGH);
        digitalWrite(MOTOR_A_IN2, LOW);
        digitalWrite(MOTOR_B_IN1, HIGH);
        digitalWrite(MOTOR_B_IN2, LOW);
        ledcWrite(0, currentSpeed);
        ledcWrite(1, currentSpeed);
      }
      else if (cmd == "LEFT") {
        Serial.println("  > Turning LEFT");
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, HIGH);
        ledcWrite(0, 0);
        ledcWrite(1, currentSpeed);
      }
      else if (cmd == "RIGHT") {
        Serial.println("  > Turning RIGHT");
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, HIGH);
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
        ledcWrite(0, currentSpeed);
        ledcWrite(1, 0);
      }
      else if (cmd == "STOP") {
        Serial.println("  > STOPPING motors");
        digitalWrite(MOTOR_A_IN1, LOW);
        digitalWrite(MOTOR_A_IN2, LOW);
        digitalWrite(MOTOR_B_IN1, LOW);
        digitalWrite(MOTOR_B_IN2, LOW);
        ledcWrite(0, 0);
        ledcWrite(1, 0);
      }
      else if (cmd.startsWith("SPEED:")) {
        int newSpeed = cmd.substring(6).toInt();
        currentSpeed = constrain(newSpeed, 0, 255);
        Serial.printf("  > Speed set to %d\n", currentSpeed);
        webSocket.sendTXT(num, "{\"speed\":" + String(currentSpeed) + "}");
      }
      else {
        Serial.println("  ! Unknown command");
        webSocket.sendTXT(num, "{\"error\":\"Invalid command\"}");
      }
      break;
    }

    // ========================
    // Other WebSocket Events
    // ========================
    case WStype_BIN:
      Serial.printf("[%u] Received binary data (%u bytes)\n", num, length);
      break;
      
    case WStype_ERROR:
      Serial.printf("[%u] WebSocket error\n", num);
      break;
      
    default:
      Serial.printf("[%u] Unknown WebSocket event type: %d\n", num, type);
      break;
  }
}

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW);
  digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN1, LOW);
  digitalWrite(MOTOR_B_IN2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

void updateSensors() {
  if (millis() - lastSensorUpdate > 200) { // Faster checks for obstacles
    distance = getDistance();
    
    // Obstacle detection logic
    if (distance <= OBSTACLE_DISTANCE && distance > 0) {
      if (!obstacleDetected) {
        Serial.println("OBSTACLE DETECTED! Stopping motors.");
        stopMotors();
        obstacleDetected = true;
      }
    } else {
      obstacleDetected = false;
    }

    lastSensorUpdate = millis();
    
    // Send status with obstacle flag
    String status = "{\"type\":\"status\",\"distance\":" + String(distance) + 
                   ",\"obstacle\":" + String(obstacleDetected ? "true" : "false") + "}";
    webSocket.broadcastTXT(status);
  }
}

void updateGPS() {
  if (millis() - lastGPSCheck > 1000) {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          gpsData = "Lat:" + String(gps.location.lat(), 6) + ",Lng:" + String(gps.location.lng(), 6);
        } else {
          gpsData = "No GPS fix";
        }
      }
    }
    lastGPSCheck = millis();
  }
}

void loadConfig() {
  if (SPIFFS.exists(CONFIG_FILE)) {
    File file = SPIFFS.open(CONFIG_FILE, "r");
    if (file) {
      String content = file.readString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, content);
      
      config.ssid = doc["ssid"].as<String>();
      config.password = doc["password"].as<String>();
      config.loginPassword = doc["loginPassword"].as<String>();
      config.defaultSpeed = doc["defaultSpeed"];
      config.maxSpeed = doc["maxSpeed"];
      
      file.close();
    }
  }
}

void handleLogin() {
  if (server.hasArg("password")) {
    String submittedPassword = server.arg("password");
    if (submittedPassword == config.loginPassword) {
      isAuthenticated = true;
      server.send(200, "application/json", "{\"success\":true}");
    } else {
      server.send(401, "application/json", "{\"error\":\"Invalid password\"}");
    }
  } else {
    server.send(400, "application/json", "{\"error\":\"Password required\"}");
  }
}

void handleLogout() {
  isAuthenticated = false;
  server.send(200, "application/json", "{\"success\":true}");
}

void handleGetStatus() {
  DynamicJsonDocument doc(512);
  doc["authenticated"] = isAuthenticated;
  doc["speed"] = currentSpeed;
  doc["distance"] = distance;
  doc["gps"] = gpsData;
  
  String output;
  serializeJson(doc, output);
  server.send(200, "application/json", output);
}
