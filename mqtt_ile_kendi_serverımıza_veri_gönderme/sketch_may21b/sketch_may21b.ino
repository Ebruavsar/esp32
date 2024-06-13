#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// WiFi ve MQTT ayarları
const char* ssid = "A";
const char* password = "15089443";
const char* mqtt_server = "192.168.179.135";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

// BNO055 sensörü için I2C pinleri
#define BNO055_SDA (21)
#define BNO055_SCL (22)

// BNO055 sensör nesnesi
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  randomSeed(micros());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
 while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.publish("espStatus", "Connected to MQTT!");
      client.subscribe("espCommand");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(9600);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // BNO055 sensörünü başlat
  if (!bno.begin())
  {
    Serial.println("BNO055 sensor not found");
    while (1);
  }
  delay(1000); // Sensörün başlaması için biraz bekle

  // Sensörü kalibrasyon moduna al
  bno.setExtCrystalUse(true);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Sensör verilerini oku
  sensors_event_t event;
  bno.getEvent(&event);

  // Verileri seri port üzerinden gönder
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  // Verileri MQTT üzerinden yayınla
  snprintf (msg, MSG_BUFFER_SIZE, "%f", event.orientation.x);
  client.publish("bno055/orientation/x", msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", event.orientation.y);
  client.publish("bno055/orientation/y", msg);
  snprintf (msg, MSG_BUFFER_SIZE, "%f", event.orientation.z);
  client.publish("bno055/orientation/z", msg);

  // Belirli bir süre bekle
  delay(500); // Örnek frekans için 500ms bekle
}
