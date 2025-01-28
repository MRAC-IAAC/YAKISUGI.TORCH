#include <WiFi.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuración del Punto de Acceso
const char* ssid = "ESP32_AccessPoint";  // Nombre de la red Wi-Fi del ESP32
const char* password = "12345678";       // Contraseña de la red Wi-Fi del ESP32

// Pines del HC-SR04
#define TRIG_PIN 25
#define ECHO_PIN 26

// Pines SPI para la pantalla OLED
#define OLED_CS     5   // Pin CS conectado al GPIO 5
#define OLED_DC     4  // Pin DC conectado al GPIO 16
#define OLED_RESET  2  // Pin RESET conectado al GPIO 17

Adafruit_SSD1306 display(128, 64, &SPI, OLED_DC, OLED_RESET, OLED_CS);

WiFiServer server(80); // Servidor en el puerto 80

void setup() {
  Serial.begin(115200);

  // Configurar pines del sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Configurar el ESP32 como Punto de Acceso
  Serial.println("Configurando Punto de Acceso...");
  WiFi.softAP(ssid, password);

  // Mostrar la IP del ESP32
  Serial.print("Punto de Acceso iniciado. Dirección IP: ");
  Serial.println(WiFi.softAPIP());

  // Iniciar el servidor
  server.begin();

  // Inicializar la pantalla OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("Error al inicializar la pantalla OLED"));
    while (true); // Detener ejecución si falla la pantalla
  }

  // Pantalla de inicio
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);

  display.setCursor(getCenteredX("WELCOME", 2), 0);
  display.println("WELCOME");

  display.setCursor(getCenteredX("this is a", 2), 16);
  display.println("this is a");
  display.setCursor(getCenteredX("YAKISUGI", 2), 33);
  display.println("YAKISUGI");
  display.setCursor(getCenteredX("torch", 2), 50);
  display.println("torch");

  display.display();
  delay(4000);
  display.clearDisplay();

  // Segunda pantalla
  display.setCursor(getCenteredX("WELCOME", 2), 0);
  display.println("WELCOME");

  display.setCursor(getCenteredX("keep right", 2), 16); // Línea 1
  display.println("keep right");
  display.setCursor(getCenteredX("distance", 2), 33); // Línea 2
  display.println("distance");
  display.setCursor(getCenteredX("from wood", 2), 50); // Línea 3
  display.println("from wood");

  display.display();
  delay(4000); // Mantener la pantalla de inicio por 4 segundos
  display.clearDisplay(); // Limpiar la pantalla
}

void loop() {
  long duration;
  float distance_cm;

  // Leer datos del sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH, 30000);

  if (duration == 0) {
    Serial.println("Error: No se recibió respuesta del HC-SR04");
    return;
  }

  distance_cm = (duration / 2.0) / 29.1;
  Serial.print("Distancia (cm): ");
  Serial.println(distance_cm, 1);

  // Enviar datos por Wi-Fi
  WiFiClient client = server.available(); // Esperar cliente
  if (client) {
    Serial.println("Cliente conectado.");
    String request = client.readStringUntil('\r');
    Serial.println("Petición recibida:");
    Serial.println(request);

    // Crear respuesta JSON
    StaticJsonDocument<200> json;
    json["distance"] = distance_cm;
    json["unit"] = "cm";

    String response;
    serializeJson(json, response);

    // Enviar respuesta al cliente
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: application/json");
    client.println("Connection: close");
    client.println();
    client.println(response);

    delay(100);
  }

  // Mostrar datos en pantalla OLED
  display.clearDisplay();

  display.setCursor(getCenteredX("DISTANCE", 2), 0);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println("DISTANCE");

  if (distance_cm > 10 && distance_cm < 15) {
    char distanceStr[8];
    sprintf(distanceStr, "%.1f cm", distance_cm);
    display.setCursor(getCenteredX(distanceStr, 2), 40);
    display.setTextSize(2);
    display.print(distanceStr);
  }

  if (distance_cm < 10) {
    display.setCursor(getCenteredX("TOO CLOSE", 2), 40);
    display.setTextSize(2);
    display.println("TOO CLOSE");
  }

  if (distance_cm > 15) {
    display.setCursor(getCenteredX("TOO FAR", 2), 40);
    display.setTextSize(2);
    display.println("TOO FAR");
  }

  display.display();
  delay(50);
}

// Función para calcular la posición centrada
int getCenteredX(const char *text, int textSize) {
  int textWidth = strlen(text) * 6 * textSize; // Cada carácter tiene un ancho base de 6 píxeles
  return (128 - textWidth) / 2;               // Centrar el texto en una pantalla de 128 píxeles de ancho
}
