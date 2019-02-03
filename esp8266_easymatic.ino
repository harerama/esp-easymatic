const char* ssid = "JetG_Wifi";
const char* password = "maximilien";
const String baseurl = "/core/api/jeeApi.php?plugin=virtual&apikey=JuOGYdRJAPKNiJC33aSrXg48HMXKyC7G&type=virtual";
const char* host = "172.27.1.3";
const int port = 80;

#define CYCLE_COUNT_1200 160000000/1200
#define CYCLE_COUNT_4800 160000000/4800

#define SEND_TO_JEEDOM_INTERVAL 60000
#define DEBUG 0

#define WIFI_RETRIES 100
#define WAIT_DELAY 1000

#include <ESP8266WiFi.h>

extern "C" {
#include "user_interface.h"  // Required for wifi_station_connect() to work
}

unsigned long next_send_time = 0;

volatile bool listening = false;     // Are we listening to the analog value
volatile bool byterecording = false; // We have seen the first bit, and are recording the current byte

volatile int threshold = 495;
volatile int dmin, dmax;

volatile byte currentByte = 0;
volatile short readBitCount = 0;
volatile byte currentFrame[6];
volatile byte previousFrame[6];
volatile short readByteCount = 0;

volatile boolean frameReady = false;
volatile boolean parity = false;
volatile boolean valid = false;
volatile boolean validFrame = false;

volatile long nextReset = 0;

volatile boolean currentValue = false;

byte currentState[15][6];
char buffer[2000];
unsigned long lastMinMaxT;


bool WiFiOn();
bool WiFiOff();
void startRecordingFrame(unsigned long now);
void startRecordingByte(unsigned long now, unsigned long currentCycle);
void stopListening(unsigned long now, int reason);
bool readBit(unsigned long now, unsigned long currentCycle, bool newValue);
void timer0_ISR (void);
void processLoopBoiler(unsigned long now);
void sendState();
boolean sendToJeedom(String url);

void startRecordingFrame(unsigned long now) {
  listening = true;
  readByteCount = 0;
  validFrame = true;
  nextReset = now + 1200;
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + CYCLE_COUNT_4800);
}

void startRecordingByte(unsigned long now, unsigned long currentCycle) {
  readBitCount = 0;
  parity = false;
  valid = true;
  byterecording = true;
  nextReset = now + 500; // whatever happens, we reset the state in 500 ms
  timer0_write(currentCycle + CYCLE_COUNT_4800 + CYCLE_COUNT_1200);
#if DEBUG
  Serial.println("Start byte");
#endif
}

void stopListening(unsigned long now, int reason) {
  byterecording = false;
  nextReset = now + 2000;
  listening = false;
  timer0_detachInterrupt();
#if DEBUG
  Serial.print("Stop listening ");
  Serial.println(reason);
#endif
}

bool readBit(unsigned long now, unsigned long currentCycle, bool newValue) {
  if (readBitCount < 8) {
    parity ^= newValue;
    if (newValue)
      bitSet(currentByte, readBitCount);
    else
      bitClear(currentByte, readBitCount);
  }

  readBitCount ++;
  if (readBitCount == 9) { // Parity bit
    if (parity != newValue) {
      // Byte is valid
      valid = false;
    }
  }
  if (readBitCount == 10) { // Stop bit
    //Serial.print("Received byte ");
    //Serial.println(valid);
    validFrame = validFrame && valid;
    if (valid) {
      currentFrame[readByteCount] = currentByte;
      readByteCount ++;

      if (readByteCount == 6) {
        // We just finished a frame
        frameReady = validFrame;
#if DEBUG
        if (!validFrame) Serial.println("Invalid frame");
#endif

        stopListening(now, valid ? 1 : 2);
      }
    }
    readBitCount = 0;
    byterecording = false;
    return true;
  }
  return false;
}

void timer0_ISR (void) {
  unsigned long currentCycle = ESP.getCycleCount();
  int analogValue = analogRead(A0);
  unsigned long now = millis();
  if (analogValue < dmin) dmin = analogValue;
  if (analogValue > dmax) dmax = analogValue;

  if (nextReset > 0 && now > nextReset) {
    stopListening(now, 0);
    return;
  }

  boolean oldValue = currentValue;
  boolean newValue = analogValue <= threshold;
  currentValue = newValue;
  if (byterecording) {
    bool byteRead = readBit(now, currentCycle, newValue);
    if (byteRead) {
      if (! listening) return; // We have read a frame, don't rearm
      timer0_write(currentCycle + CYCLE_COUNT_4800); // Wait for next start bit
    } else {
      timer0_write(currentCycle + CYCLE_COUNT_1200); // Wait for next at 1200 bauds
    }
  } else {
    boolean downTransition = oldValue && !newValue;
    if (downTransition) {
      // We have seen the fist bit, switch to byte recording mode
      startRecordingByte(now, currentCycle);
    } else {
      timer0_write(currentCycle + CYCLE_COUNT_4800); // Nothing happened, wait one more time
    }
  }
}


void setup() {
  Serial.begin(57600);
  pinMode(LED_BUILTIN, OUTPUT);
  noInterrupts();
  timer0_isr_init();
  dmin = 1024;
  dmax = 0;
  interrupts();
  next_send_time = millis() + SEND_TO_JEEDOM_INTERVAL;
  Serial.println("Ready");
}

bool WiFiOn() {
  Serial.println("Wifi on");
  int conn_tries = 0;
  WiFi.forceSleepWake();
  WiFi.mode(WIFI_STA);
  Serial.println("Wifi connect");
  wifi_station_connect();
  Serial.println("Wifi begin");
  ESP.wdtFeed();
  WiFi.begin(ssid, password);

  while ((WiFi.status() != WL_CONNECTED) && (conn_tries++ < WIFI_RETRIES)) {
    delay(100);
    ESP.wdtFeed();
#ifdef DEBUG
    Serial.print(".");
#endif
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
    return (true);
  } else {
    Serial.println("Could not connect to WiFi");
    return (false);
  }
}


bool WiFiOff() {
  int conn_tries = 0;
  Serial.println("Disconnect");
  WiFi.disconnect();
  Serial.println("Mode off");
  WiFi.mode(WIFI_OFF);
  Serial.println("forceSleepBegin");
  //WiFi.forceSleepBegin();
  Serial.println("Wait");
  while ((WiFi.status() == WL_CONNECTED) && (conn_tries++ < 10)) {
    delay(100);
    ESP.wdtFeed();
#ifdef DEBUG
    Serial.print(".");
#endif
  }
  if (WiFi.status() != WL_CONNECTED)
    return (true);
  else
    return (false);
}

void processLoopBoiler(unsigned long now) {
  if (listening) return;


  if ((now - lastMinMaxT) > 2000) {
    lastMinMaxT = now;

    if ((dmax - dmin) > 2 ) {
      int newThreshold = dmin + (dmax - dmin) / 2;
      int diff = newThreshold - threshold;
      newThreshold = (int) (threshold + diff / 2);
      if (newThreshold != threshold) {
#if DEBUG == 1
        sprintf (buffer, "Threshold %d => %d (%d, %d)", threshold, newThreshold, dmin, dmax);
        Serial.println(buffer);
#endif
        threshold = newThreshold;
      }
    } else {
#if DEBUG == 1
      sprintf (buffer, "(%d, %d)", dmin, dmax);
      Serial.println(buffer);
#endif
    }

    dmin = 1024;
    dmax = 0;
  }

  if (frameReady) {
    bool frameEqual = true;
    for (int i = 0; i < 6; ++i) {
      frameEqual = frameEqual && currentFrame[i] == previousFrame[i];
      previousFrame[i] = currentFrame[i];
    }
    if (frameEqual && currentFrame[0] < 15) {
      for (int i = 0; i < 6; ++i)
        currentState[currentFrame[0]][i] = currentFrame[i];

#if DEBUG
      for (int frameId = 1; frameId < 13; frameId += 2) {
        for (int i = 0; i < 6; ++i) {
          sprintf(buffer, "%02x", currentState[frameId][i]);
          Serial.print(buffer);
        }
        Serial.print(" - ");
      }
      Serial.println();
#else
      Serial.print("New frame ");
      Serial.println(currentFrame[0]);
#endif
    }

    frameReady = false;
  }

  //interrupts();
  startRecordingFrame(now);
}

void loop() {
  wdt_disable();
  unsigned long now = millis();
  if (now > next_send_time) {
    digitalWrite(LED_BUILTIN, HIGH);
    sendState();
    digitalWrite(LED_BUILTIN, LOW);
    next_send_time = now + SEND_TO_JEEDOM_INTERVAL;
  }

  processLoopBoiler(now);

  wdt_enable(1000);
}

boolean sendToJeedom(String url) {
  WiFiClient client;
  ESP.wdtFeed();
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return false;
  }
  ESP.wdtFeed();

#if DEBUG
  Serial.print("Requesting URL: ");
  Serial.println(url);
#endif

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    ESP.wdtFeed();
    if (millis() - timeout > 3000) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return false;
    }
  }

  // Read all the lines of the reply from server and print them to Serial
  while (client.available()) {
    String line = client.readStringUntil('\r');
#if DEBUG
    Serial.print(line);
#endif
    ESP.wdtFeed();
  }
}

void sendState() {
  delay(WAIT_DELAY);
  WiFiOn();
  uint8_t alarm = currentState[7][1];
  uint8_t status = currentState[5][1];
  uint16_t water = (((uint16_t)currentState[1][2]) << 8) + currentState[1][3];
  uint16_t heating = (((uint16_t)currentState[1][4]) << 8) + currentState[1][5];
  uint16_t blue = (((uint16_t)currentState[3][1]) << 8) + currentState[3][2];
  uint16_t red = (((uint16_t)currentState[7][2]) << 8) + currentState[7][3];

  String url = baseurl + "&id=240&value=" + alarm;
  sendToJeedom(url);

  url = baseurl + "&id=241&value=" + status;
  sendToJeedom(url);

  url = baseurl + "&id=242&value=" + water;
  sendToJeedom(url);

  url = baseurl + "&id=243&value=" + heating;
  sendToJeedom(url);

  url = baseurl + "&id=244&value=" + blue;
  sendToJeedom(url);

  url = baseurl + "&id=245&value=" + red;
  sendToJeedom(url);

  int length = 0;
  for (int frameId = 1; frameId < 13; frameId += 2) {
    for (int i = 0; i < 6; ++i) {
      length += sprintf(buffer + length, "%02x", currentState[frameId][i]);
    }
    length += sprintf(buffer + length, "-");
  }
  url = baseurl + "&id=246&value=" + String(buffer);
  sendToJeedom(url);

#if DEBUG
  Serial.println(buffer);
  Serial.print("Water: ");
  Serial.print(currentState[1][2]);
  Serial.print(" - ");
  Serial.print(currentState[1][3]);
  Serial.print(" - ");
  Serial.println(water);
  Serial.print("Heating: ");
  Serial.println(heating);
#endif

  delay(WAIT_DELAY);
  WiFiOff();
  delay(WAIT_DELAY);
}

