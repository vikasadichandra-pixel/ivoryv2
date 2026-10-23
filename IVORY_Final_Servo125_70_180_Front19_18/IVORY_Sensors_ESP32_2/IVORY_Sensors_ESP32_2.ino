#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <esp_arduino_version.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const char* WIFI_SSID = "Huehuelessgoo";
const char* WIFI_PASSWORD = "123456789";

// Left RPWM, left LPWM, right RPWM, right LPWM.
constexpr uint8_t MOTOR_PINS[] = {25, 26, 27, 14};
constexpr uint8_t ENABLE_PIN = 33;

constexpr int LEFT_POLARITY = -1;
constexpr int RIGHT_POLARITY = 1;

constexpr uint8_t SPEED = 48;       // Duty: 0–255.
constexpr uint32_t PWM_HZ = 20000;
constexpr uint32_t CHANGE_PAUSE_MS = 200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 350;
constexpr uint32_t SENSOR_TIMEOUT_MS = 1200;

HardwareSerial sensorLink(2);
WebServer server(80);

bool pwmReady = false;
bool armed = false;
char wanted = 'x';
char applied = 'x';
uint32_t changedAt = 0;
uint32_t lastCommandAt = 0;
uint64_t lastSequence = 0;

// Sensor values are informational, not motor interlocks.
char motion[8] = "WAITING";
double vibration = 0, rotation = 0;
double scanCm = -1, frontCm = -1;
double latitude = 0, longitude = 0;
int scanAngle = 70, gpsFix = 0, satellites = 0;
bool haveSensors = false;
uint32_t sensorReceivedAt = 0;

char sensorLine[224];
size_t sensorLength = 0;
bool discardLine = false;

void writePWM(uint8_t channel, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(MOTOR_PINS[channel], duty);
#else
  ledcWrite(channel, duty);
#endif
}

void stopOutputs() {
  digitalWrite(ENABLE_PIN, LOW);

  if (pwmReady) {
    for (uint8_t i = 0; i < 4; ++i) writePWM(i, 0);
  }

  applied = 'x';
}

void stopVehicle() {
  armed = false;
  wanted = 'x';
  stopOutputs();
}

void setSide(uint8_t firstChannel, int value) {
  writePWM(firstChannel, value > 0 ? value : 0);
  writePWM(firstChannel + 1, value < 0 ? -value : 0);
}

void applyMovement(char key) {
  int left = 0;
  int right = 0;

  switch (key) {
    case 'w': left = SPEED; right = SPEED; break;
    case 's': left = -SPEED; right = -SPEED; break;
    case 'a': right = SPEED; break;
    case 'd': left = SPEED; break;
    default: stopOutputs(); return;
  }

  digitalWrite(ENABLE_PIN, LOW);
  setSide(0, left * LEFT_POLARITY);
  setSide(2, right * RIGHT_POLARITY);
  digitalWrite(ENABLE_PIN, HIGH);
  applied = key;
}

void checkCommandTimeout() {
  if (WiFi.status() != WL_CONNECTED ||
      (armed &&
       uint32_t(millis() - lastCommandAt) >= COMMAND_TIMEOUT_MS)) {
    stopVehicle();
  }
}

bool validDistance(double cm) {
  return isfinite(cm) && (cm == -1 || (cm >= 2 && cm <= 400));
}

void parseSensors(const char* line) {
  char state[8] = {};
  double vib, gyro, scan, front, lat, lon;
  int angle, fix, sats, consumed = -1;

  const int fields = sscanf(
    line,
    "S,%7[^,],%lf,%lf,%lf,%lf,%d,%d,%lf,%lf,%d%n",
    state, &vib, &gyro, &scan, &front,
    &angle, &fix, &lat, &lon, &sats, &consumed
  );

  if (fields != 11 || consumed < 0 || line[consumed] != '\0')
    return;

  if (strcmp(state, "ERROR") &&
      strcmp(state, "MOVING") &&
      strcmp(state, "STOPPED")) return;

  if (!isfinite(vib) || !isfinite(gyro) ||
      vib < 0 || gyro < 0 ||
      !validDistance(scan) || !validDistance(front) ||
      angle < 0 || angle > 180 ||
      (fix != 0 && fix != 1) ||
      !isfinite(lat) || !isfinite(lon) ||
      lat < -90 || lat > 90 || lon < -180 || lon > 180 ||
      sats < 0 || sats > 100) return;

  strcpy(motion, state);
  vibration = vib;
  rotation = gyro;
  scanCm = scan;
  frontCm = front;
  scanAngle = angle;
  gpsFix = fix;
  latitude = lat;
  longitude = lon;
  satellites = sats;

  haveSensors = true;
  sensorReceivedAt = millis();
}

void receiveSensors() {
  // Bounded work so UART traffic cannot starve motor timeout checks.
  for (int budget = 0; budget < 256 && sensorLink.available(); ++budget) {
    const char c = char(sensorLink.read());

    if (c == '\r') continue;

    if (c == '\n') {
      if (!discardLine && sensorLength > 0) {
        sensorLine[sensorLength] = '\0';
        parseSensors(sensorLine);
      }
      sensorLength = 0;
      discardLine = false;
      continue;
    }

    if (c < 32 || c > 126) {
      discardLine = true;
      continue;
    }

    if (!discardLine && sensorLength < sizeof(sensorLine) - 1)
      sensorLine[sensorLength++] = c;
    else
      discardLine = true;
  }
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>IVORY WASD</title>
<style>
body{background:#15191d;color:white;font:18px system-ui;
max-width:850px;margin:auto;padding:25px}
button{background:#d32f2f;color:white;border:0;border-radius:8px;
padding:18px 45px;font-size:24px;cursor:pointer}
pre{background:#242b32;padding:18px;white-space:pre-wrap}
#movement{font-size:28px;margin:20px 0}
</style>
</head>
<body>
<h1>IVORY WASD Control</h1>
<p>Hold W: forward · S: reverse · A: left · D: right.</p>
<p>Release to stop. Space / Escape also stops.</p>
<div id="movement">STOPPED</div>
<button id="stop">STOP</button>
<h2>ESP32-2 sensors</h2>
<p>Readings only. No automatic obstacle or tilt stopping.</p>
<pre id="sensors">Waiting for ESP32-2…</pre>
<script>
let key = 'x';
let sequence = Date.now() * 1000;

const movement = document.getElementById('movement');
const sensors = document.getElementById('sensors');
const labels = {
  w:'FORWARD', s:'REVERSE', a:'LEFT', d:'RIGHT', x:'STOPPED'
};

async function send(press = false) {
  const n = ++sequence;
  const sentKey = key;

  try {
    const response = await fetch(
      '/cmd?key=' + sentKey + '&n=' + n + '&press=' + (press ? 1 : 0),
      {cache:'no-store', keepalive:true}
    );
    if (!response.ok) throw new Error(await response.text());
  } catch (error) {
    if (n === sequence) {
      key = 'x';
      movement.textContent = 'STOPPED — release key and press again';
    }
  }
}

function choose(next, press = false) {
  key = next;
  movement.textContent = labels[key];
  send(press);
}

function stop() {
  choose('x');
}

document.addEventListener('keydown', event => {
  const k = event.key.toLowerCase();

  if (['w','a','s','d'].includes(k)) {
    event.preventDefault();
    if (!event.repeat) choose(k, true);
  } else if (event.code === 'Space' || event.key === 'Escape') {
    event.preventDefault();
    stop();
  }
});

document.addEventListener('keyup', event => {
  if (event.key.toLowerCase() === key) stop();
});

document.getElementById('stop').onclick = stop;
window.addEventListener('blur', stop);
window.addEventListener('pagehide', stop);
document.addEventListener('visibilitychange', () => {
  if (document.hidden) stop();
});

setInterval(() => {
  if (key !== 'x' && !document.hidden) send(false);
}, 100);

async function poll() {
  try {
    const response = await fetch('/state', {cache:'no-store'});
    if (!response.ok) throw new Error('Status unavailable');
    const s = await response.json();

    sensors.textContent =
      (s.fresh ? 'LIVE' : 'STALE / NO SENSOR DATA') +
      '\nSensor age: ' + s.age_ms + ' ms' +
      '\nMPU motion: ' + s.motion +
      '\nVibration: ' + s.vibration_g + ' g' +
      '\nRotation magnitude: ' + s.rotation_dps + ' deg/s' +
      '\nScanner: ' + s.scan_cm + ' cm at ' + s.scan_angle + ' deg' +
      '\nFixed front sensor: ' + s.front_cm + ' cm' +
      '\nGPS fix: ' + s.gps_fix +
      '\nLatitude: ' + s.latitude +
      '\nLongitude: ' + s.longitude +
      '\nSatellites: ' + s.satellites +
      '\n(-1 cm means unavailable)';
  } catch (error) {
    sensors.textContent = 'ESP32-1 CONNECTION LOST';
    key = 'x';
    movement.textContent = 'STOPPED — connection lost';
    send(false);
  }

  setTimeout(poll, 500);
}
poll();
</script>
</body>
</html>
)HTML";

void handleCommand() {
  checkCommandTimeout();

  const String value = server.arg("key");
  const String number = server.arg("n");

  if (value.length() != 1 ||
      number.length() == 0 || number.length() > 16) {
    server.send(400, "text/plain", "Invalid command");
    return;
  }

  const char key = value[0];
  if (key != 'w' && key != 'a' && key != 's' &&
      key != 'd' && key != 'x') {
    server.send(400, "text/plain", "Unknown key");
    return;
  }

  for (size_t i = 0; i < number.length(); ++i) {
    if (number[i] < '0' || number[i] > '9') {
      server.send(400, "text/plain", "Invalid sequence");
      return;
    }
  }

  const uint64_t sequence = strtoull(number.c_str(), nullptr, 10);

  if (sequence <= lastSequence) {
    server.send(409, "text/plain", "Old command");
    return;
  }
  lastSequence = sequence;

  if (key == 'x') {
    stopVehicle();
    server.send(200, "text/plain", "STOPPED");
    return;
  }

  if (!pwmReady || WiFi.status() != WL_CONNECTED) {
    stopVehicle();
    server.send(503, "text/plain", "Motor output unavailable");
    return;
  }

  // After timeout, a heartbeat cannot restart motion.
  // A fresh physical keypress is required.
  if (!armed && server.arg("press") != "1") {
    server.send(409, "text/plain", "Release key and press again");
    return;
  }

  armed = true;
  lastCommandAt = millis();

  if (key != wanted) {
    stopOutputs();
    wanted = key;
    changedAt = millis();
  }

  server.send(200, "text/plain", "OK");
}

void handleState() {
  const uint32_t age = haveSensors
    ? uint32_t(millis() - sensorReceivedAt) : 0;

  const bool fresh = haveSensors && age < SENSOR_TIMEOUT_MS;

  char json[640];
  snprintf(
    json, sizeof(json),
    "{\"fresh\":%s,\"age_ms\":%lu,"
    "\"motion\":\"%s\",\"vibration_g\":%.3f,"
    "\"rotation_dps\":%.1f,\"scan_cm\":%.1f,"
    "\"front_cm\":%.1f,\"scan_angle\":%d,"
    "\"gps_fix\":%s,\"latitude\":%.6f,"
    "\"longitude\":%.6f,\"satellites\":%d}",
    fresh ? "true" : "false",
    (unsigned long)age,
    motion, vibration, rotation, scanCm, frontCm, scanAngle,
    (fresh && gpsFix) ? "true" : "false",
    latitude, longitude, satellites
  );

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void setup() {
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(ENABLE_PIN, OUTPUT);

  for (uint8_t pin : MOTOR_PINS) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }

  Serial.begin(115200);

  bool ok = true;
  for (uint8_t i = 0; i < 4; ++i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    if (!ledcAttach(MOTOR_PINS[i], PWM_HZ, 8)) ok = false;
#else
    if (ledcSetup(i, PWM_HZ, 8) <= 0) ok = false;
    ledcAttachPin(MOTOR_PINS[i], i);
#endif
    writePWM(i, 0);
  }

  pwmReady = ok;
  stopVehicle();

  if (!pwmReady)
    Serial.println("PWM setup failed: motors disabled.");

  sensorLink.setRxBufferSize(1024);
  sensorLink.begin(115200, SERIAL_8N1, 16, 17);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nOpen on laptop: http://");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html", PAGE);
  });

  server.on("/cmd", HTTP_GET, handleCommand);
  server.on("/state", HTTP_GET, handleState);
  server.begin();
}

void loop() {
  checkCommandTimeout();
  receiveSensors();

  if (pwmReady && armed && wanted != 'x' &&
      applied != wanted &&
      uint32_t(millis() - changedAt) >= CHANGE_PAUSE_MS) {
    applyMovement(wanted);
  }

  server.handleClient();
  checkCommandTimeout();
}
