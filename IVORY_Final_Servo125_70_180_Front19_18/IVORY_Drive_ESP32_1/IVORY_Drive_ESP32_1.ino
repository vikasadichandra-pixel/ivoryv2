#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <string.h>

// IVORY ESP32-1: WASD + BTS7960 + LIVE SENSOR DASHBOARD
// Original motor pins from wiring PDF; do not move motor wires.
#define L_RPWM 25
#define L_LPWM 26
#define R_RPWM 27
#define R_LPWM 14
#define MOTOR_EN 33
const bool INVERT_LEFT = true;   // Previous drive-direction correction
const bool INVERT_RIGHT = false; // Change only after wheels-up check
const int DRIVE_PWM = 130;       // Full duty: use less for initial testing
const unsigned long DIRECTION_PAUSE_MS = 100;
const unsigned long COMMAND_TIMEOUT_MS = 400;
const unsigned long SENSOR_TIMEOUT_MS = 1500;

const char* AP_NAME = "IVORY-ROVER";
const char* AP_PASSWORD = "ivory1234";

WebServer web(80);
HardwareSerial linkToSensors(2); // RX GPIO16, TX GPIO17

char command = 'x';
bool motorActive = false;
unsigned long commandUpdatedAt = 0, directionChangedAt = 0;
unsigned long telemetryReceivedAt = 0;
char imuState[12] = "NO DATA";
float vibG = 0, gyroDps = 0, scanCm = -1, frontCm = -1;
float gpsLat = 0, gpsLon = 0;
int scanAngle = 90, gpsFix = 0, satellites = 0;
char rxLine[170];
size_t rxUsed = 0;

void stopMotorOutputs() {
  digitalWrite(MOTOR_EN, LOW);
  analogWrite(L_RPWM, 0); analogWrite(L_LPWM, 0);
  analogWrite(R_RPWM, 0); analogWrite(R_LPWM, 0);
  motorActive = false;
}

void applySide(int rpwm, int lpwm, int direction, bool invert) {
  if (invert) direction = -direction;
  analogWrite(rpwm, direction > 0 ? DRIVE_PWM : 0);
  analogWrite(lpwm, direction < 0 ? DRIVE_PWM : 0);
}

void acceptCommand(char next) {
  commandUpdatedAt = millis();
  if (next == 'x') {
    stopMotorOutputs();
    command = 'x';
    return;
  }
  if (next == command) return; // Heartbeats do not restart the motors
  stopMotorOutputs();         // Pause before every changed drive command
  command = next;
  directionChangedAt = millis();
  Serial.printf("WASD: %c\n", command);
}

void updateMotorOutputs() {
  if (command == 'x' || motorActive ||
      millis() - directionChangedAt < DIRECTION_PAUSE_MS) return;

  int left = 0, right = 0;
  switch (command) {
    case 'w': left =  1; right =  1; break; // Forward, both sides
    case 's': left = -1; right = -1; break; // Reverse, both sides
    case 'a': left =  0; right =  1; break; // LEFT: right side forward only
    case 'd': left =  1; right =  0; break; // RIGHT: left side forward only
    default: stopMotorOutputs(); command = 'x'; return;
  }
  applySide(L_RPWM, L_LPWM, left, INVERT_LEFT);
  applySide(R_RPWM, R_LPWM, right, INVERT_RIGHT);
  digitalWrite(MOTOR_EN, HIGH);
  motorActive = true;
}

// Matching ESP32-2 line:
// S,STATE,vibration,rotation,scan_cm,front_cm,angle,gps_fix,lat,lon,satellites
void receiveTelemetry() {
  while (linkToSensors.available()) {
    char c = (char)linkToSensors.read();
    if (c == '\n') {
      rxLine[rxUsed] = '\0';
      char state[12] = "";
      float v, r, scan, front, lat, lon;
      int angle, fix, sats;
      int count = sscanf(rxLine,
        "S,%11[^,],%f,%f,%f,%f,%d,%d,%f,%f,%d",
        state, &v, &r, &scan, &front,
        &angle, &fix, &lat, &lon, &sats);
      if (count == 10 &&
          (!strcmp(state, "MOVING") || !strcmp(state, "STOPPED") ||
           !strcmp(state, "ERROR"))) {
        strcpy(imuState, state);
        vibG = v; gyroDps = r;
        scanCm = scan; frontCm = front;
        scanAngle = angle; gpsFix = fix;
        gpsLat = lat; gpsLon = lon; satellites = sats;
        telemetryReceivedAt = millis();
      }
      rxUsed = 0;
    } else if (c != '\r') {
      if (rxUsed < sizeof(rxLine) - 1) rxLine[rxUsed++] = c;
      else rxUsed = 0;
    }
  }
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta charset="utf-8"><title>IVORY ROVER</title>
<style>
body{font-family:Arial,sans-serif;text-align:center;background:#0f172a;color:#f8fafc;margin:14px}
h1{color:#38bdf8} .card{background:#1e293b;max-width:490px;margin:12px auto;padding:16px;border-radius:12px}
.key{display:inline-block;width:60px;padding:16px 0;margin:4px;border-radius:10px;background:#334155;font-size:26px;user-select:none}
.val{font-size:22px;margin:12px 0} .muted{color:#cbd5e1;font-size:13px}
button{background:#dc2626;border:0;color:#fff;font-size:22px;padding:14px 38px;border-radius:8px}
</style></head><body>
<h1>IVORY ROVER</h1>
<div class="card"><h3>WASD driving</h3>
<div class="key">W</div><br><div class="key">A</div><div class="key">S</div><div class="key">D</div>
<div class="val" id="drive">STOPPED</div><button onclick="stopNow()">STOP</button>
<p class="muted">W/S: both sides. A: right wheels only. D: left wheels only.<br>Active side PWM 255. Release key/Space: stop.</p>
</div>
<div class="card"><h3>MPU6050</h3><div class="val" id="imu">NO SENSOR DATA</div>
<div id="imuValues">Vibration: -- | Rotation: --</div>
<p class="muted">Moving/stopped is based on vibration and gyro motion, not wheel speed.</p></div>
<div class="card"><h3>Two FRONT ultrasonics</h3>
<div class="val">Servo scanner: <span id="scan">--</span></div>
<div>Servo angle: <span id="angle">--</span></div>
<div class="val">Fixed front: <span id="front">--</span></div>
<p class="muted">NO ECHO means no valid measurement. No automatic collision stop is enabled.</p></div>
<div class="card"><h3>NEO-6M GPS</h3><div class="val" id="gps">WAITING FOR FIX</div>
<div id="position">Latitude: -- | Longitude: --</div>
<div>Satellites: <span id="sats">--</span></div>
<p class="muted">A GPS fix generally requires outdoor sky visibility.</p></div>
<script>
let keys=[], current='x', inhibited=false;
const labels={w:'FORWARD',s:'BACKWARD',a:'LEFT (RIGHT SIDE)',d:'RIGHT (LEFT SIDE)',x:'STOPPED'};
const arrows={arrowup:'w',arrowleft:'a',arrowdown:'s',arrowright:'d'};
function mapped(k){k=k.toLowerCase();return arrows[k]||k;}
function send(k){fetch('/cmd?c='+k+'&t='+Date.now(),{cache:'no-store'}).catch(()=>{});}
function update(){let next=inhibited?'x':(keys.length?keys[keys.length-1]:'x');
 if(next!==current){current=next;document.getElementById('drive').textContent=labels[next];send(next);}}
function stopNow(){keys=[];inhibited=true;current='x';
 document.getElementById('drive').textContent='STOPPED';send('x');}
document.addEventListener('keydown',e=>{
 if(e.code==='Space'){e.preventDefault();stopNow();return;}
 const k=mapped(e.key);if('wasd'.includes(k)&&k.length===1){e.preventDefault();
 if(inhibited && e.repeat)return;
 if(inhibited && !e.repeat)inhibited=false;
 if(!keys.includes(k))keys.push(k);update();}});
document.addEventListener('keyup',e=>{
 const k=mapped(e.key);keys=keys.filter(x=>x!==k);
 if(keys.length===0)inhibited=false;update();});
setInterval(()=>{if(current!=='x')send(current);},120);
window.addEventListener('blur',stopNow);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopNow();});
function cm(v){return typeof v==='number' && v>=2 && v<=400?v.toFixed(1)+' cm':'NO ECHO';}
async function refresh(){try{
 const response=await fetch('/status?t='+Date.now(),{cache:'no-store'});
 const d=await response.json();
 if(!d.connected){document.getElementById('imu').textContent='NO SENSOR DATA';
  document.getElementById('scan').textContent='NO SENSOR DATA';
  document.getElementById('front').textContent='NO SENSOR DATA';
  document.getElementById('gps').textContent='NO SENSOR DATA';return;}
 document.getElementById('imu').textContent=d.imu==='MOVING'?'VEHICLE MOVING':
  d.imu==='STOPPED'?'VEHICLE STOPPED (LOW VIBRATION)':'MPU6050 ERROR';
 document.getElementById('imuValues').textContent='Vibration: '+d.vib.toFixed(3)+
  ' g | Rotation: '+d.rot.toFixed(1)+' deg/s';
 document.getElementById('scan').textContent=cm(d.scan);
 document.getElementById('front').textContent=cm(d.front);
 document.getElementById('angle').textContent=d.angle+' deg';
 document.getElementById('gps').textContent=d.fix?'GPS FIX':'NO GPS FIX';
 document.getElementById('position').textContent=d.fix?
   'Latitude: '+d.lat.toFixed(6)+' | Longitude: '+d.lon.toFixed(6):
   'Latitude: -- | Longitude: --';
 document.getElementById('sats').textContent=d.sats;
 }catch(e){document.getElementById('imu').textContent='CONNECTION LOST';}}
setInterval(refresh,400);refresh();
</script></body></html>
)HTML";

void setup() {
  pinMode(MOTOR_EN, OUTPUT); digitalWrite(MOTOR_EN, LOW);
  pinMode(L_RPWM, OUTPUT); pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT); pinMode(R_LPWM, OUTPUT);
  stopMotorOutputs();
  Serial.begin(115200);
  linkToSensors.begin(115200, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_NAME, AP_PASSWORD);
  web.on("/", [](){web.send_P(200,"text/html",PAGE);});
  web.on("/cmd", [](){
    String v = web.arg("c");
    if(v.length()!=1 || String("wasdx").indexOf(v[0])<0){
      stopMotorOutputs();command='x';web.send(400,"text/plain","INVALID");return;
    }
    acceptCommand(v[0]);
    web.send(200,"text/plain","OK");
  });
  web.on("/status", [](){
    bool fresh = telemetryReceivedAt != 0 &&
      millis()-telemetryReceivedAt < SENSOR_TIMEOUT_MS;
    String json = "{\"connected\":"+String(fresh?"true":"false")+
      ",\"imu\":\""+String(fresh?imuState:"NO DATA")+"\""+
      ",\"vib\":"+String(vibG,3)+
      ",\"rot\":"+String(gyroDps,1)+
      ",\"scan\":"+String(fresh?scanCm:-1.0f,1)+
      ",\"front\":"+String(fresh?frontCm:-1.0f,1)+
      ",\"angle\":"+String(scanAngle)+
      ",\"fix\":"+String(fresh?gpsFix:0)+
      ",\"lat\":"+String(gpsLat,6)+
      ",\"lon\":"+String(gpsLon,6)+
      ",\"sats\":"+String(satellites)+"}";
    web.send(200,"application/json",json);
  });
  web.begin();
  Serial.println("IVORY-ROVER / ivory1234 / http://192.168.4.1");
}

void loop() {
  receiveTelemetry();
  web.handleClient();
  if (command!='x' && millis()-commandUpdatedAt>COMMAND_TIMEOUT_MS) {
    stopMotorOutputs();command='x';Serial.println("CONTROL TIMEOUT: STOP");
  }
  updateMotorOutputs();
}
