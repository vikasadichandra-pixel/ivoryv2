IVORY — FINAL SERVO AND FRONT ULTRASONIC UPDATE
===============================================

ESP32-1 MOTOR / Wi-Fi DASHBOARD — UNCHANGED from the previous final package.
  LEFT BTS7960 RPWM GPIO25, LPWM GPIO26
  RIGHT BTS7960 RPWM GPIO27, LPWM GPIO14
  BOTH driver enable pins GPIO33
  UART: RX GPIO16 from ESP32-2 TX17; TX GPIO17 to ESP32-2 RX16
  Wi-Fi: IVORY-ROVER / ivory1234 / http://192.168.4.1

ESP32-2 SENSOR / SERVO PINOUT
  SG90 servo signal GPIO25
  SCANNING front ultrasonic TRIG GPIO32, ECHO GPIO35 (via resistor divider)
  FIXED FRONT ultrasonic TRIG GPIO19, ECHO GPIO18 (via resistor divider)
  MPU6050 SDA GPIO21, SCL GPIO22, address 0x68
  GPS TX -> ESP32-2 RX GPIO34 at 9600 baud, RX unconnected
  UART RX GPIO16 <- ESP32-1 TX17; UART TX GPIO17 -> ESP32-1 RX16
  All GND returns share an appropriate reference.

SERVO MOVEMENT (user's cosine-easing smoothMove pattern, now nonblocking)
  Boot: write 125 degrees; wait 1500 ms.
  Smoothly sweep 125 -> 70 degrees over 500 ms.
  Continuous: 70 -> 180 over 900 ms, wait 100 ms,
              180 -> 70 over 900 ms, wait 100 ms; repeat forever.
  Timing resolution is ~15 ms; sensor polling can cause slight timing jitter.

INSTALL ESP32-2 LIBRARIES in Arduino IDE Library Manager
  ESP32Servo and TinyGPSPlus (TinyGPS++).
  Wire and HardwareSerial are ESP32 board core libraries.

IMPORTANT: Ultrasonic ECHO pins are 5V without a divider; use level
shifting / resistor divider to below 3.3V for ESP32 GPIO18 and GPIO35.
BOTH HC-SR04 need verified regulated 5V. The SG90 must have a suitable
separate regulated 5V supply and shared ground. Check the 180-degree servo
mechanical stop with power removable and no interference. Disconnect power
before rewiring. Motor current must NOT flow through ESP32 ground wiring.

Do not confuse ESP32-1 GPIO25 (motor driver) with ESP32-2 GPIO25 (servo).
For the requested update, upload ONLY the sensor sketch to ESP32-2.
The matching motor sketch is included solely for completeness.

Telemetry on UART at 115200 (unchanged):
S,STATE,vibration_g,gyro_dps,scan_cm,front_cm,scan_angle,gps_fix,lat,lon,sats

GPS might show NO GPS FIX indoors. Sonar and IMU only display information;
there is NO automatic motor stopping on obstacle proximity.
