/*
ArduinoRover.ino
October 25, 2022
Lukas Severinghaus, Carlos Chacon, Miguel Chacon, Salsabil Soliman

Receives TCP commands from the computer, and translates that to motor and servo movements.
Smoothes the servo movement.
Designed to run on the Arduino Engineering Kit rover for the EGR455 team project.
Our rover was assigned a static IP address in the router, so we don't worry about setting a static IP here.
*/

#include <WiFiNINA.h>
#include <ArduinoMotorCarrier.h>
#include <SPI.h>
char ssid[] = "XXXXXXXX";// Network SSID
char pass[] = "XXXXXXXX";   // Network password
int status = WL_IDLE_STATUS;

// Fork limit positions
#define FORK_MIN 128
#define FORK_MAX 180

// Is a new fork position available
bool fork_new = false;

// Union for converting from bytes to double
union {
  double num;
  byte array[sizeof(double)];
} doubleConvert;

// Wifi servers for the different listeners
WiFiServer vel_server(25000);
WiFiServer ang_vel_server(25002);
WiFiServer fork_server(25001);

// Setup, run once on start
void setup() {
  Serial.begin(115200);
  controller.begin();
  controller.reboot();
  delay(500);
  Serial.println("Controller online");

  int status = WiFi.begin(ssid, pass);
  if (status != WL_CONNECTED) {
    Serial.println("Couldn't get a WiFi connection");
    while(true);
  }
  else {
    vel_server.begin();
    ang_vel_server.begin();
    fork_server.begin();
    Serial.print("Connected to WiFi. My address: ");
    IPAddress myAddress = WiFi.localIP();
    Serial.println(myAddress);
  }
  
  Serial.println("Start");

  // Move the servo to indicate it's ready to go
  setServo(FORK_MAX);
  delay(500);
  setServo(FORK_MIN);
  delay(500);
}

// Target and current servo positions, for smoothing
float servo_target = FORK_MIN;
float current_servo = FORK_MIN;

// Main event loop, run continuously
void loop() {
  // Poll the TCP servers
  float vel = get_vel();
  float ang_vel = get_ang_vel();
  servo_target = get_fork();

  // If a new fork position available
  if(fork_new){
    // Perform blocking fixed velocity smoothing
    while(current_servo != servo_target){
      if(current_servo < servo_target){
        current_servo += 1;
      }else if (current_servo > servo_target){
        current_servo -= 1;
      }
      Serial.print("Moving ");
      Serial.println(current_servo);
      setServo(current_servo);
      delay(20);
    }
    Serial.println("Done moving");
    fork_new = false;
  }

  // Drive the wheels based on the linear and angular velocities
  vels_drive_wheels(vel, ang_vel);
}

// **** Hardware Control **** //
void setServo(float dec){
  //Limit the servo position to the max/min constraints
  dec = max(FORK_MIN, min(dec, FORK_MAX));
  servo3.setAngle(dec);
}


void vels_drive_wheels(float lin_v, float ang_v){
  // Derived from the Simulink model matrix math
  float wheel_l = (2*lin_v)/9 - (17*ang_v)/18;
  float wheel_r = (17*ang_v)/18 + (2*lin_v)/9;
  
  driveMotors(wheel_l * 10, wheel_r * 10);
}



void driveMotors(int l, int r){
  // Reverse the right motor as it's oriented opposite.
  M1.setDuty(-r);
  M2.setDuty(l);
}

// **** End Hardware Control **** //


// **** TCP Parsing **** //

/*
Theory of operation:
    The listener functions check for a new TCP client every time they're called.
    If enough bytes are available, they're parsed as a double, little endian.

    The timeout is then updated to the time it was received. 

    If there's no message available, and it's been more than 500ms since the last update, 
    stop the motion. 

The servo always stays in its fixed position, it doesn't timeout to a return position.
*/

// Internal velocity
int _get_vel_vel = 0;
// Internal last time packet was received
int _get_vel_time = 0;

int get_vel(){
  WiFiClient vel_client = vel_server.available();
  if(vel_client){
    if(vel_client.available() >= sizeof(double)){
      vel_client.read(doubleConvert.array, sizeof(double));
      Serial.print("Got number: ");
      Serial.println(doubleConvert.num);
      _get_vel_vel = doubleConvert.num;
      _get_vel_time = millis();
    }
  }

  if(millis() - _get_vel_time > 500){
    _get_vel_vel = 0;
  }
  return _get_vel_vel;
}

// Internal angular velocity
int _get_ang_vel = 0;
// Internal last time packet was received
int _get_ang_time = 0;
int get_ang_vel(){
  WiFiClient ang_client = ang_vel_server.available();
  if(ang_client){
    if(ang_client.available() >= sizeof(double)){
      ang_client.read(doubleConvert.array, sizeof(double));
      Serial.print("Got angular number: ");
      Serial.println(doubleConvert.num);
      _get_ang_vel = doubleConvert.num;
      _get_ang_time = millis();
    }
  }

  if(millis() - _get_ang_time > 500){
    _get_ang_vel = 0;
  }
  return _get_ang_vel;
}

// Internal fork pos
int _get_fork_pos = 0;
double get_fork(){
  WiFiClient fork_client = fork_server.available();
  if(fork_client){
    if(fork_client.available() >= sizeof(double)){
      fork_client.read(doubleConvert.array, sizeof(double));
      Serial.print("Got fork number: ");
      Serial.println(doubleConvert.num);
      _get_fork_pos = doubleConvert.num;
      fork_new = true;
    }
  }

  return _get_fork_pos;
}
// **** End TCP Parsing **** //
