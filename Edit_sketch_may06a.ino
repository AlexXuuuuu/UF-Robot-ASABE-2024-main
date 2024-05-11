#include <RPLidar.h>

#define RPLIDAR_MOTOR 3  // The PWM pin for the speed control of RPLIDAR's motor

RPLidar lidar;

void setup() {
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 255); // Start the RPLIDAR motor

  Serial.begin(115200); // Start serial communication at 115200 baud rate for debugging
  Serial2.begin(115200); // Initialize Serial2 at the baud rate required by your LIDAR
  lidar.begin(Serial2); // Begin the LIDAR on Serial2
  
  if (IS_OK(lidar.startScan())) {  // Start scanning
    Serial.println("LIDAR Scan Started...");
  } else {
    Serial.println("Error starting LIDAR scan!");
  }
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    // Data is read correctly
    float distance = lidar.getCurrentPoint().distance; // distance in mm
    float angle = lidar.getCurrentPoint().angle; // angle in degrees
    bool startBit = lidar.getCurrentPoint().startBit; // new scan indicator
    byte quality = lidar.getCurrentPoint().quality; // measurement quality

    // Print the read values to the serial monitor
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" degrees, Distance: ");
    Serial.print(distance);
    Serial.print(" mm, Quality: ");
    Serial.println(quality);
  } else {
    // No point ready, but avoid restarting too quickly
    static unsigned long lastRestartAttempt = 0;
    unsigned long currentMillis = millis();
    if (currentMillis - lastRestartAttempt > 5000) { // 5 seconds delay before trying to restart
      lastRestartAttempt = currentMillis;
      
      rplidar_response_device_info_t info;
      if (IS_OK(lidar.getDeviceInfo(info, 100))) {
        Serial.println("LIDAR Detected, restarting scan...");
        lidar.startScan();
        analogWrite(RPLIDAR_MOTOR, 255); // Restart motor
      } else {
        Serial.println("Failed to communicate with LIDAR.");
      }
    }
  }
}
