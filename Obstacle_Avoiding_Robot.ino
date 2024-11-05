#include <Servo.h>
#include <Wire.h>

// Motor control pins
#define R1 2    // Right motor forward
#define R2 3    // Right motor backward
#define L1 4    // Left motor forward
#define L2 5    // Left motor backward

// Ultrasonic sensor pins
#define trigPin 9   // Trigger pin for the ultrasonic sensor
#define echoPin 10  // Echo pin for the ultrasonic sensor

// Servo motor pin
#define servoPin 6
Servo myServo;      // Create a Servo object to control the servo motor

// Variables for ultrasonic sensor
long duration;      // Duration of the ultrasonic pulse
int distance;       // Distance to the nearest obstacle in front
int distanceRight;  // Distance to the nearest obstacle to the right
int distanceLeft;   // Distance to the nearest obstacle to the left

void setup() {
  // Set motor control pins as output
  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
  pinMode(L1, OUTPUT);
  pinMode(L2, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);  // Trigger pin as output
  pinMode(echoPin, INPUT);   // Echo pin as input

  // Attach the servo motor to its pin and set to forward position (90 degrees)
  myServo.attach(servoPin);
  myServo.write(90);  // Point servo forward

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure the distance to the nearest obstacle in front
  distance = getDistance();

  // If an obstacle is detected within 30 cm
  if (distance > 0 && distance <= 30) {
    moveStop();         // Stop the robot
    delay(500);         // Short pause
    moveBackward();     // Move backward slightly to avoid collision
    delay(200);
    moveStop();         // Stop after moving back

    // Check for alternative paths
    distanceRight = lookRight();  // Measure distance to the right
    distanceLeft = lookLeft();    // Measure distance to the left
    delay(500);                   // Short pause to stabilize measurements

    // Turn towards the direction with more space
    if (distanceRight > distanceLeft) {
      turnRight();  // Turn right if more space is available on the right
    } else {
      turnLeft();   // Turn left if more space is available on the left
    }

    delay(300);     // Pause to complete turn
    moveStop();     // Stop after turning
  } else {
    moveForward();  // Move forward if no obstacle is within 30 cm
  }

  delay(50);        // Short delay before repeating loop
}

// Function to get the distance from the ultrasonic sensor
int getDistance() {
  // Trigger the ultrasonic pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo time, with a 20 ms timeout
  duration = pulseIn(echoPin, HIGH, 20000);

  // If no echo was received within the timeout
  if (duration == 0) {
    Serial.println("No echo received");
    return 0;
  }

  // Calculate distance in cm (duration * speed of sound / 2)
  int distance = duration * 0.034 / 2;

  // Print the calculated distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.println(distance);

  return distance;  // Return measured distance
}

// Function to check distance on the right side
int lookRight() {
  myServo.write(0);       // Turn servo to look right (0 degrees)
  delay(500);             // Wait for servo to settle
  int distance = getDistance();  // Measure distance to the right
  myServo.write(90);      // Reset servo to forward position
  return distance;        // Return distance to the right
}

// Function to check distance on the left side
int lookLeft() {
  myServo.write(180);     // Turn servo to look left (180 degrees)
  delay(500);             // Wait for servo to settle
  int distance = getDistance();  // Measure distance to the left
  myServo.write(90);      // Reset servo to forward position
  return distance;        // Return distance to the left
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(R1, HIGH); // Right motor forward
  digitalWrite(R2, LOW);
  digitalWrite(L1, HIGH); // Left motor forward
  digitalWrite(L2, LOW);
}

// Function to move the robot backward
void moveBackward() {
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH); // Right motor backward
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH); // Left motor backward
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(R1, LOW);
  digitalWrite(R2, HIGH); // Right motor backward
  digitalWrite(L1, HIGH); // Left motor forward
  digitalWrite(L2, LOW);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(R1, HIGH); // Right motor forward
  digitalWrite(R2, LOW);
  digitalWrite(L1, LOW);
  digitalWrite(L2, HIGH); // Left motor backward
}

// Function to stop the robot
void moveStop() {
  digitalWrite(R1, LOW); // Stop right motor
  digitalWrite(R2, LOW);
  digitalWrite(L1, LOW); // Stop left motor
  digitalWrite(L2, LOW);
}
