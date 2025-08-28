#define IN1 4    // Motor 1 control pin 1
#define IN2 5    // Motor 1 control pin 2
#define IN3 6    // Motor 2 control pin 1
#define IN4 7    // Motor 2 control pin 2

// Ultrasonic sensor pins
#define TRIG_PIN A0
#define ECHO_PIN A1

// Speed for motors (PWM control, reduced for slow movement)
#define MOTOR_SPEED 150  // Adjusted speed for slow movement
  
// Distance threshold for obstacle detection (in cm)
#define DISTANCE_THRESHOLD 10

// Variables to track obstacle state
long previousDistance = 999;  // Previous distance to compare with current
bool obstacleMovingToward = false; // Flag for obstacle moving closer

void setup() {
  // Set motor control pins as OUTPUT
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure distance from the ultrasonic sensor
  long distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Detect obstacle movement
  if (distance < DISTANCE_THRESHOLD) {
    obstacleMovingToward = (distance < previousDistance);
    if (obstacleMovingToward) {
      // If obstacle is moving closer, reverse
      stopMotors();
    } else {
      // If obstacle is stationary, stop temporarily
      stopMotors();
    }
  } else {
    // No obstacle within threshold, move forward
    moveMotorsForward();
  }

  // Save the current distance for next loop comparison
  previousDistance = distance;

  delay(100); // Small delay for stability
}

// Function to measure distance using the ultrasonic sensor
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = duration * 0.034 / 2; // Convert duration to distance in cm
  return distance;
}

// Function to move motors forward
void moveMotorsForward() {
  analogWrite(IN1, MOTOR_SPEED);
  analogWrite(IN2, 0);
  analogWrite(IN3, MOTOR_SPEED);
  analogWrite(IN4, 0);
}



// Function to stop motors
void stopMotors() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
