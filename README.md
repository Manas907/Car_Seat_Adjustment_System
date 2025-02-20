# Car Seat Adjustment System

## Project Overview
The Seat Adjustment System is a simple embedded hardware project designed to automatically adjust the position and angle of a car seat using a motor and an embedded controller (Arduino or similar microcontroller). The user can adjust the seat position by turning a potentiometer or pressing buttons to control the seat's movement.

## Components Required
- **Microcontroller**: Arduino Uno (or any microcontroller with PWM capability)
- **Motor Driver**: L298N (to control the motor)
- **DC Motor**: To adjust the seat position
- **Potentiometer**: To simulate the user input for seat position
- **Buttons**: To set seat angles or movements (optional)
- **Power Supply**: To power the motor and microcontroller
- **Cables and Connectors**: For wiring the components
- **Breadboard**: For prototyping (if required)

## Hardware Circuit

### Motor Control
The motor will be connected to the motor driver (L298N), which is then controlled by the microcontroller via PWM signals.

### Potentiometer
This will simulate user input to adjust the seat position.

### Buttons
Used to control forward/backward seat movement.

## Code Implementation
```cpp
// Include necessary libraries
#include <Arduino.h>

// Define pins for motor control
#define IN1 3     // Motor A input pin 1
#define IN2 4     // Motor A input pin 2
#define ENA 5     // Motor A PWM pin (for speed control)
#define POT_PIN A0 // Potentiometer input pin for seat position

// Motor direction constants
#define FORWARD HIGH
#define BACKWARD LOW

// Setup the potentiometer reading and motor control pins
void setup() {
  Serial.begin(9600);

  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Initialize PWM for motor speed control
  analogWrite(ENA, 255); // Max speed initially

  // Setup potentiometer pin
  pinMode(POT_PIN, INPUT);

  Serial.println("Seat Adjustment System Initialized.");
}

// Main loop
void loop() {
  // Read potentiometer value (0-1023)
  int potValue = analogRead(POT_PIN);
  int seatPosition = map(potValue, 0, 1023, 0, 180); // Map to seat angle range (e.g., 0-180 degrees)

  // Display the seat position on the Serial Monitor
  Serial.print("Seat Position: ");
  Serial.println(seatPosition);

  // Control motor based on potentiometer value (seat position)
  if (seatPosition < 90) {
    // Move seat backward
    digitalWrite(IN1, BACKWARD);
    digitalWrite(IN2, FORWARD);
  } else if (seatPosition > 90) {
    // Move seat forward
    digitalWrite(IN1, FORWARD);
    digitalWrite(IN2, BACKWARD);
  } else {
    // Stop the motor when seat is in the middle
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  // Delay to prevent excessive motor movement
  delay(100);
}
