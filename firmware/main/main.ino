#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

enum class ControlMode { GUI, SENSOR };

// Placeholder functions
void dcMoveRelative(float degrees) {
    Serial.print("DC relative degrees: ");
    Serial.println(degrees);
}

void dcSetVelocity(float rpm) {
    Serial.print("DC RPM: ");
    Serial.println(rpm);
}

void set_servo_angle(float degrees) {
    Serial.print("Servo degrees: ");
    Serial.println(degrees);
}

float stepperMoveRelative(float degrees) {
    Serial.print("Stepper relative degrees: ");
    Serial.println(degrees);
    return degrees;  // Replace with step-rounded angle.
}

void setControlMode(ControlMode mode) {
    Serial.print("Mode: ");
    Serial.println(mode == ControlMode::GUI ? "GUI" : "SENSOR");
}

void setSystemEnabled(bool enabled) {
    Serial.print("Enabled: ");
    Serial.println(enabled);
    // Implement motor enable/disable here.
}

void dcUpdate() {}
void stepperUpdate() {}


// Parse one complete command.
void parseCommand(char* message) {
    char* comma = strchr(message, ',');
    if (comma == nullptr) return;

    *comma = '\0';
    char* command = message;
    char* argument = comma + 1;

    if (strcmp(command, "MODE") == 0) {
        if (strcmp(argument, "GUI") == 0)
            setControlMode(ControlMode::GUI);
        else if (strcmp(argument, "SENSOR") == 0)
            setControlMode(ControlMode::SENSOR);

        return;
    }

    if (strcmp(command, "ENABLE") == 0) {
        if (strcmp(argument, "1") == 0)
            setSystemEnabled(true);
        else if (strcmp(argument, "0") == 0)
            setSystemEnabled(false);

        return;
    }

    char* end;
    float value = (float)strtod(argument, &end);
    if (end == argument || *end != '\0' || !isfinite(value))
        return;

    if (strcmp(command, "DC_MOVE") == 0)
        dcMoveRelative(value);
    else if (strcmp(command, "DC_VEL") == 0)
        dcSetVelocity(value);
    else if (strcmp(command, "SERVO") == 0)
        set_servo_angle(value);
    else if (strcmp(command, "STEPPER_MOVE") == 0)
        stepperMoveRelative(value);
}


// Collect incoming characters without waiting for a full line.
void readSerialCommands() {
    static char buffer[80];
    static uint8_t index = 0;
    static bool overflow = false;

    while (Serial.available() > 0) {
        char c = Serial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            buffer[index] = '\0';

            if (!overflow)
                parseCommand(buffer);

            index = 0;
            overflow = false;
        } else if (!overflow) {
            if (index < sizeof(buffer) - 1)
                buffer[index++] = c;
            else
                overflow = true;
        }
    }
}


void setup() {
    Serial.begin(115200);
    // Initialize sensors and motors here.
}

void loop() {
    readSerialCommands();
    dcUpdate();
    stepperUpdate();
}