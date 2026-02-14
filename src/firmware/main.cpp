#include <Arduino.h> // Or the nRF5 SDK / Zephyr if you're feeling brave

const int DOOR_PIN = 2; // Connect reed switch here

void setup() {
    pinMode(DOOR_PIN, INPUT_PULLUP);
    
    // TACTICAL: Wake up the CPU only when the pin state changes
    attachInterrupt(digitalPinToInterrupt(DOOR_PIN), onDoorStateChange, CHANGE);
    
    Serial.begin(115200);
    Serial.println("System Initialized: Deep Sleep Mode Active");
}

void onDoorStateChange() {
    // This runs the moment the door opens or closes
    bool isOpen = digitalRead(DOOR_PIN);
    
    if (isOpen) {
        // Broadcast "Intrusion Detected"
    } else {
        // Broadcast "Secure"
    }
}

void loop() {
    // The loop stays empty to demonstrate low-power design.
    // The CPU is managed by the interrupt handler.
}
