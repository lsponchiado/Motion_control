#include <Motion_control.h>

// valores utilizados pelo objeto da classe Encoder

int pin1 = 2; // channel 1
int pin2 = 3; // channel 2
long pulses = 600; // resolution in pulses per revolution
double pitch = 40.0; // [mm]

// cria um novo objeto da classe Encoder
Encoder encoder(pin1, pin2, pulses, pitch);

long current = 0, last = 0;

void setup() {
  
  Serial.begin(9600);
  
}

void loop() {
  
  current = millis();

  if (current - last >= 100) {
    
    Serial.println(encoder.readLinearPosition());
    //Serial.println(encoder.readLinearSpeed());
    
    last = current;
    
  }

}
