#include <Motion_control.h>

// Parâmetros do encoder

int channel1 = 2; // channel 1
int channel2 = 3; // channel 2
long pulses = 600; // resolution in pulses per revolution
double pitch = 40.0; // [mm]

// Parâmetros do eixo

byte pinOutput = 5;
byte pinIN1 = 6;
byte pinIN2 = 7;
double maxOutput = 0.5; // output maximo de 50% da capacidade do motor
double output1 = 0.8; // 80% do output máximo definido -> 40% da capacidade do motor
double output2 = 0.5; // 50% do output máximo definido -> 25% da capacidade do motor

Axis axis(pinOutput, pinIN1, pinIN2);
Encoder encoder(channel1, channel2, pulses, pitch);

bool sentido = 1;
long currentTime = 0, waiting = 0, printData = 0;
double ref1 = 0, ref2 = 250.0, currentPosition;

void setup() {

  axis.setOpMode(1); // Ativa o eixo
  axis.setMaxOutput(maxOutput); // Limita o output para 50%
  axis.setOutput(output1);; // Começa com 80% (40% do total) de output

  Serial.begin(9600);
  
}

void loop() {

  currentTime = millis();
  currentPosition = encoder.readLinearPosition();

  if(sentido == 1 && (currentTime - waiting) > 2000){ // Move o eixo para frente
    
    if(currentPosition >= ref2){
     
      sentido = 0;
      waiting = currentTime;
      axis.setOutput(output2);
      axis.looseAxis();
      
    } else {
      
      axis.moveAxisFoward();
      
    }
    
  } else if (sentido == 0 && (currentTime - waiting) > 2000){ // Move o eixo para trás

    if(currentPosition <= ref1){
  
      sentido = 1;
      waiting = currentTime;
      axis.setOutput(output1);
      axis.looseAxis();
      
    } else {
      
      axis.moveAxisBackward();
      
    }

  }

  if(currentTime - printData > 100){
    
    Serial.println(currentPosition);
    printData = currentTime;

  }

}
