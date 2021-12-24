#include <Motion_control.h>

// Variaveis utilizadas no sketch

double setpoint_1[4] = {0.0, 10.0, 20.0, 30.0};
double setpoint_2[4] = {0.0, 5.0, 15.0, 25.0};
int8_t counter1 = 0, counter2 = 0;
int32_t currentTime = 0, delay_axis1 = 0, delay_axis2 = 0;

// Parametros dos encoders

uint8_t channel1_1 = 1, channel2_1 = 2;    // pinos fisicos do encoder 1
uint8_t channel1_2 = 3, channel2_2 = 7;    // pinos fisicos do encoder 2
uint16_t resolution = 600;                 // resolucao dos encoders em pulsos por volta
double pitch = 40.0;                       // pitch dos eixos em mm

Encoder encoders[] = {Encoder(channel1_1, channel2_1, resolution, pitch), Encoder(channel1_2, channel2_2, resolution, pitch)};

// Parametros dos eixos

uint8_t pinControl1 = 10;                  // pino para o sinal PWM do eixo 1
uint8_t pinControl2 = 11;                  // pino para o sinal PWM do eixo 2
uint8_t pinIN1_1 = 5, pinIN2_1 = 6;        // pinos para controle da ponte H do eixo 1
uint8_t pinIN1_2 = 8, pinIN2_2 = 9;        // pinos para controle da ponte H do eixo 2

Axis axes[] = {Axis(pinControl1, pinIN1_1, pinIN2_1), Axis(pinControl2, pinIN1_2, pinIN2_2)};


//  Parametros do controlador

double Kp = 1.0, Ki = 0.5, Kd = 0.0;      // Ganhos do controlador (nesse caso todos os eixos terão os mesmos ganhos)
int8_t nAxes = 2;                         // Numero de eixos a serem controlados

Controller_data data;                     // Variavel que permite interagir com o controlador

Controller controller(axes, encoders, nAxes);


void setup() {

  for(int i = 0; i < nAxes; i++){
    
    data.tolerance[i] = 2.0;        // Tolerance = +/- 2 mm
    data.gains[0][i] = 1.0;         // Kp = 1
    data.gains[1][i] = 0.5;         // Ki = 0.5
    data.gains[2][i] = 0.0;         // Kd = 0.0
    data.controller[i] = 2;         // PI controller

    axes[i].setOpMode(1);           // Ativando os eixos
    
  }

  controller.setTolerance(data);    // Alterando a tolerancia do controlador (padrao = 1)
  controller.setController(data);   // Alterando o tipo de controlador utilizado (padrao = P)
  controller.setGains(data);        // Alterando o valor dos ganhos

  controller.setOpMode(1);          // Ativando o controlador

}

void loop() {
  
  currentTime = millis();
  nextSetpoint();

}

void nextSetpoint(){

  if(controller.isInPosition(axes[0]) == 1 && (currentTime - delay_axis1) >= 3000){

    counter1++;
    if(counter1 == 4) counter1 = 0;
    
    controller.setSetpoint(axes[0], setpoint_1[counter1]);
    
    delay_axis1 = currentTime;
    
   }

   if(controller.isInPosition(axes[1]) == 1 && (currentTime - delay_axis2) >= 3000){

    counter2++;
    if(counter2 == 4) counter2 = 0;
    
    controller.setSetpoint(axes[1], setpoint_2[counter2]);
    
    delay_axis2 = currentTime;
    
   }
  
}
