#ifndef Motion_Control_h
#define Motion_Control_h
#define LIBRARY_VERSION	1.0

#include "Arduino.h"


// Structs

struct Envelope {

  double limitMin[5], limitMax[5];

};

struct PID_setup {

  double Kp, Ki, Kd;
  double input, output, setpoint, error, tolerance;
  double lastInput, lastError, dError, dt, integral;
  uint32_t period, lastIteration;
  uint8_t controller, readingMode;

};

struct Controller_data {

  double input[5];
  double output[5];
  double setpoint[5];
  double gains[3][5];
  double tolerance[5];
  uint32_t period[5];
  uint8_t controller[5];
  uint8_t readingMode[5];

};

/* Classe Encoder

  - Utilizada para realizar a leitura de encoders incrementais com 1 ou 2 canais
  - Permite utilizar leituras de x1, x2 e x4 a resolução nominal do encoder (x2 e x4 apenas para encoders de 2 canais)
  - Necessária a utilização de interrupções (1 interrupção para x1 e x2 e 2 interrupções para x4)

*/

class Encoder {

public:

  // Construtores da classe

  Encoder(uint8_t pin_IN1);
  Encoder(uint8_t pin_IN1, int32_t pulsesPerRev, double pitchPerRev);
  Encoder(uint8_t pin_IN1, uint8_t pin_IN2);
  Encoder(uint8_t pin_IN1, uint8_t pin_IN2, uint8_t mode);
  Encoder(uint8_t pin_IN1, uint8_t pin_IN2, int32_t pulsesPerRev, double pitchPerRev);
  Encoder(uint8_t pin_IN1, uint8_t pin_IN2, int32_t pulsesPerRev, double pitchPerRev, uint8_t mode);

  // Funções set para configuração do encoder

  void setReference();
  void setReference(double value);
  void setPulsesPerRev(int32_t value);
  void setPitchPerRev(double value);
  void setSpeedTrigger(int32_t value);

  // Funções de leitura gerais

  int32_t readPulses();
  double readPosition(uint8_t mode);
  double readSpeed(uint8_t mode);

  // Funções de leitura para movimentos lineares

  double readLinearPosition();
  double readLinearSpeed();

  // Funções de leitura para movimentos angulares

  double readAngularPosition();
  double readAngularSpeed();

private:

  // Função usadas para rodar as interrupções

  static void ISREvent0();
  static void ISREvent1();
  static void ISREvent2();
  static void ISREvent3();
  static void ISREvent4();
  static void ISREvent5();
  static void ISREvent6();
  static void ISREvent7();
  static void ISREvent8();
  static void ISREvent9();

  void assignEvent();
  void assignDetection(void (*ISREvent)(void));

  void runInterrupt1();
  void runInterrupt2();
  void runInterrupt3();
  void runInterrupt4();

  // Variáveis da classe

  double _pitchPerRev;
  int8_t _pinIN1, _pinIN2, _lastState, _currentState, _detectionMode, _direction;
  int32_t _pulsesPerRev, _lastEventTime, _currentEventTime, _currentPulses, _triggerSpeed;

  // Variáveis estáticas da classe

  static int8_t _fwdOrder1[2];
  static int8_t _fwdOrder2[4];
  static int8_t _fwdOrder4[4];

  static uint8_t _totalInstances;

};


/* Classe Axis

  - Utilizada para controlar motores via ponte H ou com apenas um sinal
  - Funciona com placas que utilizam analogWrite ou ledcWrite
  - Permite incluir interrupções para limitar fisicamente a movimentação do eixo (máximo de 10 eixos com interrupções)
  - Modos de operação: 0 = desligado, 1 = ligado

*/

class Axis {

public:

  // Para placas que usam analogWrite

  Axis(uint8_t pin_output);
  Axis(uint8_t pin_output, uint8_t pin_IN1, uint8_t pin_IN2);
  Axis(uint8_t pin_output, uint8_t pin_IN1, uint8_t pin_IN2, uint8_t pin_limitMin, uint8_t pin_limitMax);

  // Para placas que usam ledcWrite

  Axis(uint8_t pin_output, uint8_t control_channel);
  Axis(uint8_t pin_output, uint8_t control_channel, uint8_t pin_IN1, uint8_t pin_IN2);
  Axis(uint8_t pin_output, uint8_t control_channel, uint8_t pin_IN1, uint8_t pin_IN2, uint8_t pin_limitMin, uint8_t pin_limitMax);

  // Funções set para alterar os parametros do eixo

  void setOpMode(bool mode);
  void setMaxOutput(double value);
  void setOutput(double value);

  // Funções get para leitura dos parametros do eixo

  uint8_t getID();
  uint8_t getOpMode();
  double getMaxOutput();
  double getOutput();

  // Funções de movimentação do eixo

  void moveAxis();
  void moveAxisFoward();
  void moveAxisBackward();
  void brakeAxis();
  void looseAxis();


private:

  // Função usadas para rodar as interrupções

  static void ISREvent0();
  static void ISREvent1();
  static void ISREvent2();
  static void ISREvent3();
  static void ISREvent4();
  static void ISREvent5();
  static void ISREvent6();
  static void ISREvent7();
  static void ISREvent8();
  static void ISREvent9();

  void assignEvent();
  void runInterrupt();

  // Variáveis da classe

  bool _invertible, _opMode;
  uint8_t _pinOutput, _pinIN1, _pinIN2, _pinLimitMin, _pinLimitMax, _id;
  double _output;

  #if defined(ARDUINO_ARCH_ESP32)

    uint8_t _controlChannel;
    uint16_t _maxOutput;

  #else

    uint8_t _maxOutput;

  #endif

  // Variáveis estáticas da classe

  static uint8_t _totalInstances;

};


/* Classe Controller

  -
  -
  -

*/

class Controller {

public:

  // Construtores da classe

  Controller(Axis *axes, Encoder *encoders);
  Controller(Axis *axes, Encoder *encoders, int8_t sequence[5]);
  Controller(Axis *axes, Encoder *encoders, Envelope *envelope);
  Controller(Axis *axes, Encoder *encoders, Envelope *envelope, int8_t sequence[5]);


  // Funções para operação do controlador
  void runController();

  // Funções set para alterar os parametros do controlador

  void setOpMode(bool value);
  void setSetpoint(Controller_data data);
  void setSetpoint(Axis axis, double setpoint);
  void setController(Controller_data data);
  void setController(Axis axis, uint8_t controller);
  void setGains(Controller_data data);
  void setGains(Axis axis, double Kp, double Ki, double Kd);
  void setPIDPeriod(Controller_data data);
  void setPIDPeriod(Axis axis, int32_t period);
  void setTolerance(Controller_data data);
  void setTolerance(Axis axis, double tolerance);
  void setReadingMode(Controller_data data);
  void setReadingMode(Axis axis, int8_t readingMode);
  void setAll(Controller_data data);
  void setAll(Axis *axis, Controller_data data);

  // Funções get para leitura dos parametros do controlador

  bool getOpMode();
  Controller_data getInput();
  double getInput(Axis *axis);
  Controller_data getOutput();
  double getOutput(Axis *axis);
  Controller_data getSetpoint();
  double getSetpoint(Axis *axis);
  Controller_data getGains();
  Controller_data getGains(Axis *axis);
  Controller_data getPIDPeriod();
  double getPIDPeriod(Axis *axis);
  Controller_data getController();
  int8_t getController(Axis *axis);
  Controller_data getTolerance();
  double getTolerance(Axis *axis);
  Controller_data getReadingMode();
  int8_t getReadingMode(Axis *axis);
  Controller_data getAll();
  Controller_data getAll(Axis *axis);

  // Funções auxiliares

  bool isInPosition(Axis axis);
  //

private:

  bool moveAxis(Axis *axis);
  bool isInPosition(PID_setup *pid);
  bool runCalculation(PID_setup *pid);
  void inicializePID(PID_setup *pid);

  Axis *_axes;
  Encoder *_encoders;
  PID_setup *_pid;
  Envelope *_envelope;

  int8_t _nAxes, _sequence[5];
  bool _opMode, _sequenceMode;

};

#endif
