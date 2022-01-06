#include "Motion_Control.h"
#include "Arduino.h"

// Declaração de valores para os membros estáticos

static Encoder *_instancesEncoder[10];
static Axis *_instancesAxis[10];

uint8_t Encoder::_totalInstances = 0;
uint8_t Axis::_totalInstances = 0;

int8_t Encoder::_fwdOrder1[2] = {-1, 1};
int8_t Encoder::_fwdOrder2[4] = {1, -1, -1, 1};
int8_t Encoder::_fwdOrder4[4] = {2, 0, 3, 1};

// ************************************************************ Funções da classe Encoder ************************************************************

// Construtores da classe encoder

Encoder::Encoder(uint8_t pin_IN1){

  _pinIN1 = pin_IN1;
  _pinIN2 = 0;

  _pulsesPerRev = 0;
  _pitchPerRev = 0;
  _direction = 1;
  _detectionMode = 0;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);

  assignEvent();

}

Encoder::Encoder(uint8_t pin_IN1, int32_t pulsesPerRev, double pitchPerRev){

  _pinIN1 = pin_IN1;
  _pinIN2 = 0;

  _pulsesPerRev = pulsesPerRev;
  _pitchPerRev = pitchPerRev;
  _direction = 1;
  _detectionMode = 0;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  if(_pulsesPerRev < 0) _pulsesPerRev = 0;
  if(_pitchPerRev < 0) _pitchPerRev = 0;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);

  assignEvent();

}

Encoder::Encoder(uint8_t pin_IN1, uint8_t pin_IN2){

  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;

  _pulsesPerRev = 0;
  _pitchPerRev = 0;
  _direction = 1;
  _detectionMode = 1;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);
  pinMode(_pinIN2, INPUT_PULLUP);

  assignEvent();

}

Encoder::Encoder(uint8_t pin_IN1, uint8_t pin_IN2, uint8_t mode){

  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;

  _pulsesPerRev = 0;
  _pitchPerRev = 0;
  _detectionMode = mode;
  _direction = 1;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  if(_detectionMode < 1 || _detectionMode > 3) _detectionMode = 1;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);
  pinMode(_pinIN2, INPUT_PULLUP);

  assignEvent();

}

Encoder::Encoder(uint8_t pin_IN1, uint8_t pin_IN2, int32_t pulsesPerRev, double pitchPerRev){

  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;

  _pulsesPerRev = pulsesPerRev;
  _pitchPerRev = pitchPerRev;
  _direction = 1;
  _detectionMode = 0;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  if(_pulsesPerRev < 0) _pulsesPerRev = 0;
  if(_pitchPerRev < 0) _pitchPerRev = 0;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);
  pinMode(_pinIN2, INPUT_PULLUP);

  assignEvent();

}

Encoder::Encoder(uint8_t pin_IN1, uint8_t pin_IN2, int32_t pulsesPerRev, double pitchPerRev, uint8_t mode){

  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;

  _pulsesPerRev = pulsesPerRev;
  _pitchPerRev = pitchPerRev;
  _detectionMode = mode;
  _direction = 1;
  _triggerSpeed = 100000;
  _currentPulses = 0;

  if(_pulsesPerRev < 0) _pulsesPerRev = 0;
  if(_pitchPerRev < 0) _pitchPerRev = 0;
  if(_detectionMode < 1 || _detectionMode > 3) _detectionMode = 1;

  _currentEventTime = 1;
  _lastEventTime = 0;

  pinMode(_pinIN1, INPUT_PULLUP);
  pinMode(_pinIN2, INPUT_PULLUP);

  assignEvent();

}

// Funções para configuração do encoder

void Encoder::setReference(){

  _currentPulses = 0;

}

void Encoder::setReference(double value){

  _currentPulses = value;

}

void Encoder::setPulsesPerRev(int32_t value){

  switch (_detectionMode) {

    case 0: _pulsesPerRev = value; break;
    case 1: _pulsesPerRev = 2 * value; break;
    case 2: _pulsesPerRev = 4 * value; break;

  }

}

void Encoder::setPitchPerRev(double value){

  _pitchPerRev = value;

}

void Encoder::setSpeedTrigger(int32_t value){

  _triggerSpeed = value;

}

// Funções de leitura gerais

int32_t Encoder::readPulses(){

  return _currentPulses;

}

double Encoder::readPosition(uint8_t mode){

  /* Possible units:

  * mode = 0: position in pulses
   * mode = 1: position in mm
   * mode = 2: position in rotations
   * mode = 3: position in º

  */

  if(mode == 0){

    return (double)_currentPulses;

  } else if(_pulsesPerRev != 0 && _pitchPerRev != 0){

    double temp;

    switch (mode) {
      case 0: temp = ((double)_currentPulses * _pitchPerRev) / (double)_pulsesPerRev; break;
      case 1: temp = (double)_currentPulses / (double)_pulsesPerRev; break;
      case 2: temp = 360 * (double)_currentPulses / (double)_pulsesPerRev; break;
    }

    return temp;

  } else {

    return 0;

  }

}

double Encoder::readSpeed(uint8_t mode){
  /* Possible units:

   * mode = 0: speed in pulses/s
   * mode = 1: speed in mm/s
   * mode = 2: speed in Hz
   * mode = 3: speed in RPM
   * mode = 4: º/s

  */

  int32_t delta = _currentEventTime - _lastEventTime;

  if(delta < 1) delta = 1;

  if(micros() - _currentEventTime > _triggerSpeed){

    return 0;

  } else if(mode == 0){

    return 1000000 * (double)_direction / delta;

  } else if(_pulsesPerRev != 0 && _pitchPerRev != 0){

    double temp;
      switch (mode) {
        case 1: temp = (_pitchPerRev * (double)_direction * 1000000) / (_pulsesPerRev * delta); break;
        case 2: temp = (1000000 * (double)_direction) / (_pulsesPerRev * delta); break;
        case 3: temp = (60 * 1000000 * (double)_direction) / (_pulsesPerRev * delta); break;
        case 4: temp = (360 * 1000000 * (double)_direction) / (_pulsesPerRev * delta); break;
      }

      return temp;

  } else {

    return 0;

  }

}

// Funções de leitura para movimentos lineares

double Encoder::readLinearPosition(){

  if(_pulsesPerRev != 0 && _pitchPerRev != 0){

    return ((double)_currentPulses * _pitchPerRev) / (double)_pulsesPerRev;

  } else {

    return 0;

  }

}

double Encoder::readLinearSpeed(){

  if(_pulsesPerRev != 0 && _pitchPerRev != 0){

    long delta = _currentEventTime - _lastEventTime;

    if(delta < 1) delta = 1;

    if(micros() - _currentEventTime > _triggerSpeed) return 0;
    else return (_pitchPerRev * _direction * 1000000) / (_pulsesPerRev * delta);

  } else {

    return 0;

  }

}

// Funções de leitura para movimentos angulares

double Encoder::readAngularPosition(){

  if(_pulsesPerRev != 0){

    return 360 * (double)_currentPulses / (double)_pulsesPerRev;

  } else {

    return 0;

  }

}

double Encoder::readAngularSpeed(){

  if(_pulsesPerRev != 0){

    long delta = _currentEventTime - _lastEventTime;

    if(delta < 1) delta = 1;

    if(micros() - _currentEventTime > _triggerSpeed) return 0;
    else return (360 * 1000000 * (double)_direction) / (_pulsesPerRev * delta);

  } else {

    return 0;

  }


}

// Função usadas para rodar as interrupções

void Encoder::assignEvent(){

  _instancesEncoder[this->_totalInstances] = this;

  switch (this->_totalInstances) {

    case 0: assignDetection(ISREvent0); break;
    case 1: assignDetection(ISREvent1); break;
    case 2: assignDetection(ISREvent2); break;
    case 3: assignDetection(ISREvent3); break;
    case 4: assignDetection(ISREvent4); break;
    case 5: assignDetection(ISREvent5); break;
    case 6: assignDetection(ISREvent6); break;
    case 7: assignDetection(ISREvent7); break;
    case 8: assignDetection(ISREvent8); break;
    case 9: assignDetection(ISREvent9); break;

  }

  this->_totalInstances++;

}

void Encoder::assignDetection(void (*ISREvent)(void)){

  switch (_detectionMode) {

    case 0:

      _pulsesPerRev = _pulsesPerRev * 1;

      attachInterrupt(digitalPinToInterrupt(_pinIN1), ISREvent, RISING);

      break;

    case 1:

      _pulsesPerRev = _pulsesPerRev * 1;
      _currentState = digitalRead(_pinIN2);

      attachInterrupt(digitalPinToInterrupt(_pinIN1), ISREvent, RISING);

      break;

    case 2:

      _pulsesPerRev = _pulsesPerRev * 2;
      _currentState = (digitalRead(_pinIN1) << 1) | digitalRead(_pinIN2);

      attachInterrupt(digitalPinToInterrupt(_pinIN1), ISREvent, CHANGE);

      break;

    case 3:

      _pulsesPerRev = _pulsesPerRev * 4;
      _currentState = (digitalRead(_pinIN1) << 1) | digitalRead(_pinIN2);

      attachInterrupt(digitalPinToInterrupt(_pinIN1), ISREvent, CHANGE);
      attachInterrupt(digitalPinToInterrupt(_pinIN2), ISREvent, CHANGE);

      break;

  }

}

void Encoder::ISREvent0(){

  switch (_instancesEncoder[0]->_detectionMode) {

    case 0: _instancesEncoder[0]->runInterrupt1(); break;
    case 1: _instancesEncoder[0]->runInterrupt2(); break;
    case 2: _instancesEncoder[0]->runInterrupt3(); break;
    case 3: _instancesEncoder[0]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent1(){

  switch (_instancesEncoder[1]->_detectionMode) {

    case 0: _instancesEncoder[1]->runInterrupt1(); break;
    case 1: _instancesEncoder[1]->runInterrupt2(); break;
    case 2: _instancesEncoder[1]->runInterrupt3(); break;
    case 3: _instancesEncoder[1]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent2(){

  switch (_instancesEncoder[2]->_detectionMode) {

    case 0: _instancesEncoder[2]->runInterrupt1(); break;
    case 1: _instancesEncoder[2]->runInterrupt2(); break;
    case 2: _instancesEncoder[2]->runInterrupt3(); break;
    case 3: _instancesEncoder[2]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent3(){

  switch (_instancesEncoder[3]->_detectionMode) {

    case 0: _instancesEncoder[3]->runInterrupt1(); break;
    case 1: _instancesEncoder[3]->runInterrupt2(); break;
    case 2: _instancesEncoder[3]->runInterrupt3(); break;
    case 3: _instancesEncoder[3]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent4(){

  switch (_instancesEncoder[4]->_detectionMode) {

    case 0: _instancesEncoder[4]->runInterrupt1(); break;
    case 1: _instancesEncoder[4]->runInterrupt2(); break;
    case 2: _instancesEncoder[4]->runInterrupt3(); break;
    case 3: _instancesEncoder[4]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent5(){

  switch (_instancesEncoder[5]->_detectionMode) {

    case 0: _instancesEncoder[5]->runInterrupt1(); break;
    case 1: _instancesEncoder[5]->runInterrupt2(); break;
    case 2: _instancesEncoder[5]->runInterrupt3(); break;
    case 3: _instancesEncoder[5]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent6(){

  switch (_instancesEncoder[6]->_detectionMode) {

    case 0: _instancesEncoder[6]->runInterrupt1(); break;
    case 1: _instancesEncoder[6]->runInterrupt2(); break;
    case 2: _instancesEncoder[6]->runInterrupt3(); break;
    case 3: _instancesEncoder[6]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent7(){

  switch (_instancesEncoder[7]->_detectionMode) {

    case 0: _instancesEncoder[7]->runInterrupt1(); break;
    case 1: _instancesEncoder[7]->runInterrupt2(); break;
    case 2: _instancesEncoder[7]->runInterrupt3(); break;
    case 3: _instancesEncoder[7]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent8(){

  switch (_instancesEncoder[8]->_detectionMode) {

    case 0: _instancesEncoder[8]->runInterrupt1(); break;
    case 1: _instancesEncoder[8]->runInterrupt2(); break;
    case 2: _instancesEncoder[8]->runInterrupt3(); break;
    case 3: _instancesEncoder[8]->runInterrupt4(); break;

  }

}

void Encoder::ISREvent9(){

  switch (_instancesEncoder[9]->_detectionMode) {

    case 0: _instancesEncoder[9]->runInterrupt1(); break;
    case 1: _instancesEncoder[9]->runInterrupt2(); break;
    case 2: _instancesEncoder[9]->runInterrupt3(); break;
    case 3: _instancesEncoder[9]->runInterrupt4(); break;

  }

}

void Encoder::runInterrupt1(){

  _lastEventTime = _currentEventTime;
  _currentEventTime = micros();

  _currentPulses++;

}

void Encoder::runInterrupt2(){

  _lastEventTime = _currentEventTime;
  _currentEventTime = micros();

  _currentState = digitalRead(_pinIN2);

  _currentPulses = _currentPulses + this->_fwdOrder1[_currentState];
  _direction = this->_fwdOrder1[_currentState];

}

void Encoder::runInterrupt3(){

  _lastEventTime = _currentEventTime;
  _currentEventTime = micros();

  _currentState = (digitalRead(_pinIN1) << 1) | digitalRead(_pinIN2);

  _currentPulses = _currentPulses + this->_fwdOrder2[_currentState];
  _direction = this->_fwdOrder2[_currentState];

}

void Encoder::runInterrupt4(){

  _lastEventTime = _currentEventTime;
  _currentEventTime = micros();

  _lastState = _currentState;
  _currentState = (digitalRead(_pinIN1) << 1) | digitalRead(_pinIN2);

  if(_lastState == this->_fwdOrder4[_currentState]){
    _currentPulses++;
    _direction = 1;
  } else {
    _currentPulses--;
    _direction = -1;
  }

}

// ************************************************************ Funções da classe Axis ************************************************************

// Construtores da classe

Axis::Axis(uint8_t pin_output){

  _pinOutput = pin_output;
  _pinIN1 = 0;
  _pinIN2 = 0;
  _pinLimitMax = 0;
  _pinLimitMin = 0;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 0;

  #if defined(ARDUINO_ARCH_ESP32)

  #else
    pinMode(_pinOutput, OUTPUT);
  #endif

}

Axis::Axis(uint8_t pin_output, uint8_t pin_IN1, uint8_t pin_IN2){

  _pinOutput = pin_output;
  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;
  _pinLimitMax = 0;
  _pinLimitMin = 0;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 1;

  pinMode(_pinIN1, OUTPUT);
  pinMode(_pinIN2, OUTPUT);

  #if defined(ARDUINO_ARCH_ESP32)

  #else
    pinMode(_pinOutput, OUTPUT);
  #endif

}

Axis::Axis(uint8_t pin_output, uint8_t pin_IN1, uint8_t pin_IN2, uint8_t pin_limitMin, uint8_t pin_limitMax){

  _pinOutput = pin_output;
  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;
  _pinLimitMin = pin_limitMin;
  _pinLimitMax = pin_limitMax;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 1;

  pinMode(_pinIN1, OUTPUT);
  pinMode(_pinIN2, OUTPUT);
  pinMode(_pinLimitMin, INPUT_PULLUP);
  pinMode(_pinLimitMax, INPUT_PULLUP);

  assignEvent();

  #if defined(ARDUINO_ARCH_ESP32)

  #else
    pinMode(_pinOutput, OUTPUT);
  #endif

}

Axis::Axis(uint8_t pin_output, uint8_t control_channel){

  _pinOutput = pin_output;
  _pinIN1 = 0;
  _pinIN2 = 0;
  _pinLimitMax = 0;
  _pinLimitMin = 0;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 0;

  #if defined(ARDUINO_ARCH_ESP32)
    _controlChannel = control_channel;
    ledcAttachPin(pin_output, control_channel);
    ledcSetup(control_channel, 5000, 12);
  #else
    //pinMode(_pinOutput, OUTPUT);
  #endif

}

Axis::Axis(uint8_t pin_output, uint8_t control_channel, uint8_t pin_IN1, uint8_t pin_IN2){

  _pinOutput = pin_output;
  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;
  _pinLimitMax = 0;
  _pinLimitMin = 0;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 1;

  pinMode(_pinIN1, OUTPUT);
  pinMode(_pinIN2, OUTPUT);

  #if defined(ARDUINO_ARCH_ESP32)
    _controlChannel = control_channel;
    ledcAttachPin(pin_output, control_channel);
    ledcSetup(control_channel, 5000, 12);
  #else
    //pinMode(_pinOutput, OUTPUT);
  #endif

}

Axis::Axis(uint8_t pin_output, uint8_t control_channel, uint8_t pin_IN1, uint8_t pin_IN2, uint8_t pin_limitMin, uint8_t pin_limitMax){

  _pinOutput = pin_output;
  _pinIN1 = pin_IN1;
  _pinIN2 = pin_IN2;
  _pinLimitMax = pin_limitMax;
  _pinLimitMin = pin_limitMin;

  _opMode = 0;
  _output = 0;
  _maxOutput = 1.0;
  _invertible = 1;

  pinMode(_pinIN1, OUTPUT);
  pinMode(_pinIN2, OUTPUT);
  pinMode(_pinLimitMin, INPUT_PULLUP);
  pinMode(_pinLimitMax, INPUT_PULLUP);

  assignEvent();

  #if defined(ARDUINO_ARCH_ESP32)
    _controlChannel = control_channel;
    ledcAttachPin(pin_output, control_channel);
    ledcSetup(control_channel, 5000, 12);
  #else
    //pinMode(_pinOutput, OUTPUT);
  #endif

}

// Funções set para alterar os parametros do eixo

void Axis::setOpMode(bool mode){

  _opMode = mode;

  if(_opMode == 0) looseAxis();

}

void Axis::setMaxOutput(double value){

  if(value > 0 && value <= 1.0) _maxOutput = value;
  else _maxOutput = 1.0;

}

void Axis::setOutput(double value){

  if(abs(value) < 1.0001) _output = abs(value);
  else _output = 0;

}

// Funções get para leitura dos parametros do eixo

uint8_t Axis::getID(){

  return _id;

}

uint8_t Axis::getOpMode(){

  return _opMode;

}

double Axis::getMaxOutput(){

  return _maxOutput;

}

double Axis::getOutput(){

  return _output;

}

// Funções de movimentação do eixo

void Axis::moveAxis(){

  if(_opMode == 1 && _invertible == 0){

    #if defined(ARDUINO_ARCH_ESP32)
      int16_t output = (int16_t)round(_maxOutput * _output * 4095);
      ledcWrite(_controlChannel, output);
    #else
      int8_t output = (int16_t)round(_maxOutput * _output * 255);
      analogWrite(_pinOutput, output);
    #endif

  } else {

    looseAxis();

  }

}

void Axis::moveAxisFoward(){

  if(_opMode == 1 && _invertible == 1){

    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, LOW);

    #if defined(ARDUINO_ARCH_ESP32)
      int16_t output = (int16_t)round(_maxOutput * _output * 4095);
      ledcWrite(_controlChannel, output);
    #else
      int8_t output = (int16_t)round(_maxOutput * _output * 255);
      analogWrite(_pinOutput, output);
    #endif

  } else {

    looseAxis();

  }

}

void Axis::moveAxisBackward(){

  if(_opMode == 1 && _invertible == 1){

    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, HIGH);

    #if defined(ARDUINO_ARCH_ESP32)
      int16_t output = (int16_t)round(_maxOutput * _output * 4095);
      ledcWrite(_controlChannel, output);
    #else
      int8_t output = (int16_t)round(_maxOutput * _output * 255);
      analogWrite(_pinOutput, output);
    #endif

  } else {

    looseAxis();

  }

}

void Axis::brakeAxis(){

  if(_opMode == 1 && _invertible == 1){

    digitalWrite(_pinIN1, HIGH);
    digitalWrite(_pinIN2, HIGH);

    #if defined(ARDUINO_ARCH_ESP32)
      ledcWrite(_controlChannel, 0);
    #else
      analogWrite(_pinOutput, 0);
    #endif

  } else {

    looseAxis();

  }

}

void Axis::looseAxis(){

  if(_invertible == 1){
    digitalWrite(_pinIN1, LOW);
    digitalWrite(_pinIN2, LOW);
  }

  #if defined(ARDUINO_ARCH_ESP32)
    ledcWrite(_controlChannel, 0);
  #else
    analogWrite(_pinOutput, 0);
  #endif

}

// Função usadas para rodar as interrupções

void Axis::assignEvent(){

  _instancesAxis[this->_totalInstances] = this;

  switch (this->_totalInstances) {

    case 0:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent0, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent0, FALLING);

      break;

    case 1:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent1, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent1, FALLING);

      break;

    case 2:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent2, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent2, FALLING);

      break;

    case 3:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent3, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent3, FALLING);

      break;

    case 4:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent4, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent4, FALLING);

      break;

    case 5:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent5, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent5, FALLING);

      break;

    case 6:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent6, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent6, FALLING);

      break;

    case 7:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent7, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent7, FALLING);

      break;

    case 8:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent8, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent8, FALLING);

      break;

    case 9:

      attachInterrupt(digitalPinToInterrupt(_pinLimitMin), ISREvent9, FALLING);
      attachInterrupt(digitalPinToInterrupt(_pinLimitMax), ISREvent9, FALLING);

      break;

  }

  _id = this->_totalInstances;
  this->_totalInstances++;

}

void Axis::ISREvent0(){
  _instancesAxis[0]->runInterrupt();
}

void Axis::ISREvent1(){
  _instancesAxis[1]->runInterrupt();
}

void Axis::ISREvent2(){
  _instancesAxis[2]->runInterrupt();
}

void Axis::ISREvent3(){
  _instancesAxis[3]->runInterrupt();
}

void Axis::ISREvent4(){
  _instancesAxis[4]->runInterrupt();
}

void Axis::ISREvent5(){
  _instancesAxis[5]->runInterrupt();
}

void Axis::ISREvent6(){
  _instancesAxis[6]->runInterrupt();
}

void Axis::ISREvent7(){
  _instancesAxis[7]->runInterrupt();
}

void Axis::ISREvent8(){
  _instancesAxis[8]->runInterrupt();
}

void Axis::ISREvent9(){
  _instancesAxis[9]->runInterrupt();
}

void Axis::runInterrupt(){

  setOpMode(0);

}

// ************************************************************ Funções da classe Controller ************************************************************

// Construtores da classe

Controller::Controller(Axis *axes, Encoder *encoders, int8_t nAxes){

  _nAxes = nAxes;

  _axes = axes;
  _encoders = encoders;
  _pid = new PID_setup[_nAxes];

  _opMode = 0;

  for(int i = 0; i < _nAxes; i++) inicializePID(&_pid[i]);

}

Controller::Controller(Axis *axes, Encoder *encoders, int8_t nAxes, Envelope *envelope){

  _nAxes = nAxes;

  _axes = axes;
  _encoders = encoders;
  _pid = new PID_setup[_nAxes];
  _envelope = envelope;

  _opMode = 0;

  for(int i = 0; i < _nAxes; i++) inicializePID(&_pid[i]);

}

// Funções para operação do controlador

void Controller::runController(){

  if(_opMode == 1){

    bool check;

    for(int i = 0; i < _nAxes; i++){

      if(_pid[i].controller != 0){

        _pid[i].input = _encoders[i].readPosition(_pid[i].readingMode);
        if(runCalculation(&_pid[i]) == 1) check = moveAxis(&_axes[i]);

      }

    }

  }

}

void Controller::runController(Axis axis){

  if(axis.getOpMode() == 1){

    bool check;
    int8_t id = axis.getID();

    _pid[id].input = _encoders[id].readPosition(_pid[id].readingMode);
    if(runCalculation(&_pid[id]) == 1) check = moveAxis(&_axes[id]);

  }

}

// Funções set para alterar os parametros do controlador

void Controller::setOpMode(bool value){

  _opMode = value;

}

void Controller::setSetpoint(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setSetpoint(_axes[i], data.setpoint[i]);

}

void Controller::setSetpoint(Axis axis, double setpoint){

  int8_t id = axis.getID();

  if(_envelope != NULL) {

    if(setpoint >= _envelope->limitMin[id] && setpoint <= _envelope->limitMax[id]) _pid[id].setpoint = setpoint;
    else _opMode = 0;

  } else {

    _pid[id].setpoint = setpoint;

  }

}

void Controller::setController(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setController(_axes[i], data.controller[i]);

}

void Controller::setController(Axis axis){

  int8_t id = axis.getID();

  if(_opMode == 0){

    if(_pid[id].Kp != 0 && _pid[id].Ki && _pid[id].Kd != 0) _pid[id].controller = 4;
    else if(_pid[id].Kp != 0 && _pid[id].Kd) _pid[id].controller = 3;
    else if(_pid[id].Kp != 0 && _pid[id].Ki) _pid[id].controller = 2;
    else if(_pid[id].Kp != 0) _pid[id].controller = 1;
    else _pid[id].controller = 0;

  }

}

void Controller::setController(Axis axis, uint8_t controller){

  int8_t id = axis.getID();

  if(_opMode == 0){

    switch (controller) {

      case 0:

        _pid[id].controller = 0;
        break;

      case 1:

        if(_pid[id].Kp != 0) _pid[id].controller = controller;
        else _pid[id].controller = 0;

      case 2:

        if(_pid[id].Kp != 0 && _pid[id].Ki) _pid[id].controller = controller;
        else _pid[id].controller = 0;

      case 3:

        if(_pid[id].Kp != 0 && _pid[id].Kd) _pid[id].controller = controller;
        else _pid[id].controller = 0;

      case 4:

        if(_pid[id].Kp != 0 && _pid[id].Ki && _pid[id].Kd != 0) _pid[id].controller = controller;
        else _pid[id].controller = 0;

    }

  }

}

void Controller::setGains(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setGains(_axes[i], data.Kp[i], data.Ki[i], data.Kd[i]);

}

void Controller::setGains(Axis axis, double Kp, double Ki, double Kd){

  int8_t id = axis.getID();

  _pid[id].Kp = Kp;
  _pid[id].Ki = Ki * (_pid[id].period / 1000000);
  _pid[id].Kd = Kd / (_pid[id].period / 1000000);

}

void Controller::setPIDPeriod(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setPIDPeriod(_axes[i], data.period[i]);

}

void Controller::setPIDPeriod(Axis axis, int32_t period){

  int8_t id = axis.getID();

  if(period >= 10000) _pid[id].period = period;
  else _pid[id].period = 100000;

}

void Controller::setTolerance(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setTolerance(_axes[i], data.tolerance[i]);

}

void Controller::setTolerance(Axis axis, double tolerance){

  int8_t id = axis.getID();

  if(tolerance > 0) _pid[id].tolerance = tolerance;
  else _pid[id].tolerance = 1;

}

void Controller::setReadingMode(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setPIDPeriod(_axes[i], data.readingMode[i]);

}

void Controller::setReadingMode(Axis axis, int8_t readingMode){

  int8_t id = axis.getID();

  if(readingMode >= 0 && readingMode <= 3) _pid[id].readingMode = readingMode;
  else _pid[id].readingMode = 1;

}

void Controller::setAll(Controller_data data){

  for(int i = 0; i < _nAxes; i++) setAll(_axes[i], data);

}

void Controller::setAll(Axis axis, Controller_data data){

  int8_t id = axis.getID();

  setSetpoint(_axes[id], data.setpoint[id]);
  setController(_axes[id], data.controller[id]);
  setGains(_axes[id], data.Kp[id], data.Ki[id], data.Kd[id]);
  setPIDPeriod(_axes[id], data.period[id]);
  setTolerance(_axes[id], data.tolerance[id]);
  setPIDPeriod(_axes[id], data.readingMode[id]);

}

// Funções get para leitura dos parametros do controlador

bool Controller::getOpMode(){

  return _opMode;

}

Controller_data Controller::getInput(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.input[i] = getInput(_axes[i]);

  return temp;

}

double Controller::getInput(Axis axis){

  return _pid[axis.getID()].input;

}

Controller_data Controller::getOutput(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.output[i] = getOutput(_axes[i]);

  return temp;

}

double Controller::getOutput(Axis axis){

  return _pid[axis.getID()].output;

}

Controller_data Controller::getSetpoint(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.setpoint[i] = getSetpoint(_axes[i]);

  return temp;

}

double Controller::getSetpoint(Axis axis){

  return _pid[axis.getID()].setpoint;

}

Controller_data Controller::getGains(){

  Controller_data temp1, temp2;

  for(int i = 0; i < _nAxes; i++){

    temp1 = getGains(_axes[i]);

    temp2.Kp[i] = temp1.Kp[0];
    temp2.Ki[i] = temp1.Ki[0];
    temp2.Kd[i] = temp1.Kd[0];

  }

  return temp2;

}

Controller_data Controller::getGains(Axis axis){

  Controller_data temp;
  int8_t id = axis.getID();

  temp.Kp[0] = _pid[id].Kp;
  temp.Ki[0] = _pid[id].Ki;
  temp.Kd[0] = _pid[id].Kd;

  return temp;

}

Controller_data Controller::getPIDPeriod(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.period[i] = getPIDPeriod(_axes[i]);

  return temp;

}

double Controller::getPIDPeriod(Axis axis){

  return _pid[axis.getID()].period;

}

Controller_data Controller::getController(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.controller[i] = getController(_axes[i]);

  return temp;

}

int8_t Controller::getController(Axis axis){

  return _pid[axis.getID()].controller;

}

Controller_data Controller::getTolerance(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.tolerance[i] = getTolerance(_axes[i]);

  return temp;

}

double Controller::getTolerance(Axis axis){

  return _pid[axis.getID()].tolerance;

}

Controller_data Controller::getReadingMode(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++) temp.readingMode[i] = getReadingMode(_axes[i]);

  return temp;


}

int8_t Controller::getReadingMode(Axis axis){

  return _pid[axis.getID()].readingMode;

}

Controller_data Controller::getAll(){

  Controller_data temp;

  for(int i = 0; i < _nAxes; i++){

    temp.input[i] = _pid[i].input;
    temp.output[i] = _pid[i].output;
    temp.setpoint[i] = _pid[i].setpoint;
    temp.Kp[i] = _pid[i].Kp;
    temp.Ki[i] = _pid[i].Ki;
    temp.Kd[i] = _pid[i].Kd;
    temp.period[i] = _pid[i].period;
    temp.controller[i] = _pid[i].controller;
    temp.tolerance[i] = _pid[i].tolerance;
    temp.readingMode[i] = _pid[i].readingMode;

  }

  return temp;

}

Controller_data Controller::getAll(Axis axis){

  Controller_data temp;
  int8_t id = axis.getID();

  temp.input[0] = _pid[id].input;
  temp.output[0] = _pid[id].output;
  temp.setpoint[0] = _pid[id].setpoint;
  temp.Kp[0] = _pid[id].Kp;
  temp.Ki[0] = _pid[id].Ki;
  temp.Kd[0] = _pid[id].Kd;
  temp.period[0] = _pid[id].period;
  temp.controller[0] = _pid[id].controller;
  temp.tolerance[0] = _pid[id].tolerance;
  temp.readingMode[0] = _pid[id].readingMode;

  return temp;

}

int8_t Controller::getTotalAxes(){

  return _nAxes;

}

// Funções auxiliares

bool Controller::isInPosition(){

  bool temp = 1;

  for(int i = 0; i < _nAxes; i++) temp = temp * isInPosition(&_pid[i]);

  return temp;

}

bool Controller::isInPosition(Axis axis){

  return isInPosition(&_pid[axis.getID()]);

}

bool Controller::isInPosition(PID_setup *pid){

  double input = pid->input;
  double error = pid->setpoint - input;
  double tolerance = pid->tolerance;

  if(abs(error) <= tolerance) return 1;
  else return 0;

}

bool Controller::moveAxis(Axis *axis){

  int id = axis->getID();

  axis->setOutput(_pid[id].output);

  if(axis->getOpMode() == 1){

    if(_pid[id].output > 0) axis->moveAxisFoward();
    else if(_pid[id].output < 0) axis->moveAxisBackward();
    else axis->looseAxis();

    return 1;

  } else {

    return 0;

  }

}

bool Controller::runCalculation(PID_setup *pid){

  double currentTime = micros();
  double dt = currentTime - pid->lastIteration;

  if(dt > pid->period){

    // Limitar o valor máximo do dt para quando o eixo passa muito tempo sem rodar o calculo
    if(dt > pid->period * 1.1) dt = pid->period * 1.1;

    pid->dt = dt / 1000000;
    pid->error = pid->setpoint - pid->input;
    pid->dError = pid->error - pid->lastError;

    // Calcular a integral do erro apenas enquanto a saída não está saturada
    if(abs(pid->output) < 1) pid->integral = pid->integral + pid->error * pid->dt;

    switch(pid->controller){

      /* Controladores:
        - case 0: Control off
        - case 1: P
        - case 2: PI
        - case 3: PD
        - case 4: PID
      */

      case 0: pid->output = 0;
      case 1: pid->output = pid->Kp * pid->error; break;
      case 2: pid->output = pid->Kp * pid->error + pid->Ki * pid->integral; break;
      case 3: pid->output = pid->Kp * pid->error + pid->Kd * pid->dError; break;
      case 4: pid->output = pid->Kp * pid->error + pid->Ki * pid->integral + pid->Kd * pid->dError; break;

    }

    // Saturar a saída caso ultrapasse 1
    if(pid->output > 1) pid->output = 1;
    else if(pid->output < - 1) pid->output = -1;

    // Atualizar os parâmetros da última iteração
    pid->lastError = pid->error;
    pid->lastIteration = currentTime;

    return 1;

  } else {

    return 0;

  }

}

void Controller::inicializePID(PID_setup *pid){

  pid->Kp = 1;
  pid->Ki = 0;
  pid->Kd = 0;

  pid->input = 0;
  pid->output = 0;
  pid->setpoint = 0;
  pid->period = 100000;
  pid->tolerance = 1;

  pid->lastError = 0;
  pid->lastInput = 0;
  pid->integral = 0;
  pid->lastIteration = 0;

  pid->controller = 0;
  pid->readingMode = 1;

}
