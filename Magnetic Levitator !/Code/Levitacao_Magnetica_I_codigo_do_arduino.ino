//Projeto de Levitacao Magnetica implementado no arduino
//controle PID

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define ajusteTimer1    256     
#define prescaler   64 

const int analogInPin = A0;  
const int analogInPin1 = A1; 
const int analogInPin2 = A2; 
const int analogOutPin = 9; // Analog output pin that the transistor is attached to

int sensorValue = 0;   // value read from the pot
int sensorValue1 = 0;  
int sensorValue2 = 0;  
int outputValue = 0;        // value output to the PWM (analog out)

float posicaoRef = 0; //em mm
float campoRef = 0;

int erroPosicao  = 0;
int erroCampo  = 0;
float erroCurrent  = 0;

float derivada;

int derivadaCampo, derivadaErro;
float derivadaCurrent; 
int sensorValueA;
float integral;

float  kp, kd, ki;

// constantes para convercao AD rapida
#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


void setup() {
  // initialize serial communications at 115200 bps:
  Serial.begin(115200); 
  pinMode(analogOutPin, OUTPUT);  
  Setup_timer2();
  sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt
  //convercao AD rapida
  // set prescale to 16
  sbi(ADCSRA,ADPS2) ;
  cbi(ADCSRA,ADPS1) ;
  cbi(ADCSRA,ADPS0) ;
  delay(20000); // tempo para sincronizar arduino e processing sem erro
}

float funcaoPWMxB(float b) {
  float pwm;
  pwm= 45.16289+927.1107*pow(2.6429,-b/22.25711)+86.97756*pow(2.6429,-b/184.46454);
  return pwm;
}

float funcaoBxZ(float z) {
  float b;
  b = 8.14+700*pow(2.6429,-z/1.481);
  //  b = z;
  return b;
}

float funcaoZxB(float b) {
  float z;
  z = b;
  return z;
}

void loop() {

  posicaoRef =analogRead(analogInPin2);
  posicaoRef = float(posicaoRef)*8/1024;

  sensorValue1=analogRead(analogInPin1);

  kp=analogRead(A3);
  kp=kp*6/1024;
  kd=analogRead(A4);
  kd=kd*25/1024;
  ki=analogRead(5);
  ki=ki*0.1/1024;

  sensorValueA= sensorValue;

  //Obtendo um valor medio  
  sensorValue=0; 
  for(int i=0; i<16; i++)
    sensorValue  =  sensorValue + analogRead(analogInPin);
  sensorValue  = sensorValue/16; 

  //da posiçao de set point para campo de set point, pois a minha entrada do sensor é campo magnetico, e nao posicao
  campoRef = funcaoBxZ(posicaoRef);

  //erro  
  erroPosicao = posicaoRef - funcaoZxB(sensorValue);  
  erroCampo = campoRef - sensorValue;
  //posicaoRef - sensorValue negativo indica que o objeto esta acima da posiçao de referencia e a corrente de saida deve ser menor
  //posicaoRef - sensorValue positivo indica que o objeto esta abaixo da posiçao de referencia e a corrente de saida deve ser maior
  erroCurrent = funcaoPWMxB(sensorValue)- funcaoPWMxB(campoRef) ;
  derivada= - funcaoZxB(sensorValue)+ funcaoZxB(sensorValueA);
  //obs: derivada =  derivadaCampo
  derivadaCampo= sensorValueA- sensorValue;
  derivadaErro=derivadaCampo;
  derivadaCurrent= + funcaoPWMxB(sensorValue) - funcaoPWMxB(sensorValueA);

  if(ki == 0)
    integral= 0;
  else 
    integral= integral+  erroCurrent;

  // quanto mais proximo da bobina, maior potencial para a instabilidade pois qualquer pequena mudança na corrente da bobina acarreta numa maior variaçao da posição
  // e quando mais proximo da bobina(sensor), maior é tambem a variação do valor de campo lido pelo sensor
  //sendo assim, a minima oescilaçao na altura acarreta numa grande variação no valor do sensor, causando uma maior variação do campo magnetico e maior a instabilidade
  //isto no caso de um controle cujos termos p i d sejam porporcionais ao erro do sensor
  // para esse sistema é melhor um controle no qual os ganhos sejam menores quanto mais perto o objeto estiver da bobina. Maior será a estabilidade
  //a partir de uma calibragem rapida, testes, verificou-se isso

  // outputValue = float( funcaoPWMxB(campoRef)) + float( derivadaCampo)*kd+ float(erroCampo)*kp +integral*ki; // neste controle quanto mais proximo da bobina maiores serao os pesos

  outputValue =  funcaoPWMxB(campoRef) +  derivadaCurrent*kd+ erroCurrent*kp +integral*ki; // neste controle quanto mais proximo da bobina menores serao os pesos
  //kp 1.21
  //kd 9.67
  //ki 1.66

  // print the results on processing:
  processingComunicate();

}


ISR(TIMER2_OVF_vect) {

  if (outputValue>255)
    analogWrite(analogOutPin, 255); 
  else if (outputValue<0)

    analogWrite(analogOutPin,0);  
  else
    analogWrite(analogOutPin, outputValue);   
}


// timer2 setup
// set prescaler to 1, PWM mode to phase correct PWM,  16000000/510 = 31372.55 Hz clock
// set prescaler to 3, PWM mode to phase correct PWM,  (16000000/510)/2^3 = 3921.57 Hz clock
void Setup_timer2() {

  // Timer2 Clock Prescaler to : 4
  sbi (TCCR2B, CS20);
  sbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);


  // Timer2 PWM Mode set to Phase Correct PWM
  cbi (TCCR2A, COM2A0);  // clear Compare Match
  sbi (TCCR2A, COM2A1);

  sbi (TCCR2A, WGM20);  // Mode 1  / Phase Correct PWM
  cbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);

}

void processingComunicate() {
  Serial.println(sensorValue);

  Serial.println(outputValue);

  Serial.println(kp);
  Serial.println(kd);
  Serial.println(ki*1000);

  Serial.println(campoRef); 

  if (Serial.available() > 0){
    int valorLido = Serial.read();
    posicaoRef = valorLido;
  }
}


