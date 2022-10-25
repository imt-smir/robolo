/*
terminal1: roscore
terminal2: rosrun teleop_twist_keyboard teleop_twist_keyboard.py
terminal3: ./controlador_com_ros_e_serial.py
*/

/*
CONTAS:
x: velocidade linear do teleop
z: velocidade angular do teleop
R = RAIO_DA_RODA = 2.6 cm : raio da roda 
L = DISTANCIA_DAS_RODAS = 22.3 cm : distância entre as rodas 

n [rotação] = RAD_PARA_ROT = 0.5 [rotação] * 1 [rad] / pi [rad] = 0.1592 [rotação]

Vright = VEL_DESEJADA2 = (2 * x + z * L) * n / (2 * R)
Vleft = VEL_DESEJADA1 = (2 * x - z * L) * n / (2 * R)
*/

// Importando bibliotecas
#include <SoftwareSerial.h>
#include <stdlib.h>
SoftwareSerial mySerial(10, 11); // RX, TX

//  Constantes utilizadas
const float RAIO_DA_RODA = 2.6/100;
const float DISTANCIA_DAS_RODAS = 22.3/100;
const float RAD_PARA_ROT = 0.1592;
const float VEL_MAX = 7;
const long TEMPO_DE_LEITURA = 100; // E-03 segundos

// Valores resgatados do Teleop
float x = 0;
float z = 0;

// Variáveis que armazena os dados recebidos da entrada serial
char c = 'i'; // tanto faz o valor inicializado
char v[5]; // velocidade linear
char w[5]; // velocidade angular

// Dados recebidos da entrada serial
String data;

// Velocidades Desejadas 
float VEL_DESEJADA1 = 0.0; // em rotações por segundo (Vmáx = 9.1 rot/s)
float VEL_DESEJADA2 = 0.0;

//  Pinos dos PWMs dos Motores (1/2)
const int pin_PwmM1 = 6;
const int pin_PwmM2 = 3;

//  Pinos das Direções dos Motors (1/2)
const int pin_InAM1 = 22;
const int pin_InBM1 = 24;
const int pin_InAM2 = 26;
const int pin_InBM2 = 28;

//  Pinos dos Encoders (A/B) dos Motores (1/2)
//     Motor1
const int pin_EN1A = 20;
const int pin_EN1B = 9;
//     Motor 2
const int pin_EN2A = 21;
const int pin_EN2B = 8;

// Pino do botão para acionamento do código
const int pin_Botao = 12;

// Variáveis relacionadas à leitura dos Encoders dos Motores (1/2)
volatile int leitura1 = 0;
int ant_leitura1 = 0;
volatile int leitura2 = 0;
int ant_leitura2 = 0;

// Variáveis relacionadas ao tempo
long tf = 0; // t(k) em milissegundos
long to = 0; // t(k-1) em milissegundos
long contador = 0; // millis() - contador < TEMPO_DE_LEITURA
float dt; // [t(k) - t(k-1)] em segundos

// Variáveis relacionadas à velocidade
float velocidade1 = 0; // v(k) em rotações por segundo
float ant_velocidade1 = 0; // v(k-1) em rotações por segundo
float velocidade2 = 0; // v(k) em rotações por segundo
float ant_velocidade2 = 0; // v(k-1) em rotações por segundo

// Variáveis relacionadas ao erro das velocidades
float e1 = 0; // e(k) = vDesejada - v(k)
float ant_e1 = 0; // e(k-1) = vDesejada - v(k-1)
float e2 = 0; // e(k) = vDesejada - v(k)
float ant_e2 = 0; // e(k-1) = vDesejada - v(k-1)

// Variáveis relacionadas ao movimento dos motores
int pwm1 = 0; // valor que será inputado no motor, controlando a intensidade da velocidade
int dir1 = 1; // valor que será inputado no motor, controlando sua direção (dizendo se irá para frente ou para trás)
int pwm2 = 0; // valor que será inputado no motor, controlando a intensidade da velocidade
int dir2 = 1; // valor que será inputado no motor, controlando sua direção (dizendo se irá para frente ou para trás)

// Funções (explicações na montagem da função no final do código)
void girarMotor(int dir, int pin, int pwmVal, int in1, int in2); 
void create_string(char* v, char* w);
void alteraVelocidade();


// Setup
void setup() {
    mySerial.begin(57600); // porta serial escolhida
    Serial.begin(9600);

    // pinMode dos pinos de Direção
    pinMode(pin_InAM1, OUTPUT); 
    pinMode(pin_InBM1, OUTPUT);
    pinMode(pin_InAM2, OUTPUT);
    pinMode(pin_InBM2, OUTPUT);

    // pinMode do botão
    pinMode(pin_Botao, INPUT);

    // pinMode dos pinos dos Encoders
    pinMode(pin_EN1A, INPUT);
    pinMode(pin_EN1B, INPUT);
    pinMode(pin_EN2A, INPUT);
    pinMode(pin_EN2B, INPUT);

    // criando a característica de interrupção para leitura dos pinos dos Encoders
    attachInterrupt(digitalPinToInterrupt(pin_EN1A), atualiza_leitura1, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_EN2A), atualiza_leitura2, RISING);

}

// Loop
void loop() {
    c = mySerial.read();
    if(mySerial.available() > 0){
        create_string(v, w);
        alteraVelocidade();
    }

    //Criando um intervalo para a mudança da velocidade do motor
    if (millis() - contador > TEMPO_DE_LEITURA) {
        // Cálculo do tempo
        tf = millis(); // em milissegundo
        dt = ((float) (tf - to)) / (1000.0); // em segundos
        to = tf; // em milissegundo

        // Cálculo da velocidade
        ant_velocidade1  = velocidade1; // em rotações por segundo
        velocidade1  = (leitura1 - ant_leitura1)/ (300.0 * dt);   // 300 pulsos por volta, portanto isto transforma
                                                                // pulsos por segundo em rotações por segundo
        ant_leitura1  = leitura1; // em rotações por segundo

        ant_velocidade2 = velocidade2;
        velocidade2 = (leitura2 - ant_leitura2) / (300.0 * dt);
        ant_leitura2 = leitura2;

        // Cálculo do erro
        ant_e1 = e1;
        e1 = VEL_DESEJADA1 - velocidade1;

        ant_e2 = e2;
        e2 = VEL_DESEJADA2 - velocidade2;

        // Aplicando a equação de diferenças encontrada
        pwm1 = 11.05 * e1 + 1.093 * ant_e1 + pwm1;
        pwm2 = 9.247 * e2 + 2.608 * ant_e2 + pwm2;

        if (pwm1 >= 255) pwm1 = 255; // pwmMáx = 255
        if (pwm1 <= -255) pwm1 = -255;

        if (pwm2 >= 255) pwm2 = 255; // pwmMáx = 255
        if (pwm2 <= -255) pwm2 = -255;


        // Atualizando o movimento do motor
        moveMotor(pwm1, pwm2);

        // Próxima iteração do "void loop()" não entrará novamente neste while, apenas quando o novo intervalo chegar
        contador = millis();
    }
}

// Criação das Funções

// Quando o encoder1 perceber algum giro na roda1, esta função atualizará o valor de leitura1
void atualiza_leitura1() {
  if (digitalRead(pin_EN1B) == 1) leitura1++;
  else leitura1--;
}

// Quando o encoder2 perceber algum giro na roda2, esta função atualizará o valor de leitura2
void atualiza_leitura2() {
  if (digitalRead(pin_EN2B) == 0) leitura2--;
  else leitura2++;
}

// Determina o movimento do motor
void moveMotor(int pwm1, int pwm2){
    analogWrite(pin_PwmM1, abs(pwm1));
    if(pwm1 > 0){
        digitalWrite(pin_InAM1, LOW);
        digitalWrite(pin_InBM1, HIGH);
    }
    else{
        digitalWrite(pin_InAM1, HIGH);
        digitalWrite(pin_InBM1, LOW);
    }

    analogWrite(pin_PwmM2, abs(pwm2));
    if(pwm2 > 0){
        digitalWrite(pin_InBM2, LOW);
        digitalWrite(pin_InAM2, HIGH);
    }
    else{
        digitalWrite(pin_InBM2, HIGH);
        digitalWrite(pin_InAM2, LOW);
    }

}

// Recebe os valores lidos da entrada serial, e transforma
// em uma sequência de char 
void create_string(char* v, char* w){
    int idx = 0;
    int i = 3; 
    data = mySerial.readString();
    while(data[idx] == 's'){
        idx++;
    }
    // Velocidade linear
    v[0] = data[idx];
    idx++;
    if(v[0] == '-'){
        i++;
    }
    for(int k = 1; k < i; k++){
        v[k] = data[idx];
        idx++;
    }
    v[i] = '\0';

    // Velocidade angular
    i = 3;
    w[0] = data[idx];
    idx++;
    if(w[0] == '-'){
        i++;
    }
    for(int k = 1; k < i; k++){
        w[k] = data[idx];
        idx++;
    }
    w[i] = '\0';
}


// Altera as velocidades desejadas conforme os controles no teleop
void alteraVelocidade(){
    x = atof(v);
    z = atof(w);
    

    VEL_DESEJADA1 = (2 * x - z * DISTANCIA_DAS_RODAS) * RAD_PARA_ROT / (2 * RAIO_DA_RODA);;
    if (VEL_DESEJADA1 < -VEL_MAX){
        VEL_DESEJADA1 = -VEL_MAX;
    }
    else if(VEL_DESEJADA1 > VEL_MAX){
        VEL_DESEJADA1 = VEL_MAX;
    }

    VEL_DESEJADA2 = (2 * x + z * DISTANCIA_DAS_RODAS) * RAD_PARA_ROT / (2 * RAIO_DA_RODA);
    if (VEL_DESEJADA2 < -VEL_MAX){
        VEL_DESEJADA2 = -VEL_MAX;
    }
    else if(VEL_DESEJADA2 > VEL_MAX){
        VEL_DESEJADA2 = VEL_MAX;
    }