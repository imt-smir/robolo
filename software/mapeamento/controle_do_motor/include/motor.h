#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

/*
terminal1: roscore
terminal2: rosrun teleop_twist_keyboard teleop_twist_keyboard.py
terminal3: ./controlador_com_ros_e_serial.py
*/

/*
CONTAS:
x: velocidade linear do teleop
z: velocidade angular do teleop
R = WHEEL_RADIUS = 2.6 cm : raio da roda 
L = WHEEL_DISTANCE = 22.3 cm : distância entre as rodas 

n [rotação] = RAD_TO_ROT = 0.5 [rotação] * 1 [rad] / pi [rad] = 0.1592 [rotação]

Vright = setpoint_vel2 = (2 * x + z * L) * n / (2 * R)
Vleft = setpoint_vel1 = (2 * x - z * L) * n / (2 * R)
*/

SoftwareSerial mySerial(10, 11); // RX, TX

//  Constantes utilizadas
#define WHEEL_RADIUS (float)(2.6/100)
#define WHEEL_DISTANCE (float)(22.3/100)
#define RAD_TO_ROT = (float)(0.1592)
#define MAX_VEL = (float)(7)
#define ENCODER_READING_TIME (long)(100) // E-03 segundos

// Valores resgatados do Teleop
float x = 0;
float z = 0;

// Variáveis que armazena os dados recebidos da entrada serial
char c; // verifica o primeiro caractere da entrada serial
char v[5]; // velocidade linear
char w[5]; // velocidade angular

// Dados recebidos da entrada serial
String data;

// Velocidades Desejadas 
float setpoint_vel1 = 0.0; // em rotações por segundo (Vmáx = 9.1 rot/s)
float setpoint_vel2 = 0.0;

//  Pinos dos PWMs dos Motores (1/2)
#define pin_pwm_m1 6
#define pin_pwm_m2 3

//  Pinos das Direções dos Motors (1/2)
#define pin_InAM1 22
#define pin_InBM1 24
#define pin_InAM2 26
#define pin_InBM2 28

//  Pinos dos Encoders (A/B) dos Motores (1/2)
//     Motor1
#define pin_EN1A 20
#define pin_EN1B 9
//     Motor 2
#define pin_EN2A 21
#define pin_EN2B 8

// Pino do botão para acionamento do código
#define pin_Botao 12

// Variáveis relacionadas à leitura dos Encoders dos Motores (1/2)
volatile int encoder_data1 = 0;
int last_encoder_data1 = 0;
volatile int encoder_data2 = 0;
int last_encoder_data2 = 0;

// Variáveis relacionadas ao tempo
long tf = 0; // t(k) em milissegundos
long to = 0; // t(k-1) em milissegundos
long counter = 0; // millis() - counter < ENCODER_READING_TIME
float dt; // [t(k) - t(k-1)] em segundos

// Variáveis relacionadas à velocidade
float velocity1 = 0; // v(k) em rotações por segundo
float last_velocity1 = 0; // v(k-1) em rotações por segundo
float velocity2 = 0; // v(k) em rotações por segundo
float last_velocity2 = 0; // v(k-1) em rotações por segundo

// Variáveis relacionadas ao erro das velocidades
float e1 = 0; // e(k) = vDesejada - v(k)
float last_e1 = 0; // e(k-1) = vDesejada - v(k-1)
float e2 = 0; // e(k) = vDesejada - v(k)
float last_e2 = 0; // e(k-1) = vDesejada - v(k-1)

// Variáveis relacionadas ao movimento dos motores
int pwm1 = 0; // valor que será inputado no motor, controlando a intensidade da velocidade
int dir1 = 1; // valor que será inputado no motor, controlando sua direção (dizendo se irá para frente ou para trás)
int pwm2 = 0; // valor que será inputado no motor, controlando a intensidade da velocidade
int dir2 = 1; // valor que será inputado no motor, controlando sua direção (dizendo se irá para frente ou para trás)

// Funções (explicações na montagem da função no final do código)
void rotateMotor(int pwm1, int pwm2);
void create_string(char* v, char* w);
void changeVelocity();
void update_encoder_data1();
void update_encoder_data2(); 

#endif // MOTOR_H_INCLUDED