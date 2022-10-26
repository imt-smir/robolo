#include "motor.h"

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
    analogWrite(pin_pwm_m1, abs(pwm1));
    if(pwm1 > 0){
        digitalWrite(pin_InAM1, LOW);
        digitalWrite(pin_InBM1, HIGH);
    }
    else{
        digitalWrite(pin_InAM1, HIGH);
        digitalWrite(pin_InBM1, LOW);
    }

    analogWrite(pin_pwm_m2, abs(pwm2));
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
    

    vel_desejada1 = (2 * x - z * DISTANCIA_DAS_RODAS) * RAD_PARA_ROT / (2 * RAIO_DA_RODA);;
    if (vel_desejada1 < -VEL_MAX){
        vel_desejada1 = -VEL_MAX;
    }
    else if(vel_desejada1 > VEL_MAX){
        vel_desejada1 = VEL_MAX;
    }

    vel_desejada2 = (2 * x + z * DISTANCIA_DAS_RODAS) * RAD_PARA_ROT / (2 * RAIO_DA_RODA);
    if (vel_desejada2 < -VEL_MAX){
        vel_desejada2 = -VEL_MAX;
    }
    else if(vel_desejada2 > VEL_MAX){
        vel_desejada2 = VEL_MAX;
    }
}