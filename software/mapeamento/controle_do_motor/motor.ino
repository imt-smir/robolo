#include "motor.h"

// Criação das Funções
// Quando o encoder1 perceber algum giro na roda1, esta função atualizará o valor de encoder_data1
void update_encoder_data1() {
  if (digitalRead(pin_EN1B) == 1) encoder_data1++;
  else encoder_data1--;
}

// Quando o encoder2 perceber algum giro na roda2, esta função atualizará o valor de encoder_data2
void update_encoder_data2() {
  if (digitalRead(pin_EN2B) == 0) encoder_data2--;
  else encoder_data2++;
}

// Determina o movimento do motor
void rotateMotor(int pwm1, int pwm2){
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

// Recebe os valores lidos da entrada serial, e transforma em uma sequência de char 
// void create_string(char* v, char* w){
//     int idx = 0;
//     int i = 3; 
//     data = mySerial.readString();
//     while(data[idx] == 's'){
//         idx++;
//     }
//     // Velocidade linear
//     v[0] = data[idx];
//     idx++;
//     if(v[0] == '-'){
//         i++;
//     }
//     for(int k = 1; k < i; k++){
//         v[k] = data[idx];
//         idx++;
//     }
//     v[i] = '\0';

//     // Velocidade angular
//     i = 3;
//     w[0] = data[idx];
//     idx++;
//     if(w[0] == '-'){
//         i++;
//     }
//     for(int k = 1; k < i; k++){
//         w[k] = data[idx];
//         idx++;
//     }
//     w[i] = '\0';
// }




// Altera as velocidades desejadas conforme os controles no teleop
// void changeVelocity(){
//     x = atof(v);
//     z = atof(w);
    

//     setpoint_vel1 = (2 * x - z * WHEEL_DISTANCE) * RAD_TO_ROT / (2 * WHEEL_RADIUS);;
//     if (setpoint_vel1 < -MAX_VEL){
//         setpoint_vel1 = -MAX_VEL;
//     }
//     else if(setpoint_vel1 > MAX_VEL){
//         setpoint_vel1 = MAX_VEL;
//     }

//     setpoint_vel2 = (2 * x + z * WHEEL_DISTANCE) * RAD_TO_ROT / (2 * WHEEL_RADIUS);
//     if (setpoint_vel2 < -MAX_VEL){
//         setpoint_vel2 = -MAX_VEL;
//     }
//     else if(setpoint_vel2 > MAX_VEL){
//         setpoint_vel2 = MAX_VEL;
//     }
// }

bool validateDataChunk(String* data){
    *data = mySerial.readString();
    if((*data)[0] != '\x05')  return false;
    if((*data)[(*data).length() - 1] != '\x04') return false;
    if(atoi((*data)[(*data).length() - 2]) != (*data).length() - 3) return false;
    return true;
}