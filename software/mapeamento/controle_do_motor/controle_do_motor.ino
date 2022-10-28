// Importando bibliotecas
#include <SoftwareSerial.h>
#include <stdlib.h>
#include "motor.h"


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
    attachInterrupt(digitalPinToInterrupt(pin_EN1A), update_encoder_data1, RISING);
    attachInterrupt(digitalPinToInterrupt(pin_EN2A), update_encoder_data2, RISING);

}

// Loop
void loop() {
    c = mySerial.read();
    if(mySerial.available() > 0){
        data_not_lost = validateDataChunk(&data);
        Serial.print(data);
        Serial.print('\t');
        Serial.println(data_not_lost);
        readingdataChunk(&v, &w, data);
        stringToVel(v, w, &x, &z);
        updateSetpoint(x, z, &setpoint_vel1, &setpoint_vel2);
    }

    // //Criando um intervalo para a mudança da velocidade do motor
    // if (millis() - counter > ENCODER_READING_TIME) {
    //     // Cálculo do tempo
    //     tf = millis(); // em milissegundo
    //     dt = ((float) (tf - to)) / (1000.0); // em segundos
    //     to = tf; // em milissegundo

    //     // Cálculo da velocidade
    //     last_velocity1  = velocity1; // em rotações por segundo
    //     velocity1  = (encoder_data1 - last_encoder_data1)/ (300.0 * dt);   // 300 pulsos por volta, portanto isto transforma
    //                                                             // pulsos por segundo em rotações por segundo
    //     last_encoder_data1  = encoder_data1; // em rotações por segundo

    //     last_velocity2 = velocity2;
    //     velocity2 = (encoder_data2 - last_encoder_data2) / (300.0 * dt);
    //     last_encoder_data2 = encoder_data2;

    //     // Cálculo do erro
    //     last_e1 = e1;
    //     e1 = setpoint_vel1 - velocity1;

    //     last_e2 = e2;
    //     e2 = setpoint_vel2 - velocity2;

    //     // Aplicando a equação de diferenças encontrada
    //     pwm1 = 11.05 * e1 + 1.093 * last_e1 + pwm1;
    //     pwm2 = 9.247 * e2 + 2.608 * last_e2 + pwm2;

    //     if (pwm1 >= 255) pwm1 = 255; // pwmMáx = 255
    //     if (pwm1 <= -255) pwm1 = -255;

    //     if (pwm2 >= 255) pwm2 = 255; // pwmMáx = 255
    //     if (pwm2 <= -255) pwm2 = -255;


    //     // Atualizando o movimento do motor
    //     rotateMotor(pwm1, pwm2);

    //     // Próxima iteração do "void loop()" não entrará novamente neste while, apenas quando o novo intervalo chegar
    //     counter = millis();
    // }
}
