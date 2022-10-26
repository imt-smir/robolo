// Importando bibliotecas
#include <SoftwareSerial.h>
#include <stdlib.h>
#include "../include/motor.h"


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
        e1 = vel_desejada1 - velocidade1;

        ant_e2 = e2;
        e2 = vel_desejada2 - velocidade2;

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
