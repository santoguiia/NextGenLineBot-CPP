#include <SPI.h>
#include <Pixy.h>
#include <BasicLinearAlgebra.h>
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

using namespace BLA;
#define START   39

// Parametros do Robo Global:
float T = 50; //time step (mm)

//inicialize:
float dw = 0; // controle
float Up, Ui, U;
bool paramMode = 0;

//ajuste PI:
float Kp = .7;
float Ki = 0.0;
float Uiant = 0;

//ajuste controle atraso:
float b = .01;
float c = b * 700; // (T ^ 2 * r ^ 2 * wd / L);
float Uant = 0;

int espera = 0;
float xt = 0;
float yt = 0;

float xError[10];

unsigned long controlTimer = millis();           // Timer to track elapsed time

#define segmento 1      //tamanho do segmento de reta entre os pontos da curva
#define n_segmentos 10  //numero de segmentos
#define RX_PIN 3
#define TX_PIN 1
#define X_CENTER ((PIXY_MAX_X - PIXY_MIN_X) / 2)
float y_vec[n_segmentos];
// This is the main Pixy object
Pixy pixy;

HardwareSerial SerialPort(0);

void setup() {
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  pixy.init();
  SerialBT.begin("Cabeça");
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(START, INPUT);

  delay(1000);
}

void loop() {
  // Track elapsed time
  unsigned long currentTime = millis();

  if (currentTime - controlTimer >= T) {
    if (digitalRead(START)) {
      Uiant = 0;
      Uant = 0;
    }
    if (SerialBT.available()) {
      String str = "";
      while (SerialBT.available()) {
        char c = SerialBT.read();
        str += c;
      }
      float f = atof(str.c_str());
      if (f == 10) {
        SerialBT.print(b);
        SerialBT.print(" ");
        SerialBT.print(c);
        SerialBT.print(" ");
        SerialBT.println(dw);
      }
      else if (f == 11) {
        paramMode = 0;
      }
      else if (f == 12) {
        paramMode = 1;
      }
      else {
        if (paramMode == 0) {
          c = c/b * f;
          b = f;
        }
        else {
          c = b * f;
        }
      }
    }

    int j;
    uint16_t blocks;

    // grab blocks!
    blocks = pixy.getBlocks();
    //Serial.println(pixy.blocks[j].x);

    // If there are detect blocks, print them!
    if (blocks) {
      float x = 0;
      float y = 0;
      float xcm = 0;
      float ycm = 0;
      int width = 0;
      espera++;
      for (j = 0; j < blocks; j++) {
        if (width < pixy.blocks[j].width) {
          width = pixy.blocks[j].width;
          x = (pixy.blocks[j].x - X_CENTER);
        }
        if (y < pixy.blocks[j].height) {
          y = pixy.blocks[j].height;
        }
      }
      xt += x;
      yt += y;

      if (espera % 5 == 0) {
        xt = xt / 5;
        yt = yt / 5;
        //        Serial.println(xt);      Serial.print(" ");
        if (xt >= 250) {
          xt = 250;
        }
        if (yt >= 199) {
          yt = 199;
        }
        espera = 0;
        //x = 159;
        //y = 199;
        xcm = (xt / 159) * (6 + (yt / 199) * abs(xt / 159) * 9);
        ycm = (yt / 199) * 13;
        float r = ycm / sin(atan(xcm / ycm));
        float angulo = 1000000 * acos(1 - ((segmento ^ 2) / (2 * r * r)));
        for (int i = 0; i < n_segmentos; i++) {
          xError[i] = r * (1 - cos(i * angulo / 1000000));
          if (isnan(xError[i])) {
            xError[i] = 0;
          }
        }
        //        // Controle PI:
        //        Up = Kp * xt;
        //        Ui = Uiant + Ki * xt;
        //        Ui = constrain(Ui, -127, 127); //evitar windup
        //        Uiant = Ui;
        //        dw = Ui + Up;

        //      if (yt<= 80)
        //        dw = dw *3;

        // Controle atraso (filtro):
        U = Uant * b + c * xt * T / 1000; //xError[9];
        U = constrain(U, -127, 127); //evitar windup
        Uant = U;
        dw = U;

        if (dw >= 126)
          dw = 126;
        if (dw <= -126)
          dw = -126;
        SerialBT.println(dw);

        byte send = dw;
        send = send + 127;

        //        if ((pixy.blocks[1].width * pixy.blocks[1].height > 450) and (pixy.blocks[1].x > 140)){
        //          send = 0;
        //        }

        if (pixy.blocks[0].width * pixy.blocks[0].height > 57000) {
          send = 254;
          SerialBT.println("x");
        }
        Serial.write(send);
        //        Serial.println(xt);      Serial.print(" ");
        //      Serial.println(yt);      Serial.print(" ");

        xt = 0;
        yt = 0;
      }
    }
  }
}
