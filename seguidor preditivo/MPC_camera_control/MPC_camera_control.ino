#include <SPI.h>
#include <Pixy.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;
// Parametros do Robo Global:
float T = .5;  //time step
float r = 1;   //wheel radius
byte wd = 1;  // defined angular velocity
float L = 10;  //Length between wheels
//inicialize:
float dw = 0, du = 0;  // controle
int espera = 0;
float xt = 0;
float yt = 0;
//parametros MPC:
float alfa = .3;
#define N 10   // prediction horizon
#define Nu 10  // control horizon
float lamb = .7;
float K1[N];
float xError[N];

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
  //    G = alfa * tril(ones(Nu, N), 0);
  BLA::Matrix<Nu, N>
  G;
  G.Fill(0);
  // Control matrix G:
  for (int i = 0; i < Nu; i++) {   //columns
    for (int j = 0; j < N; j++) {  //lines
      if (i >= j) {
        G(i, j) = alfa;
      }
    }
  }
  BLA::Matrix<N, Nu> Gt = ~G;
  BLA::Matrix<N, N> C = Gt * G;
  BLA::Matrix<N, Nu> L;
  L.Fill(0);
  for (int i = 0; i < Nu; i++) {   //columns
    for (int j = 0; j < N; j++) {  //lines
      if (i == j) {
        L(i, j) = lamb;
      }
    }
  }
  BLA::Matrix<N, N> D = C + L;
  BLA::Matrix<N, N> D_inv = D;
  bool is_nonsingular = Invert(D_inv);
  BLA::Matrix<N, N> K = D_inv * Gt * 100;
  for (int i = 0; i < N; i++) {
    K1[i] = K(0, i);  // Assign the values from K to K1
  }
  delay(1000);
}

void loop() {
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
    for (j = 0; j <= blocks; j++) {
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
      if (xt >= 250) {
        xt = 0;
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
        y_vec[i] = r * sin(i * angulo / 1000000);
        if (isnan(y_vec[i])) {
          y_vec[i] = 0;
        }
      }
      float f[Nu];
      for (int i = 0; i < N; i++) {
        f[i] = alfa * dw;
      }
      du = 0;
      for (int i = 0; i < N; i++) {
        du += (K1[i] * (xError[i] - f[i])) / 100;
      }
      if (isnan(dw)) {
        dw = 0;
      }
      dw = dw + du;
      byte send = 0;
      send = dw * 18.2 + 127.5;
      if (send >= 255)
        send = 255;
      if (send <= 0)
        send = 0;
            Serial.write(wd);
            SerialPort.write(send);

      xt = 0;
      yt = 0;
    }
  }
}
