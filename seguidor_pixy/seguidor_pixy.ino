//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// This sketch is a good place to start if you're just getting started with 
// Pixy and Arduino.  This program simply prints the detected object blocks 
// (including color codes) through the serial console.  It uses the Arduino's 
// ICSP port.  For more information go here:
//
// http://cmucam.org/projects/cmucam5/wiki/Hooking_up_Pixy_to_a_Microcontroller_(like_an_Arduino)
//
// It prints the detected blocks once per second because printing all of the 
// blocks for all 50 frames per second would overwhelm the Arduino's serial port.
//
   
#include <SPI.h>
#include <Pixy.h>

#define segmento 10 // Tamanho do segmento de reta entre os pontos da curva
#define n_segmentos 10 // Número de segmentos

float x_vec[n_segmentos];
float y_vec[n_segmentos];

Pixy pixy;

// Função para converter as coordenadas de pixels para centímetros
float pixelsParaCm(int pixels)
{
  // Considerando que o formato óptico é 1/6-inch e o tamanho do pixel é 1.9 µm x 1.9 µm
  // Calcular o tamanho do sensor em milímetros (2.46 mm horizontal)
  float tamanhoSensorX = 2.46;

  // Calcular o número de pixels por milímetro
  float pixelsPorMmX = 1.0 / (1.9 / 1000.0);

  // Converter pixels para centímetros
  float cm = pixels / pixelsPorMmX / 10.0;

  return cm;
}

void setup()
{
  Serial.begin(9600);
  Serial.print("Iniciando...\n");
  pixy.init();
}

void loop()
{
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];

  // Capturar blocos!
  blocks = pixy.getBlocks();

  // Se houver blocos detectados, imprimi-los!
  if (blocks)
  {
    int x = 0;
    int y = 0;
    i++;

    // Fazer isso (imprimir) a cada 50 quadros porque imprimir todos os
    // blocos para todos os 50 quadros por segundo sobrecarregaria a porta serial do Arduino.
    if (i % 50 == 0)
    {
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.println();
      x = 0;
      y = 0;
      for (j = 1; j <= blocks; j++)
      {
        Serial.println(j);
        if (x < pixy.blocks[j].width)
          x = pixy.blocks[j].width;
        if (y < pixy.blocks[j].height)
          y = pixy.blocks[j].height;
      }
      float r = y / sin(atan(x / y));
      float angulo = 1000000 * acos(1 - ((segmento ^ 2) / (2 * r * r)));
      for (int i = 0; i < n_segmentos; i++)
      {
        x_vec[i] = r * (1 - cos(i * angulo / 1000000));
        y_vec[i] = r * sin(i * angulo / 1000000);
        //Serial.println(x_vec[i]*x_vec[i]+y_vec[i]*y_vec[i]);
      }
      Serial.println(pixelsParaCm(x));
      Serial.println(pixelsParaCm(y));
    }
  }
}
