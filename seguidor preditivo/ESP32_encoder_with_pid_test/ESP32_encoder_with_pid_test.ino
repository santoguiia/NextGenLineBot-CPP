#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

TaskHandle_t InterruptCore;

// Para leitura Serial ************************************
#define TX_PIN 12  // Porta de transmissão do ESP escravo
#define RX_PIN 13  // Porta de recepção do escravo
byte mainVel = 40;
int valor2 = 0;
int valor1 = 40;
bool finished = 0;

// Para os motores ************************************
#define START   23
#define SENSOR   22
int Sensorpassing = 0;
bool prevSensor = 1, Sensor = 0, Pressed = 0, prevPressed = 0;

#define STANDBY   5

#define ENCODER_A   32  // The two encoder outputs to read velocity!
#define ENCODER_B   33
#define ENA 15   //PWM RIGHT MOTOR   (should be const int PWM_PIN ? ?)
#define ForwardA 2    //HIGH --> MOTOR FORWARD
#define BackwardA 4   //HIGH --> MOTOR BACKWARD

#define ENCODER_C   35
#define ENCODER_D   34
#define ENB 21      //PWM RIGHT MOTOR
#define ForwardB 18
#define BackwardB 19

// These let us convert ticks-to-RPM
#define GEARING     10 //how many encoder rotations befor the main shaft rotates
#define ENCODERMULT 28 //ticks per encoder rotation
volatile float RPSA = 0, RPSB = 0;
volatile uint32_t lastA1 = 0, lastA2 = 0;
volatile bool motordir1 = false, motordir2 = false;
int pulseCountA = 0, pulseCountB = 0;  // Added this line to keep track of the pulse count

// Para contagem de tempo ************************************
unsigned long controlTimer = millis();
unsigned long crossingline = millis();
unsigned long bouncetime = millis();

// Control PI variables
#define T 50  // time interval for PID calculation  //const unsigned long??
int Kp[] = {1.3, 1.3}; // proportional gain
int Ki[] = {5.4, 5.4}; // integral gain
float Kd[] = {.007, .007};
float Error[2], Up[2], Ui[2], Ud[2], Uiant[] = {30, 30}, RPSf[2], RPSfant[2], usat[2];
#define KA 0.0075
int motorSpeed[2];      // Initial motor speed

// Definição sinal de referência
float dw[] = { -100, 100}; //RPS
// Inicialização de varíaveis
int j = 0, k = 0;
bool power = 0;

// setting PWM properties
const int freq = 30000;
const int PWM_PIN1 = 0;
const int PWM_PIN2 = 1;
const int resolution = 8;

volatile int lastEncodedA = 0, lastEncodedB = 0; // Here updated value of encoder store.

void IRAM_ATTR interruptA() {
  //  int MSB = digitalRead(ENCODER_A); //MSB = most significant bit
  //  int LSB = digitalRead(ENCODER_B); //LSB = least significant bit
  pulseCountA ++;

  //  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  //  int sum  = (lastEncodedA << 2) | encoded; //adding it to the previous encoded value
  //  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCountA --;
  //  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pulseCountA ++;
  //  lastEncodedA = encoded; //store this value for next time

  //  Serial.println(pulseCountA);
}
void IRAM_ATTR interruptB() {
  int MSB = digitalRead(ENCODER_C); //MSB = most significant bit
  int LSB = digitalRead(ENCODER_D); //LSB = least significant bit

  int encoded = (MSB << 1) | LSB; //converting the 2 pin value to single number
  int sum  = (lastEncodedB << 2) | encoded; //adding it to the previous encoded value

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) pulseCountB --;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) pulseCountB ++;

  lastEncodedB = encoded; //store this value for next time
  //  Serial.println(pulseCountB);
}

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 2; i++) {
    Error[i] = 0;
    Up[i] = 0;
    Ui[i] = 0;
    Ud[i] = 0;
    RPSf[i] = 0;
    RPSfant[i] = 0;
    usat[i] = 0;
    motorSpeed[i] = 0;

    SerialBT.begin("Pé");
    Serial.println("The device started, now you can pair it with bluetooth!");
    delay(1000);
  }

  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_C, INPUT_PULLUP);
  pinMode(ENCODER_D, INPUT_PULLUP);
  pinMode(SENSOR, INPUT);

  pinMode(STANDBY, OUTPUT);
  pinMode(ForwardA, OUTPUT);
  pinMode(BackwardA, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ForwardB, OUTPUT);
  pinMode(BackwardB, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(STANDBY, HIGH);

  // configure PWM functionalitites
  ledcSetup(PWM_PIN1, freq, resolution);
  ledcSetup(PWM_PIN2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(ENA, PWM_PIN1);
  ledcAttachPin(ENB, PWM_PIN2);

  //create a task that will be executed in the InterruptCode() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    InterruptCode,   /* Task function. */
    "InterruptCore",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    NULL,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */

  delay(100);
  //  Serial.print("beginn running on core ");
  //  Serial.println(xPortGetCoreID());
}

void InterruptCode( void * pvParameters ) {
  //  Serial.print("InterruptTask running on core ");
  //  Serial.println(xPortGetCoreID());

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interruptA, RISING);
  //  attachInterrupt(digitalPinToInterrupt(ENCODER_B), interruptA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_C), interruptB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_D), interruptB, CHANGE);

  //while (true) {
  //vTaskDelay(pdMS_TO_TICKS(10)); // Adjust the delay time as needed
  //}
  vTaskDelete(NULL);
}

void loop() {
  unsigned long currentTime = millis();
  Pressed = digitalRead(START);
  if ((prevPressed != Pressed) and (bouncetime + 1000 < currentTime)) {
    prevPressed = Pressed;
    if (Pressed == 1) {
      power = !power;
    }
    bouncetime = currentTime;
    prevPressed = Pressed;
  }

  if (power) {
    // Track elapsed time
    if (currentTime - controlTimer >= T) {
      //      Serial.print(analogRead(SENSOR));
      controlTimer = currentTime;  // Reset the loop timer

      k++;
      if (k >= 50) {
        k = 0;
        j++;
        if (j >= 2) j = 0;
      }

      if (SerialBT.available()) {
        String str = "";
        while (SerialBT.available()) {
          char c = SerialBT.read();
          str += c;
        }
        float f = atof(str.c_str());
        if (f == 1) {
          SerialBT.print(valor1);
          SerialBT.print(" ");
          SerialBT.println(valor2);
        }
        else {
          valor1 = f;
        }
      }

      float revA = pulseCountA * 1000 / 30;          // rev per ms * 1000 = sec
      revA /= GEARING;             // account for gear ratio
      revA /= ENCODERMULT;         // account for multiple ticks per rotation
      revA /= 6.283185307; // convert from Rad/s to RPS
      RPSA = revA * 4 * 125; //account for different tics...
      pulseCountA = 0;
      //        Serial.println(RPSA);

      float revB = pulseCountB * 1000 / 30;          // rev per ms * 1000 = sec
      revB /= GEARING;             // account for gear ratio
      revB /= ENCODERMULT;         // account for multiple ticks per rotation
      revB /= 6.283185307; // convert from Rad/s to RPS
      RPSB = revB * 125;
      pulseCountB = 0;
      //    Serial.println(RPSB);


      if (Serial.available() >= 1) {
        //      Serial.readBytes((uint8_t*)&valor1, sizeof(valor1));  //wd
        //      Serial.readBytes((uint8_t*)&valor2, sizeof(valor2));  //dw
        //      valor1 = Serial.read();
        //      valor2 = Serial.read();
        int communication = Serial.read();
        if (communication != 254) {
          valor2 = communication - 127;
        } else {
          crossingline = currentTime;
          valor2 = 0;
          SerialBT.println("x");
        }
        //            Serial.print(valor1);                  Serial.print(" ");
        Serial.println(valor2);
      }

      //      if (finished == 0) {
      //        valor1 ++;
      //        valor2 = 0;
      //        if (valor1 == mainVel) {
      //          finished = 1;
      //        }
      //      }

      Sensor = digitalRead(SENSOR);
      if ((prevSensor == 0) and (Sensor == 1) and (crossingline + 3000 < currentTime)) {
        Sensorpassing++;
        SerialBT.println(Sensorpassing);
        if (Sensorpassing == 2) {
          SerialBT.println("S");
          delay(1000);
          power = 0;
        }
      }
      prevSensor = Sensor;

      //    valor2 = 0;
      //      if (valor2 > 0) {
      //        dw[1] = valor1;
      //        dw[0] = valor1 - (valor2) * 1;
      //      }
      //      else {
      //        dw[1] = valor1 + (valor2) * 1;
      //        dw[0] = valor1;
      //      }

      dw[0] = valor1 - (valor2) * 1;
      dw[1] = valor1 + (valor2) * 1;
      dw[0] = -dw[0];
      dw[1] = -dw[1];

      if (dw[0] >= 127) dw[0] = 127;
      if (dw[1] >= 127) dw[1] = 127;

      //PID motor 1:
      //     filtro passa-baixas:
      RPSfant[0] = RPSf[0];
      RPSf[0] = 0.5 * RPSA + 0.5 * RPSf[0];

      if (dw[0] < 0) {
        moveBackwardA();
        motorSpeed[0] = -motorSpeed[0];
      } else {
        moveForwardA();
      }

      //controle:
      Error[0] = abs(dw[0]) - RPSf[0];

      Up[0] = Kp[0] * Error[0];
      Ui[0] = Uiant[0] + Ki[0] * Error[0] * T / 1000;
      Ud[0] = Kd[0] * (RPSf[0] - RPSfant[0]) / T * 1000;

      Ui[0] = constrain(Ui[0], -250, 250); //evitar windup
      Uiant[0] = Ui[0];

      motorSpeed[0] = Ui[0] + Up[0] - Ud[0];

      //    // antiwindup
      //    usat = motorSpeed;
      //    if (usat > 255) usat = 255;
      //    if (usat < 0) usat  = 0;
      //    motorSpeed = motorSpeed - KA * (motorSpeed - usat);

      motorSpeed[0] = constrain(motorSpeed[0], 0, 250);

      ledcWrite(PWM_PIN1, motorSpeed[0]);
      //                Serial.print(dw[j]);    Serial.print(" ");
      //                Serial.print(Error[0]);    Serial.print(" ");
      //                Serial.print(RPSf[0]);    Serial.print(" ");
      //                //        Serial.print(Up[0]);    Serial.print(" ");
      //                //        Serial.print(Ui[0]);    Serial.print(" ");
      //                //        Serial.print(Ud[0]);    Serial.print(" ");
      //                Serial.println(motorSpeed[0]);    Serial.print(" ");

      //    //PID motor 2:
      //     filtro passa-baixas:
      RPSfant[1] = RPSf[1];
      RPSf[1] = 0.5 * RPSB + 0.5 * RPSf[1];

      //controle:
      Error[1] = dw[1] - RPSf[1];
      Up[1] = Kp[1] * Error[1];
      Ui[1] = Uiant[1] + Ki[1] * Error[1] * T / 1000;
      Ud[1] = Kd[1] * (RPSf[1] - RPSfant[1]) / T * 1000;

      Ui[1] = constrain(Ui[1], -250, 250); //evitar windup
      Uiant[1] = Ui[1];

      motorSpeed[1] = Ui[1] + Up[1] - Ud[1];

      motorSpeed[1] = constrain(motorSpeed[1], -250, 250);
      //      Serial.print(dw[1]);    Serial.print(" ");
      //      Serial.print(Error[1]);    Serial.print(" ");
      //      Serial.print(RPSf[1]);    Serial.print(" ");
      //      Serial.print(Up[1]);    Serial.print(" ");
      //      Serial.print(Ui[1]);    Serial.print(" ");
      //      Serial.print(Ud[1]);    Serial.print(" ");
      //      Serial.print(motorSpeed[1]);    Serial.print(" ");
      //      Serial.println();
      if (motorSpeed[1] < 0) {
        moveBackwardB();
        motorSpeed[1] = -motorSpeed[1];
      } else {
        moveForwardB();
      }
      ledcWrite(PWM_PIN2, motorSpeed[1]);
      //      Serial.print(dw[1]);    Serial.print(" ");
      //      Serial.print(Error[1]);    Serial.print(" ");
      //      Serial.print(RPSf[1]);    Serial.print(" ");
      //      //    Serial.print(Up[1]);    Serial.print(" ");
      //      //    Serial.print(Ui[1]);    Serial.print(" ");
      //      //    Serial.print(Ud[1]);    Serial.print(" ");
      //      Serial.print(motorSpeed[1]);    Serial.print(" ");
      //      Serial.println();
    }
  } else {
    //    delay(1000);
    stopA();
    stopB();
    dw[0] = 255;
    dw[1] = 255;
    Sensorpassing = 0;
    finished = 0;
    //    valor1 = 0;

    //    ledcWrite(PWM_PIN1, 0);
    //    ledcWrite(PWM_PIN2, 0);
  }
}


void moveForwardA() {
  digitalWrite(BackwardA, HIGH);
  digitalWrite(ForwardA, LOW);
}
void moveBackwardA() {
  digitalWrite(ForwardA, HIGH);
  digitalWrite(BackwardA, LOW);
}
void stopA() {
  digitalWrite(ForwardA, HIGH);
  digitalWrite(BackwardA, HIGH);
}
void moveForwardB() {
  digitalWrite(ForwardB, HIGH);
  digitalWrite(BackwardB, LOW);
}
void moveBackwardB() {
  digitalWrite(BackwardB, HIGH);
  digitalWrite(ForwardB, LOW);
}
void stopB() {
  digitalWrite(ForwardB, HIGH);
  digitalWrite(BackwardB, HIGH);
}
