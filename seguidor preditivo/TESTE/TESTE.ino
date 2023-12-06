TaskHandle_t InterruptCore;

// Para leitura Serial ************************************
#define TX_PIN 12  // Porta de transmissão do ESP escravo
#define RX_PIN 13  // Porta de recepção do escravo
//byte valor1 = 0;
int valor2 = 0;

// Para os motores ************************************
#define START   23
#define STANDBY   5

#define ENA 15   //PWM RIGHT MOTOR   (should be const int PWM_PIN ? ?)
#define ForwardA 2    //HIGH --> MOTOR FORWARD
#define BackwardA 4   //HIGH --> MOTOR BACKWARD

#define ENB 21      //PWM RIGHT MOTOR
#define ForwardB 18
#define BackwardB 19

// Para contagem de tempo ************************************
unsigned long timer = millis();           // Timer to track elapsed time
unsigned long loopTimer = 0;  // Timer to track the loop duration
unsigned long controlTimer = millis();

// Control PI variables
#define T 50  // time interval for PID calculation  //const unsigned long??
int Kp[] = {1.3, 1.3}; // proportional gain
int Ki[] = {5.4, 5.4}; // integral gain
float Kd[] = {.07, .07};
float Error[2], Up[2], Ui[2], Ud[2], Uiant[] = {0, 0}, RPSf[2], RPSfant[2], usat[2];
#define KA 0.0075
int motorSpeed[2];      // Initial motor speed

// Definição sinal de referência
// Inicialização de varíaveis
int j = 0, k = 0;
bool power = 0;

// setting PWM properties
const int freq = 30000;
const int PWM_PIN1 = 0;
const int PWM_PIN2 = 1;
const int resolution = 8;

volatile int lastEncodedA = 0, lastEncodedB = 0; // Here updated value of encoder store.

//void IRAM_ATTR Start() {
//  power = !power;
//}

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

    //    attachInterrupt(digitalPinToInterrupt(START), Start, RISING);

  }

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
}

void loop() {
  while (digitalRead(START) == 1) {
    power = !power;
    delay(500);
  }
  if (power) {
    // Track elapsed time
    unsigned long currentTime = millis();

    if (currentTime - controlTimer >= T) {
      controlTimer = currentTime;  // Reset the loop timer


      if (Serial.available() >= 1) {
        //      Serial.readBytes((uint8_t*)&valor1, sizeof(valor1));  //wd
        //      Serial.readBytes((uint8_t*)&valor2, sizeof(valor2));  //dw
        //      valor1 = Serial.read();
        valor2 = Serial.read();
        valor2 = valor2 - 127;
        Serial.println(valor2);

        // Faça algo com os valores recebidos
        // Por exemplo, exiba os valores no monitor serial
        //            Serial.print(valor1);                  Serial.print(" ");
      }
      int dw0 = 0 - (valor2) * 1;
      int dw1 = 0 + (valor2) * 1;
      if (dw0 >= 127) dw0 = 127;
      if (dw1 >= 127) dw1 = 127;
      //dw0 = -dw0;
      //dw1 = -dw1;


      //dw[0] = valor1 - (valor2 - 127) * 1;
      //dw[1] = valor1 + (valor2 - 127) * 1;

      dw0 = constrain(dw0, -250, 250);

      if (dw0 < 0) {
        moveBackwardA();
        dw0 = -dw0;
      } else {
        moveForwardA();
      }
      ledcWrite(PWM_PIN1, dw0);
      //      Serial.print(dw0);    Serial.print(" ");

      dw1 = constrain(dw1, -250, 250);
      if (dw1 < 0) {
        moveBackwardB();
        dw1 = -dw1;
      } else {
        moveForwardB();
      }
      ledcWrite(PWM_PIN2, dw1);
      //      Serial.println(dw1); Serial.print(" ");

    }
  } else {
    ledcWrite(PWM_PIN1, 0);
    ledcWrite(PWM_PIN2, 0);
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
