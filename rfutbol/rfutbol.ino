/*
 * Proyecto: Control de Robot FUTBOL mediante Bluetooth
 * Autores: Miranda Francisco, Mirabile Emiliano
 * Institución: Colegio Secundario Tomas Alva Edison
 * Año: 2025
 * Descripción: Código para controlar un robot fútbol utilizando un ESP32 D1 WEMOS MINI,
 * mediante comunicación Bluetooth.
 */

// Incluir la librería para la comunicación Bluetooth con el ESP32
#include "BluetoothSerial.h"

// Configuración de Bluetooth
const char *pin = "1234";  // PIN de conexión Bluetooth
String device_name = "MaradonaTAE2026";  // Nombre del dispositivo Bluetooth

// Definir si se va a usar PIN de seguridad (Descomentar la línea de abajo si quieres activarlo)
#define USE_PIN 

// Verificación de que el Bluetooth esté habilitado
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;  // Objeto para la comunicación Bluetooth

//////////////////////////////////////////////////////////////

// DEFINICIÓN DE PINES
// Definición de pines para el control de los motores
#define pinMotor1A 16   // Pin para el motor 1 (sentido A)
#define pinMotor1B 17   // Pin para el motor 1 (sentido B)
#define pinMotor2A 21   // Pin para el motor 2 (sentido A)
#define pinMotor2B 22   // Pin para el motor 2 (sentido B)

// Definición de pines para el control de luces (LEDs)
#define led_interno 2   // Pin asociado al LED interno
#define pinLedRojo 18   // ¡NUEVO! Ajusta al pin real de tu robot
#define pinLedVerde 19  // ¡NUEVO! Ajusta al pin real de tu robot

// Variable para almacenar la señal recibida por Bluetooth
char btSignal;

// Velocidad inicial del robot (en porcentaje)
int Speed = 100;

// ============================================================
// CALIBRACIÓN DE MOTORES  <-- CORREGÍ ESTOS VALORES A MANO
// 1.0 = sin corrección | 0.90 = 90% de potencia
// Se reduce SIEMPRE el motor que va más rápido (nunca más de 1.0)
// ============================================================
float factorM1 = 1.0;
float factorM2 = 1.0;

// Declaración de la función setMotors para evitar advertencias de compilación
void setMotors(float m1, float m2);

// Configuración inicial del sistema
void setup() {
  // Inicialización de la comunicación serial
  Serial.begin(115200);
  
  // Inicialización del Bluetooth con el nombre del dispositivo
  SerialBT.begin(device_name); 
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

  // Configuración del PIN de Bluetooth si está habilitado
  #ifdef USE_PIN
    // CORREGIDO: Se agregó el número 4 como segundo argumento (longitud de "1234")
    SerialBT.setPin(pin, 4);
    Serial.println("Using PIN");
  #endif
  
  // Configuración de los pines de los motores y LEDs como salidas
  pinMode(pinMotor1A, OUTPUT);
  pinMode(pinMotor1B, OUTPUT);
  pinMode(pinMotor2A, OUTPUT);
  pinMode(pinMotor2B, OUTPUT);
  
  pinMode(led_interno, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);
  pinMode(pinLedVerde, OUTPUT);

  delay(100);  // Breve espera para la configuración
}

void loop() { 
  // Comprobación de si hay datos disponibles desde el Bluetooth
  while (SerialBT.available()) {
    btSignal = SerialBT.read();  // Leer el dato recibido
    Serial.println(btSignal);    // Imprimir la señal recibida en el monitor serial

    // Control de la velocidad del robot según la señal recibida
    if (btSignal == '0') Speed = 50;
    if (btSignal == '1') Speed = 55;
    if (btSignal == '2') Speed = 60;
    if (btSignal == '3') Speed = 65;
    if (btSignal == '4') Speed = 70;
    if (btSignal == '5') Speed = 75;
    if (btSignal == '6') Speed = 80;
    if (btSignal == '7') Speed = 85;
    if (btSignal == '8') Speed = 90;
    if (btSignal == '9') Speed = 95;
    if (btSignal == 'q') Speed = 100;

    // Control del movimiento del robot según la señal recibida
    // Movimiento hacia atrás
    if (btSignal == 'B') {
      setMotors(-Speed, -Speed);  // Dirección inversa
    }
    // Movimiento hacia adelante
    else if (btSignal == 'F') {
      setMotors(Speed, Speed);    // Dirección hacia adelante
    }
    // Giro hacia la izquierda
    else if (btSignal == 'L') {
      setMotors(-0.45 * Speed, 0.95 * Speed);  // Giro hacia la izquierda
    }
    // Giro hacia la derecha
    else if (btSignal == 'R') {
      setMotors(0.95 * Speed, -0.45 * Speed);  // Giro hacia la derecha
    }
    // Detener el robot
    else if (btSignal == 'S') {
      setMotors(0, 0);  // Detener motores
    }
    // Adelante - Derecha
    else if (btSignal == 'G') {
      setMotors(0.60 * Speed, Speed);  // Movimiento hacia adelante con giro a la derecha
    }
    // Atras - Derecha
    else if (btSignal == 'H') {
      setMotors(-0.50 * Speed, -Speed);  // Movimiento hacia atrás con giro a la derecha
    }
    // Adelante - Izquierda
    else if (btSignal == 'I') {
      setMotors(Speed, 0.60 * Speed);  // Movimiento hacia adelante con giro a la izquierda
    }
    // Atras - Izquierda
    else if (btSignal == 'J') {
      setMotors(-Speed, -0.50 * Speed);  // Movimiento hacia atrás con giro a la izquierda
    }
    // Encender la luz verde
    else if (btSignal == 'W') {
      analogWrite(pinLedRojo, 0);    // Apagar el LED rojo
      analogWrite(pinLedVerde, 255); // Encender el LED verde
    }
    // Apagar la luz verde
    else if (btSignal == 'w') {
      analogWrite(pinLedRojo, 255);  // Encender el LED rojo
      analogWrite(pinLedVerde, 0);   // Apagar el LED verde
    }
    // Encender la luz interna
    else if (btSignal == 'U') {
      digitalWrite(led_interno, HIGH); // Encender el LED interno
    }
    // Apagar la luz interna
    else if (btSignal == 'u') {
      digitalWrite(led_interno, LOW); // Apagar el LED interno
    }

    // --- Calibración por Bluetooth (opcional, se puede borrar este bloque) ---
    // K/k = bajar/subir potencia del motor 1 | N/n = bajar/subir potencia del motor 2
    else if (btSignal == 'K') {  // Motor 1: bajar potencia
      factorM1 = constrain(factorM1 - 0.02, 0.50, 1.0);
      SerialBT.printf("M1=%.2f  M2=%.2f\n", factorM1, factorM2);
    }
    else if (btSignal == 'k') {  // Motor 1: subir potencia
      factorM1 = constrain(factorM1 + 0.02, 0.50, 1.0);
      SerialBT.printf("M1=%.2f  M2=%.2f\n", factorM1, factorM2);
    }
    else if (btSignal == 'N') {  // Motor 2: bajar potencia
      factorM2 = constrain(factorM2 - 0.02, 0.50, 1.0);
      SerialBT.printf("M1=%.2f  M2=%.2f\n", factorM1, factorM2);
    }
    else if (btSignal == 'n') {  // Motor 2: subir potencia
      factorM2 = constrain(factorM2 + 0.02, 0.50, 1.0);
      SerialBT.printf("M1=%.2f  M2=%.2f\n", factorM1, factorM2);
    }
  }
}

// Función para controlar los motores
void setMotors(float m1, float m2) {
  // Limitar la velocidad máxima a 100%
  if (abs(m1) > 100) {
    m1 = 100 * (m1 / abs(m1));  // Normalizar la velocidad
  }
  if (abs(m2) > 100) {
    m2 = 100 * (m2 / abs(m2));  // Normalizar la velocidad
  }

  // Aplicar calibración de cada motor
  m1 *= factorM1;
  m2 *= factorM2;

  // Convertir la velocidad a valores PWM (0 a 255)
  m1 = m1 * 255 / 100;
  m2 = m2 * 255 / 100;

  // Controlar el motor 1
  if (m1 >= 0) {
    analogWrite(pinMotor1A, m1);  // Sentido A
    analogWrite(pinMotor1B, 0);   // Sentido B apagado
  } else {
    analogWrite(pinMotor1A, 0);   // Sentido A apagado
    analogWrite(pinMotor1B, abs(m1)); // Sentido B activado
  }

  // Controlar el motor 2
  if (m2 >= 0) {
    analogWrite(pinMotor2A, m2);  // Sentido A
    analogWrite(pinMotor2B, 0);   // Sentido B apagado
  } else {
    analogWrite(pinMotor2A, 0);   // Sentido A apagado
    analogWrite(pinMotor2B, abs(m2)); // Sentido B activado
  }
}
