/*
 * Proyecto: Control de Robot FUTBOL
 * Autores: Cohorte 2028 Tecnicatura en Robótica
 * Institución: IES Tomas Alva Edison
 * Año: 2026
 * Descripción: Código para controlar un robot fútbol utilizando un ESP32 D1 WEMOS MINI
 * usando FreeRTOS
 *
 * Comandos: Bluetooth Serial (sin WiFi).
 * Motores: la potencia cambia con rampas de pendiente configurable (ver RAMPAS DE POTENCIA).
 *
 * Arquitectura:
 *   ComunicacionTask --> [cola de comandos] --> ControlTask --> rampa --> motores
 *   ControlTask corre con período fijo (RAMPA_PERIODO_MS): atiende los comandos
 *   pendientes (fijan el OBJETIVO de cada motor) y luego avanza la rampa un paso.
 *   Solo ControlTask toca los motores.
 */

// ============================================================
// SELECCIÓN DE FUNCIONES
// ============================================================
#define USE_PIN          // PIN de seguridad para Bluetooth
//#define DEBUG_RAMPA    // Descomentar para ver objetivo/actual de cada motor (sirve en el Serial Plotter)

// ============================================================
// RAMPAS DE POTENCIA (pendiente configurable)
// Las pendientes se expresan en % de potencia por segundo.
// Valor 0 = sin rampa (cambio inmediato).
//
// Ejemplos con 250 %/s: de 0 a 100 % tarda 0,4 s
//           con 500 %/s: de 100 a 0 % tarda 0,2 s
//           con 100 %/s: de 0 a 100 % tarda 1 s
//
// Un cambio de sentido (adelante -> atrás) primero frena hasta cero con la
// pendiente de frenado y luego acelera en el otro sentido con la de aceleración.
// ============================================================
#define RAMPA_ACEL_PCT_S    250.0f   // al aumentar la potencia (arrancar / acelerar)
#define RAMPA_DECEL_PCT_S   500.0f   // al reducir la potencia hacia cero (frenar / soltar)
#define RAMPA_PERIODO_MS    10       // cada cuántos ms se actualiza la rampa (mínimo 1)

// Librerías
#include <esp_system.h>
#include "BluetoothSerial.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

// Verificación de que el Bluetooth esté habilitado
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

// ============================================================
// CONFIGURACIÓN
// ============================================================
const char *pin = "1234";             // PIN de conexión Bluetooth
String device_name = "MessiTAE2026";  // Nombre del dispositivo Bluetooth
BluetoothSerial SerialBT;             // Objeto para la comunicación Bluetooth

//////////////////////////////////////////////////////////////

// DEFINICIÓN DE PINES
#define pinMotor1A 16   // Pin para el motor 1 (sentido A)
#define pinMotor1B 17   // Pin para el motor 1 (sentido B)
#define pinMotor2A 21   // Pin para el motor 2 (sentido A)
#define pinMotor2B 22   // Pin para el motor 2 (sentido B)

#define led_interno 2   // Pin asociado al LED interno
#define pinLedRojo 18   // ¡NUEVO! Ajusta al pin real de tu robot
#define pinLedVerde 19  // ¡NUEVO! Ajusta al pin real de tu robot

// Eventos internos (no los envía el usuario)
#define EVT_CONECTADO    0x01   // un cliente Bluetooth se conectó
#define EVT_DESCONECTADO 0x02   // el cliente Bluetooth se desconectó

// Cola FreeRTOS
QueueHandle_t colaComandos;

// Velocidad inicial del robot (en porcentaje)
int Speed = 100;

// ============================================================
// CALIBRACIÓN DE MOTORES  <-- CORREGÍ ESTOS VALORES A MANO
// 1.0 = sin corrección | 0.90 = 90% de potencia
// Se reduce SIEMPRE el motor que va más rápido (nunca más de 1.0)
// La calibración se aplica a la salida, después de la rampa.
// ============================================================
float factorM1 = 1.0;
float factorM2 = 1.0;

// ============================================================
// ESTADO DE LAS RAMPAS (en % con signo: positivo = sentido A, negativo = sentido B)
// Solo las usa ControlTask, por eso no necesitan protección.
// ============================================================
float objetivoM1 = 0, objetivoM2 = 0;   // potencia pedida por los comandos
float actualM1   = 0, actualM2   = 0;   // potencia que realmente se aplica

// Prototipos
void setMotors(float m1, float m2);
void detenerYa();
void escribirMotores(float m1, float m2);
float avanzarRampa(float actual, float objetivo);
void procesarComando(char c);
void responder(const String &s);
void BlinkTask(void *parameter);
void ComunicacionTask(void *parameter);
void ControlTask(void *parameter);

// ------------------------------------------------------------
// Eventos Bluetooth
// Se ejecutan en el contexto de la pila BT: solo encolan, nunca tocan motores.
// ------------------------------------------------------------
void btEvento(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  char c;
  if (event == ESP_SPP_SRV_OPEN_EVT) {        // un cliente se conectó
    c = EVT_CONECTADO;
    xQueueSend(colaComandos, &c, 0);
  } else if (event == ESP_SPP_CLOSE_EVT) {    // el cliente se desconectó
    c = EVT_DESCONECTADO;
    xQueueSend(colaComandos, &c, 0);
  }
}

// Configuración inicial del sistema
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Obtener la causa del reinicio
  esp_reset_reason_t causa = esp_reset_reason();
  Serial.print("Código de reinicio: ");
  Serial.println(causa);
  Serial.print("Significado: ");

  switch (causa) {
    case ESP_RST_UNKNOWN:   Serial.println("Desconocido"); break;
    case ESP_RST_POWERON:   Serial.println("Encendido por botón físico o conexión de energía (Power On)"); break;
    case ESP_RST_EXT:       Serial.println("Pin de reset externo (EN/RST)"); break;
    case ESP_RST_SW:        Serial.println("Reinicio por software (esp_restart())"); break;
    case ESP_RST_PANIC:     Serial.println("Excepción de hardware / Error crítico (Crash/Panic)"); break;
    case ESP_RST_INT_WDT:   Serial.println("Watchdog Timer interrumpido (Interrupción de reloj)"); break;
    case ESP_RST_TASK_WDT:  Serial.println("Watchdog de tareas (Una tarea bloqueó el sistema)"); break;
    case ESP_RST_WDT:       Serial.println("Otros Watchdogs"); break;
    case ESP_RST_DEEPSLEEP: Serial.println("Despertar del modo Sueño Profundo (Deep Sleep)"); break;
    case ESP_RST_BROWNOUT:  Serial.println("Caída de voltaje (Marrón/Brownout)"); break;
    case ESP_RST_SDIO:      Serial.println("Reinicio por entrada/salida SDIO"); break;
    default:                Serial.println("Causa no mapeada"); break;
  }

  // La cola se crea ANTES de iniciar Bluetooth porque su callback la usa
  colaComandos = xQueueCreate(32, sizeof(char));
  if (colaComandos == NULL) {
    Serial.println("ERROR: no se pudo crear la cola de comandos");
    while (true) delay(1000);
  }

  // Pines de motores y LEDs
  pinMode(pinMotor1A, OUTPUT);
  pinMode(pinMotor1B, OUTPUT);
  pinMode(pinMotor2A, OUTPUT);
  pinMode(pinMotor2B, OUTPUT);

  pinMode(led_interno, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);
  pinMode(pinLedVerde, OUTPUT);
  detenerYa();  // arranca detenido

  // Configuración del PIN de Bluetooth si está habilitado
  #ifdef USE_PIN
    SerialBT.setPin(pin, 4);
    Serial.println("Usando PIN");
  #endif
  SerialBT.register_callback(btEvento);
  if (!SerialBT.begin(device_name)) {
    Serial.println("ERROR: Bluetooth no inició");
  }
  Serial.printf("Bluetooth activo con el nombre \"%s\"\n", device_name.c_str());

  // Tareas FreeRTOS
  xTaskCreatePinnedToCore(BlinkTask,        "BlinkTask",        2048, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(ComunicacionTask, "ComunicacionTask", 3072, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(ControlTask,      "ControlTask",      6144, NULL, 5, NULL, 1);

  delay(100);  // Breve espera para la configuración
}

void loop() {
  vTaskDelay(portMAX_DELAY);  // todo el trabajo lo hacen las tareas
}

// ------------------------------------------------------------
// Tarea blink para verificar ejecución
// ------------------------------------------------------------
void BlinkTask(void *parameter) {
  for (;;)
  {
    digitalWrite(led_interno, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(led_interno, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ------------------------------------------------------------
// Tarea de comunicación: lee Bluetooth y manda todo a la cola
// ------------------------------------------------------------
void ComunicacionTask(void *parameter) {
  char c;
  for (;;) {
    while (SerialBT.available()) {
      c = SerialBT.read();
      xQueueSend(colaComandos, &c, 0);
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}

// ------------------------------------------------------------
// Tarea de control: único lugar donde se manejan los motores.
// Corre con período fijo RAMPA_PERIODO_MS (vTaskDelayUntil), así la
// pendiente de las rampas es exacta aunque lleguen muchos comandos.
// Cada ciclo: 1) atiende los comandos pendientes (fijan el objetivo)
//             2) avanza un paso cada rampa
//             3) escribe la salida a los motores
// ------------------------------------------------------------
void ControlTask(void *parameter) {
  char c;
  TickType_t ultimoCiclo = xTaskGetTickCount();

#ifdef DEBUG_RAMPA
  uint32_t contador = 0;
  const uint32_t cada = (100 / RAMPA_PERIODO_MS) ? (100 / RAMPA_PERIODO_MS) : 1;   // ~10 muestras por segundo
#endif

  for (;;) {
    // 1) Comandos pendientes
    while (xQueueReceive(colaComandos, &c, 0) == pdTRUE) {
      procesarComando(c);
    }

    // 2) Rampa de cada motor
    actualM1 = avanzarRampa(actualM1, objetivoM1);
    actualM2 = avanzarRampa(actualM2, objetivoM2);

    // 3) Salida
    escribirMotores(actualM1, actualM2);

#ifdef DEBUG_RAMPA
    if (++contador % cada == 0) {
      Serial.printf("M1_obj:%.1f,M1_act:%.1f,M2_obj:%.1f,M2_act:%.1f\n",
                    objetivoM1, actualM1, objetivoM2, actualM2);
    }
#endif

    vTaskDelayUntil(&ultimoCiclo, pdMS_TO_TICKS(RAMPA_PERIODO_MS));
  }
}

// ------------------------------------------------------------
// Avanza un paso de rampa desde 'actual' hacia 'objetivo' (valores en % con signo).
// - Si la potencia se acerca a cero (frenar) usa RAMPA_DECEL_PCT_S.
// - Si se aleja de cero o arranca desde cero (acelerar) usa RAMPA_ACEL_PCT_S.
// - Pendiente 0 = sin rampa.
// ------------------------------------------------------------
float avanzarRampa(float actual, float objetivo) {
  float pendiente;

  if (actual == objetivo) return actual;

  bool haciaCero = (actual > 0 && objetivo < actual) || (actual < 0 && objetivo > actual);
  if (haciaCero) pendiente = RAMPA_DECEL_PCT_S;
  else pendiente = RAMPA_ACEL_PCT_S;
  
  if (pendiente <= 0) return objetivo;                      // Salto directo, sin rampa

  float paso  = RAMPA_PERIODO_MS * (pendiente / 1000.0f);   // % que se puede mover en este ciclo
  float delta = objetivo - actual;
  if (fabsf(delta) <= paso) return objetivo;                // falta menos de un paso

  if (delta > 0) return actual + paso;
  else return actual - paso;
}

// Envía texto al cliente Bluetooth conectado
void responder(const String &s) {
  if (SerialBT.hasClient()) SerialBT.print(s);
}

// ------------------------------------------------------------
// Interpretación de comandos Bluetooth
// Los comandos de movimiento solo fijan el OBJETIVO; la rampa lo alcanza gradualmente.
// ------------------------------------------------------------
void procesarComando(char btSignal) {
  if (btSignal == EVT_CONECTADO) {
    detenerYa();
    Speed = 100;                 // estado inicial conocido
    Serial.println("BT conectado");
    return;
  }
  if (btSignal == EVT_DESCONECTADO) {
    detenerYa();                 // por seguridad, parada inmediata (sin rampa)
    Serial.println("BT desconectado: motores detenidos");
    return;
  }

  Serial.println(btSignal);

  // Control de la velocidad
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

  // Movimiento
  if (btSignal == 'B') {
    setMotors(-Speed, -Speed);
  }
  else if (btSignal == 'F') {
    setMotors(Speed, Speed);
  }
  else if (btSignal == 'L') {
    setMotors(-0.45 * Speed, 0.95 * Speed);
  }
  else if (btSignal == 'R') {
    setMotors(0.95 * Speed, -0.45 * Speed);
  }
  else if (btSignal == 'S') {
    setMotors(0, 0);             // frena con la rampa de frenado
  }
  else if (btSignal == 'G') {          // Adelante - Derecha
    setMotors(0.50 * Speed, Speed);
  }
  else if (btSignal == 'H') {          // Atrás - Derecha
    setMotors(-0.50 * Speed, -Speed);
  }
  else if (btSignal == 'I') {          // Adelante - Izquierda
    setMotors(Speed, 0.50 * Speed);
  }
  else if (btSignal == 'J') {          // Atrás - Izquierda
    setMotors(-Speed, -0.50 * Speed);
  }
  // Luces
  else if (btSignal == 'W') {
    analogWrite(pinLedRojo, 0);
    analogWrite(pinLedVerde, 255);
  }
  else if (btSignal == 'w') {
    analogWrite(pinLedRojo, 255);
    analogWrite(pinLedVerde, 0);
  }
  else if (btSignal == 'U') {
    digitalWrite(led_interno, HIGH);
  }
  else if (btSignal == 'u') {
    digitalWrite(led_interno, LOW);
  }
  // Calibración (K/k = bajar/subir motor 1 | N/n = bajar/subir motor 2)
  else if (btSignal == 'K') {
    factorM1 = constrain(factorM1 - 0.02, 0.50, 1.0);
    responder(String("M1=") + String(factorM1, 2) + "  M2=" + String(factorM2, 2) + "\n");
  }
  else if (btSignal == 'k') {
    factorM1 = constrain(factorM1 + 0.02, 0.50, 1.0);
    responder(String("M1=") + String(factorM1, 2) + "  M2=" + String(factorM2, 2) + "\n");
  }
  else if (btSignal == 'N') {
    factorM2 = constrain(factorM2 - 0.02, 0.50, 1.0);
    responder(String("M1=") + String(factorM1, 2) + "  M2=" + String(factorM2, 2) + "\n");
  }
  else if (btSignal == 'n') {
    factorM2 = constrain(factorM2 + 0.02, 0.50, 1.0);
    responder(String("M1=") + String(factorM1, 2) + "  M2=" + String(factorM2, 2) + "\n");
  }
}

// ------------------------------------------------------------
// Fija el OBJETIVO de potencia de cada motor (%, con signo).
// No mueve los motores directamente: lo hace ControlTask con la rampa.
// ------------------------------------------------------------
void setMotors(float m1, float m2) {
  objetivoM1 = constrain(m1, -100, 100);
  objetivoM2 = constrain(m2, -100, 100);
}

// Parada inmediata, saltando la rampa (conexión/desconexión y arranque)
void detenerYa() {
  objetivoM1 = objetivoM2 = 0;
  actualM1 = actualM2 = 0;
  escribirMotores(0, 0);
}

// ------------------------------------------------------------
// Escribe la potencia (%, con signo) en los pines de los motores.
// Aplica la calibración de cada motor y convierte a PWM (0 a 255).
// ------------------------------------------------------------
void escribirMotores(float m1, float m2) {
  // Aplicar calibración de cada motor
  m1 *= factorM1;
  m2 *= factorM2;

  // Convertir la velocidad a valores PWM (0 a 255)
  int pwm1 = (int)lroundf(constrain(m1, -100, 100) * 255.0f / 100.0f);
  int pwm2 = (int)lroundf(constrain(m2, -100, 100) * 255.0f / 100.0f);

  // Controlar el motor 1
  if (pwm1 >= 0) {
    analogWrite(pinMotor1A, pwm1);    // Sentido A
    analogWrite(pinMotor1B, 0);       // Sentido B apagado
  } else {
    analogWrite(pinMotor1A, 0);       // Sentido A apagado
    analogWrite(pinMotor1B, -pwm1);   // Sentido B activado
  }

  // Controlar el motor 2
  if (pwm2 >= 0) {
    analogWrite(pinMotor2A, pwm2);    // Sentido A
    analogWrite(pinMotor2B, 0);       // Sentido B apagado
  } else {
    analogWrite(pinMotor2A, 0);       // Sentido A apagado
    analogWrite(pinMotor2B, -pwm2);   // Sentido B activado
  }
}
