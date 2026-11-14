/*
 * Proyecto: Control de Robot FUTBOL (versión JOYSTICK ANALÓGICO)
 * Basado en el código original de: Cohorte 2028 Tecnicatura en Robótica
 * Institución: IES Tomas Alva Edison
 * Año: 2026
 * Placa: ESP32 NodeMCU (ESP-WROOM-32) + puente H L298N + FreeRTOS + Bluepad32
 * Entorno: Arduino IDE 2.3.10 + paquete esp32_bluepad32 4.1.0 (core Arduino-ESP32 2.0.17)
 *
 * CONTROLES
 *   Stick IZQUIERDO (arriba/abajo) .... acelerar / retroceder (proporcional)
 *   Stick DERECHO (izquierda/derecha) . girar a la izquierda / derecha (proporcional)
 *   L1 / R1 ........................... bajar / subir MODO de velocidad (ECO, NORMAL, TURBO)
 *   Cuadrado / Triángulo .............. calibración motor 1: bajar / subir
 *   Cruz / Círculo .................... calibración motor 2: bajar / subir
 *   L3 + R3 (clic de ambos sticks) .... borrar calibración (vuelve a 1.00)
 *   D-pad izquierda / derecha ......... luz verde / luz roja
 *   Share + Options (mantener 3 s) .... DESVINCULAR el joystick (Xbox: View + Menu)
 *   (Xbox: X=Cuadrado, Y=Triángulo, A=Cruz, B=Círculo)
 *
 * VINCULACIÓN (el robot obedece a UN solo joystick)
 *   - La primera vez, el primer joystick que se conecta queda VINCULADO y se guarda en la memoria.
 *   - Desde ahí, cualquier otro joystick que intente conectarse es rechazado, aunque esté más cerca.
 *   - Para cambiar de joystick: mantener Share + Options 3 segundos en el joystick vinculado
 *     (o escribir "olvidar" en el Monitor Serie). El próximo joystick que se conecte queda vinculado.
 *   - No hace falta escribir ninguna MAC a mano.
 *
 * ARQUITECTURA (FreeRTOS)
 *
 *   ComunicacionTask (prio 4)            ControlTask (prio 5, período fijo 10 ms)
 *   ---------------------------          ---------------------------------------
 *   lee el joystick (Bluepad32)  --->    atiende la cola --> fija OBJETIVO de cada motor
 *   zona muerta + curva + mezcla  cola   rampa (arranque progresivo) --> calibración --> PWM
 *   modo de velocidad            Comando watchdog: sin señal => frena
 *
 *   - ComunicacionTask INTERPRETA a la persona: stick -> potencia deseada en %.
 *   - ControlTask es el único que toca los motores: aplica rampas, calibración y seguridad.
 *
 *   [SO] TELEMETRÍA (agregado para Sistemas Operativos)
 *   Timer de hardware (100 ms) --> ISR --> semáforo binario --> TelemetriaTask (prio 3)
 *   TelemetriaTask lee los datos de los motores (protegidos con un mutex) y los
 *   envía por UART2 al módulo Bluetooth HC-06.
 *   El Bluetooth interno del ESP32 queda dedicado al joystick (Bluepad32).
 *
 *   BlinkTask: el LED azul parpadea mientras espera el joystick; al conectarse
 *   la tarea se SUSPENDE y el LED queda fijo. Al desconectarse se REANUDA.
 *
 * CONVENCIÓN: Motor 1 = rueda IZQUIERDA, Motor 2 = rueda DERECHA.
 *
 * Todo lo agregado para Sistemas Operativos está marcado con [SO].
 */

#define VERSION_FIRMWARE "2.0-SO"   // [SO] 1.x = versión de la competencia, 2.0 = con telemetría

// ============================================================
// SELECCIÓN DE FUNCIONES
// ============================================================
//#define DEBUG_RAMPA             // Descomentar para ver objetivo/actual de cada motor (Serial Plotter)
//#define DEBUG_JOYSTICK          // Descomentar para ver en el Monitor Serie lo que lee cada stick (detectar drift)
#define USAR_WATCHDOG_JOYSTICK    // Si deja de llegar señal del joystick, el robot frena (ver TIMEOUT_JOYSTICK_MS)

// ============================================================
// AJUSTES DE MANEJO (se pueden tocar probando el robot)
// ============================================================
#define ZONA_MUERTA          0.01f   // fracción del stick que se ignora (0.08 = 8 %) para evitar el drift
#define SENTIDO_AVANCE       1       // poner -1 si al empujar el stick izquierdo hacia adelante el robot retrocede
#define GANANCIA_GIRO        1.0f    // sensibilidad del giro (bajar a 0.7 si gira demasiado brusco)
#define INVERTIR_GIRO        1       // poner -1 si al mover el stick derecho a la derecha el robot gira a la izquierda
#define POTENCIA_MIN         15.0f   // % apenas sale de la zona muerta (vence la fricción; la rampa igual lo suaviza)
#define CURVA_LINEAL         0.7f    // 1.0 = respuesta lineal (más potencia a medio stick), 0.0 = muy suave al centro
#define TIMEOUT_JOYSTICK_MS  300     // sin señal durante este tiempo => frenar

// ============================================================
// MODOS DE VELOCIDAD
//   maxPct : potencia máxima con el stick a fondo (100 = PWM al máximo, 255)
//   acel   : rampa de aceleración (% por segundo) -> arranque progresivo, sin tirones
//   decel  : rampa de frenado (% por segundo)
//   r,g,b  : color de la luz del joystick (PS4/PS5) como indicador
// Ejemplo: TURBO acel 200 => tarda 0,5 s en ir de 0 a 100 %.
// Valor de rampa 0 = sin rampa (cambio inmediato).
// ============================================================
struct ModoVel {
  const char *nombre;
  float maxPct;
  float acel;
  float decel;
  uint8_t r, g, b;
};

const ModoVel MODOS[] = {
  { "ECO",     65.0f, 110.0f, 300.0f,   0, 255,   0 },   // 0 -> 65 % en ~0,6 s
  { "NORMAL",  85.0f, 160.0f, 400.0f, 255, 180,   0 },   // 0 -> 85 % en ~0,5 s
  { "TURBO",  100.0f, 200.0f, 500.0f, 255,   0,   0 },   // 0 -> 100 % en 0,5 s (todo lo que da el PWM)
};
#define N_MODOS       3
#define MODO_INICIAL  1               // arranca en NORMAL

// Librerías
#include <esp_system.h>
#include <Bluepad32.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"   // [SO] semáforos y mutex

// ============================================================
// PINES
// ============================================================
//
// Cada pin se indica con tres nombres para evitar errores al cablear:
//   - número de GPIO (el que usa el código)
//   - SERIGRAFÍA: el texto que está impreso en la placa del ESP32 (lo que hay que buscar al cablear)
//   - borne del L298N al que va
//
//   Nombre en el código | GPIO | SERIGRAFÍA en la placa | Va al L298N / función
//   --------------------+------+------------------------+--------------------------------------------
//   pinMotor1A          |   5  |  D5                    | IN1  (motor 1, rueda izquierda, sentido A)
//   pinMotor1B          |  19  |  D19                   | IN2  (motor 1, rueda izquierda, sentido B)
//   pinMotor2A          |  21  |  D21                   | IN3  (motor 2, rueda derecha, sentido A)
//   pinMotor2B          |  22  |  D22                   | IN4  (motor 2, rueda derecha, sentido B)
//   pinLedRojo          |  18  |  D18                   | LED rojo (opcional)
//   pinLedVerde         |  23  |  D23                   | LED verde (opcional)
//   led_interno         |   2  |  D2                    | LED azul de la placa (no se cablea)
//   PIN_HC06_RX  [SO]   |  16  |  RX2                   | TXD del HC-06 (telemetría)
//   PIN_HC06_TX  [SO]   |  17  |  TX2                   | RXD del HC-06 (telemetría)
//   (masa)              |  --  |  GND                   | GND del L298N  (OBLIGATORIO, masa común)
//   (alimentación)      |  --  |  VIN                   | 5V del L298N   (solo si se alimenta desde el L298N)
//
// Pines del lado DERECHO de la placa, con el USB hacia abajo, de arriba hacia abajo:
//   D23, D22, TX0, RX0, D21, D19, D18, D5, TX2, RX2, D4, D2, D15, GND, 3V3
// (el ESP32 usa el mismo pinout que el diagrama "ESP32 Dev Board Pinout")
//
// IMPORTANTE: si un cable IN del L298N queda suelto, el L298N lo lee como HIGH
// y esa rueda gira sola. Ver la nota "RUEDA QUE GIRA SOLA" más abajo.
//
#define pinMotor1A 5    // GPIO5  = D5  en la placa -> IN1 del L298N (motor 1, rueda izquierda, sentido A)
#define pinMotor1B 19   // GPIO19 = D19 en la placa -> IN2 del L298N (motor 1, rueda izquierda, sentido B)
#define pinMotor2A 21   // GPIO21 = D21 en la placa -> IN3 del L298N (motor 2, rueda derecha, sentido A)
#define pinMotor2B 22   // GPIO22 = D22 en la placa -> IN4 del L298N (motor 2, rueda derecha, sentido B)

#define led_interno 2   // GPIO2  = D2  en la placa (LED azul interno)
#define pinLedRojo 18   // GPIO18 = D18 en la placa -> LED rojo (opcional)
#define pinLedVerde 23  // GPIO23 = D23 en la placa -> LED verde (opcional)

// Bits del D-pad de Bluepad32
#define DPAD_ARRIBA     0x01
#define DPAD_ABAJO      0x02
#define DPAD_DERECHA    0x04
#define DPAD_IZQUIERDA  0x08

// ============================================================
// MENSAJES ENTRE TAREAS (cola FreeRTOS)
// ============================================================
enum TipoCmd : uint8_t {
  CMD_CONECTADO,      // se conectó el joystick
  CMD_DESCONECTADO,   // se desconectó el joystick
  CMD_MOTORES,        // potencia deseada: a = motor 1 (%), b = motor 2 (%), modo = modo de velocidad
  CMD_CALIBRAR,       // a = cambio al factor del motor 1, b = cambio al factor del motor 2
  CMD_BORRAR_CAL,     // vuelve la calibración a 1.00
  CMD_LUCES           // a = 1 luz verde, a = 0 luz roja
};

struct Comando {
  uint8_t tipo;
  uint8_t modo;
  float a;
  float b;
};

QueueHandle_t colaComandos;
ControllerPtr gamepad = nullptr;      // joystick conectado (solo usamos uno)
Preferences prefs;                    // memoria flash para guardar la calibración

// ============================================================
// VINCULACIÓN DEL JOYSTICK (se guarda en la memoria flash)
// ============================================================
Preferences prefsJoy;                 // espacio aparte en la flash para la MAC del joystick vinculado
uint8_t macVinculada[6];              // dirección Bluetooth del joystick dueño del robot
bool hayVinculo = false;              // false = el próximo joystick que se conecte queda vinculado
#define MAX_RECHAZOS 4
ControllerPtr rechazados[MAX_RECHAZOS] = {nullptr};   // joysticks ajenos a desconectar
#define TIEMPO_DESVINCULAR_MS 3000    // cuánto hay que mantener Share + Options

// ============================================================
// CALIBRACIÓN DE MOTORES (se ajusta con los botones y SE GUARDA al apagar)
// 1.0 = sin corrección | 0.90 = 90 % de potencia
// Se reduce SIEMPRE el motor que va más rápido (nunca más de 1.0)
// OJO: si quedó guardado un valor bajo, ese motor nunca llega al 100 %.
//      L3 + R3 lo vuelve a 1.00.
// ============================================================
float factorM1 = 1.0f;
float factorM2 = 1.0f;

// ============================================================
// ESTADO DE LAS RAMPAS (en % con signo: positivo = sentido A, negativo = sentido B)
// Solo las usa ControlTask, por eso no necesitan protección.
// ============================================================
float objetivoM1 = 0, objetivoM2 = 0;   // potencia pedida
float actualM1   = 0, actualM2   = 0;   // potencia que realmente se aplica
float rampaAcel  = MODOS[MODO_INICIAL].acel;    // se actualizan según el modo activo
float rampaDecel = MODOS[MODO_INICIAL].decel;
#define RAMPA_PERIODO_MS 10             // cada cuántos ms se actualiza la rampa

// ============================================================
// [SO] TELEMETRÍA POR HC-06 (timer de hardware + ISR + semáforo + mutex)
// ============================================================
#define PIN_HC06_RX            16     // GPIO16 (RX2) <- TX del HC-06
#define PIN_HC06_TX            17     // GPIO17 (TX2) -> RX del HC-06
#define BAUD_HC06              9600   // velocidad de fábrica del HC-06
#define PERIODO_TELEMETRIA_MS  100    // cada cuánto interrumpe el timer

// Datos que se envían. Los ESCRIBE ControlTask y los LEE TelemetriaTask,
// por eso están protegidos con mutexDatos (los tres campos tienen que ser del mismo instante).
struct DatosTelemetria {
  uint8_t modo;     // 0 = ECO, 1 = NORMAL, 2 = TURBO
  int     pwm1;     // PWM aplicado al motor 1 (-255 a 255, negativo = reversa)
  int     pwm2;     // PWM aplicado al motor 2
};
DatosTelemetria datosTel = {MODO_INICIAL, 0, 0};

SemaphoreHandle_t mutexDatos      = NULL;  // mutex: protege datosTel
SemaphoreHandle_t semTelemetria   = NULL;  // semáforo binario: la ISR avisa "pasaron 100 ms"
hw_timer_t       *timerTelemetria = NULL;  // timer de hardware del ESP32
TaskHandle_t      handleBlink     = NULL;  // para suspender / reanudar BlinkTask

// Solo los usa ControlTask (no necesitan protección)
uint8_t modoActual   = MODO_INICIAL;       // último modo recibido
int     pwmAplicado1 = 0;                  // último PWM escrito en cada motor
int     pwmAplicado2 = 0;

// ============================================================
// PWM compatible con core Arduino-ESP32 2.x y 3.x
//   core 3.x: se usa analogWrite() directamente
//   core 2.x: analogWrite no existe, se usa LEDC (un canal por pin)
// ============================================================
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_CORE3
#endif

#ifndef PWM_CORE3
const uint8_t pinesPwm[6] = {pinMotor1A, pinMotor1B, pinMotor2A, pinMotor2B, pinLedRojo, pinLedVerde};
#endif

void pwmInit() {
#ifndef PWM_CORE3
  for (uint8_t i = 0; i < 6; i++) {
    ledcSetup(i, 1000, 8);            // canal i, 1 kHz, 8 bits (0-255)
    ledcAttachPin(pinesPwm[i], i);
  }
#endif
}

void pwmWrite(uint8_t pin, int duty) {
#ifdef PWM_CORE3
  analogWrite(pin, duty);
#else
  for (uint8_t i = 0; i < 6; i++) {
    if (pinesPwm[i] == pin) { ledcWrite(i, duty); return; }
  }
#endif
}

// Prototipos
void setMotors(float m1, float m2);
void detenerYa();
void escribirMotores(float m1, float m2);
float avanzarRampa(float actual, float objetivo);
void procesarComando(uint8_t tipo, uint8_t modo, float a, float b);
void enviarComando(uint8_t tipo, uint8_t modo, float a, float b);
void aplicarModo(ControllerPtr g, uint8_t modo);
void desvincular();
void BlinkTask(void *parameter);
void ComunicacionTask(void *parameter);
void ControlTask(void *parameter);
void TelemetriaTask(void *parameter);      // [SO]
void iniciarTimerTelemetria();             // [SO]

// ------------------------------------------------------------
// [SO] ISR del timer: se ejecuta cada PERIODO_TELEMETRIA_MS.
// Hace lo MÍNIMO: libera el semáforo y, si con eso se despertó una tarea
// de mayor prioridad que la interrumpida, pide el cambio de contexto.
// IRAM_ATTR = el código queda en RAM interna (una ISR no puede depender de la flash).
// ------------------------------------------------------------
void IRAM_ATTR onTimerTelemetria() {
  BaseType_t despertarTarea = pdFALSE;
  xSemaphoreGiveFromISR(semTelemetria, &despertarTarea);
  if (despertarTarea == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// ------------------------------------------------------------
// [SO] Configura el timer de hardware para interrumpir cada 100 ms.
// La forma de hacerlo cambió entre el core 2.x y el 3.x del ESP32.
// ------------------------------------------------------------
void iniciarTimerTelemetria() {
#ifdef PWM_CORE3
  // core 3.x: se indica la frecuencia del timer directamente
  timerTelemetria = timerBegin(1000000);                         // 1 MHz -> 1 tick = 1 us
  if (timerTelemetria == NULL) {
    Serial.println("ERROR: no se pudo iniciar el timer de telemetría");
    return;
  }
  timerAttachInterrupt(timerTelemetria, &onTimerTelemetria);
  timerAlarm(timerTelemetria, PERIODO_TELEMETRIA_MS * 1000, true, 0);   // 100000 us, se repite siempre
#else
  // core 2.x: timer 0, divisor 80 -> 80 MHz / 80 = 1 MHz -> 1 tick = 1 us
  timerTelemetria = timerBegin(0, 80, true);
  if (timerTelemetria == NULL) {
    Serial.println("ERROR: no se pudo iniciar el timer de telemetría");
    return;
  }
  timerAttachInterrupt(timerTelemetria, &onTimerTelemetria, false);   // false: el core 2.x solo admite interrupción por nivel
  timerAlarmWrite(timerTelemetria, PERIODO_TELEMETRIA_MS * 1000, true); // 100000 us, se repite siempre
  timerAlarmEnable(timerTelemetria);
#endif
}

// ------------------------------------------------------------
// Eventos del joystick
// Se ejecutan dentro de BP32.update() (en ComunicacionTask):
// solo encolan, nunca tocan motores.
// ------------------------------------------------------------
// Anota un joystick ajeno para desconectarlo (se hace en ComunicacionTask, fuera del callback)
void rechazar(ControllerPtr ctl) {
  for (uint8_t i = 0; i < MAX_RECHAZOS; i++) {
    if (rechazados[i] == nullptr) { rechazados[i] = ctl; return; }
  }
}

void onConnectedController(ControllerPtr ctl) {
  ControllerProperties p = ctl->getProperties();
  Serial.printf("Joystick detectado: %02X:%02X:%02X:%02X:%02X:%02X\n",
                p.btaddr[0], p.btaddr[1], p.btaddr[2], p.btaddr[3], p.btaddr[4], p.btaddr[5]);

  // Ya hay un joystick manejando el robot: el segundo se rechaza
  if (gamepad != nullptr) {
    Serial.println("Ya hay un joystick en uso: se rechaza");
    rechazar(ctl);
    return;
  }

  // Hay un joystick vinculado y este no es: se rechaza
  if (hayVinculo && memcmp(p.btaddr, macVinculada, 6) != 0) {
    Serial.println("No es el joystick vinculado: se rechaza");
    rechazar(ctl);
    return;
  }

  // Sin vínculo todavía: este joystick pasa a ser el dueño del robot
  if (!hayVinculo) {
    memcpy(macVinculada, p.btaddr, 6);
    hayVinculo = true;
    prefsJoy.putBytes("mac", macVinculada, 6);
    Serial.println("Joystick VINCULADO y guardado");
  }

  gamepad = ctl;
  enviarComando(CMD_CONECTADO, 0, 0, 0);
}

void onDisconnectedController(ControllerPtr ctl) {
  for (uint8_t i = 0; i < MAX_RECHAZOS; i++) {
    if (rechazados[i] == ctl) rechazados[i] = nullptr;
  }
  if (gamepad == ctl) {
    gamepad = nullptr;
    enviarComando(CMD_DESCONECTADO, 0, 0, 0);   // frena el robot
  }
}

// Borra el vínculo: el próximo joystick que se conecte queda como dueño
void desvincular() {
  hayVinculo = false;
  prefsJoy.remove("mac");
  Serial.println("Joystick DESVINCULADO: el próximo que se conecte queda vinculado");
  if (gamepad) {
    gamepad->playDualRumble(0, 400, 255, 255);   // vibración larga como aviso
    gamepad->disconnect();
  }
}

// Configuración inicial del sistema
void setup() {
  // Lo PRIMERO: pines de motores en LOW, así ninguna rueda se mueve mientras arranca la placa
  pinMode(pinMotor1A, OUTPUT); digitalWrite(pinMotor1A, LOW);
  pinMode(pinMotor1B, OUTPUT); digitalWrite(pinMotor1B, LOW);
  pinMode(pinMotor2A, OUTPUT); digitalWrite(pinMotor2A, LOW);
  pinMode(pinMotor2B, OUTPUT); digitalWrite(pinMotor2B, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Firmware Pancho v" VERSION_FIRMWARE);   // [SO]
  Serial.printf("FreeRTOS: tick = %d Hz\n", configTICK_RATE_HZ);   // [SO] para verificar la configuración

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

  // La cola se crea ANTES de iniciar Bluepad32 porque sus callbacks la usan
  colaComandos = xQueueCreate(32, sizeof(Comando));
  if (colaComandos == NULL) {
    Serial.println("ERROR: no se pudo crear la cola de comandos");
    while (true) delay(1000);
  }

  // [SO] Mutex y semáforo de la telemetría (se crean antes que las tareas que los usan)
  mutexDatos    = xSemaphoreCreateMutex();    // mutex: tiene herencia de prioridad
  semTelemetria = xSemaphoreCreateBinary();   // binario: arranca "vacío" (en 0)
  if (mutexDatos == NULL || semTelemetria == NULL) {
    Serial.println("ERROR: no se pudo crear el mutex o el semáforo");
    while (true) delay(1000);
  }

  // [SO] UART2 hacia el módulo HC-06
  Serial2.begin(BAUD_HC06, SERIAL_8N1, PIN_HC06_RX, PIN_HC06_TX);

  // LEDs
  pinMode(led_interno, OUTPUT);
  pinMode(pinLedRojo, OUTPUT);
  pinMode(pinLedVerde, OUTPUT);
  pwmInit();
  detenerYa();  // arranca detenido

  // Calibración guardada en la memoria flash
  prefs.begin("robot", false);
  factorM1 = prefs.getFloat("fm1", 1.0f);
  factorM2 = prefs.getFloat("fm2", 1.0f);
  Serial.printf("Calibración cargada: M1=%.2f  M2=%.2f\n", factorM1, factorM2);
  if (factorM1 < 1.0f || factorM2 < 1.0f) {
    Serial.println("AVISO: hay un motor con potencia reducida por calibración (L3+R3 para borrar)");
  }

  // Joystick vinculado guardado en la memoria flash
  prefsJoy.begin("joystick", false);
  hayVinculo = (prefsJoy.getBytes("mac", macVinculada, 6) == 6);
  if (hayVinculo) {
    Serial.printf("Joystick vinculado: %02X:%02X:%02X:%02X:%02X:%02X (solo este puede manejar el robot)\n",
                  macVinculada[0], macVinculada[1], macVinculada[2],
                  macVinculada[3], macVinculada[4], macVinculada[5]);
  } else {
    Serial.println("Sin joystick vinculado: el primero que se conecte queda vinculado");
  }

  // MAC Bluetooth del ESP32 (SOLO para joysticks PS3 con Sixaxis Pair Tool; con PS4/PS5/Xbox no hace falta)
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_BT);
  Serial.printf("MAC Bluetooth del ESP32: %02x:%02x:%02x:%02x:%02x:%02x\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  // Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);
  // BP32.forgetBluetoothKeys();   // descomentar para borrar emparejamientos guardados
  Serial.println("Bluepad32 listo: encendé el joystick para emparejar");

  // Tareas FreeRTOS
  xTaskCreatePinnedToCore(BlinkTask,        "BlinkTask",        2048, NULL, 2, &handleBlink, 1);  // [SO] se guarda el handle
  xTaskCreatePinnedToCore(TelemetriaTask,   "TelemetriaTask",   4096, NULL, 3, NULL, 1);          // [SO]
  xTaskCreatePinnedToCore(ComunicacionTask, "ComunicacionTask", 8192, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(ControlTask,      "ControlTask",      6144, NULL, 5, NULL, 1);

  // [SO] El timer arranca al final, cuando ya existe la tarea que espera el semáforo
  iniciarTimerTelemetria();

  delay(100);  // Breve espera para la configuración
}

void loop() {
  // [SO] La tarea de loop() no hace nada: se elimina a sí misma y libera su memoria.
  // Todo el trabajo lo hacen las tareas creadas en setup().
  vTaskDelete(NULL);
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
// Funciones auxiliares de lectura del joystick
// ------------------------------------------------------------

// Pone un valor del stick (-511..512) en el rango -1.0 .. 1.0
float normalizar(int v) {
  return constrain(v / 512.0f, -1.0f, 1.0f);
}

// Ignora la zona central y re-escala el resto para que no haya salto al salir de ella
float aplicarZonaMuerta(float x) {
  float a = fabsf(x);
  if (a < ZONA_MUERTA) return 0.0f;
  a = (a - ZONA_MUERTA) / (1.0f - ZONA_MUERTA);
  return (x < 0) ? -a : a;
}

// Curva de respuesta: mezcla de lineal y cuadrática. A fondo siempre llega a 1.0.
// Con CURVA_LINEAL = 0.7, a medio stick da 0.43 (antes 0.35): más potencia en todo el recorrido.
float curva(float x) {
  return CURVA_LINEAL * x + (1.0f - CURVA_LINEAL) * x * fabsf(x);
}

// Convierte la salida del mezclador (-1..1) en % de potencia con signo.
// Aplica la potencia mínima (para vencer la fricción) y el máximo del modo.
float aPotencia(float m, float maxPct) {
  float a = fabsf(m);
  if (a < 0.01f) return 0.0f;
  float p = POTENCIA_MIN + (maxPct - POTENCIA_MIN) * a;
  return (m < 0) ? -p : p;
}

// Devuelve true solo en el instante en que se aprieta el botón
bool flanco(bool ahora, bool &antes) {
  bool r = ahora && !antes;
  antes = ahora;
  return r;
}

// Avisa el modo con la luz del joystick (PS4/PS5) y una vibración corta
void aplicarModo(ControllerPtr g, uint8_t modo) {
  Serial.printf("Modo: %s (máx %.0f %%, 0->máx en %.2f s)\n",
                MODOS[modo].nombre, MODOS[modo].maxPct, MODOS[modo].maxPct / MODOS[modo].acel);
  g->setColorLED(MODOS[modo].r, MODOS[modo].g, MODOS[modo].b);
  g->playDualRumble(0, 120, 120, 60);
}

void enviarComando(uint8_t tipo, uint8_t modo, float a, float b) {
  Comando c = { tipo, modo, a, b };
  xQueueSend(colaComandos, &c, 0);
}

// ------------------------------------------------------------
// Tarea de comunicación: lee el joystick, interpreta y manda a la cola
// ------------------------------------------------------------
void ComunicacionTask(void *parameter) {
  uint8_t modo = MODO_INICIAL;
  ControllerPtr visto = nullptr;
  bool pL1 = false, pR1 = false;
  bool pCuad = false, pTri = false, pCruz = false, pCirc = false;
  bool pIzq = false, pDer = false, pL3R3 = false;
  uint32_t inicioDesvincular = 0;      // cuándo se empezó a mantener Share + Options
  String lineaSerie = "";
#ifdef DEBUG_JOYSTICK
  uint32_t ultimoDebug = 0;
#endif

  for (;;) {
    bool datos = BP32.update();          // procesa Bluetooth y dispara los callbacks

    // --- Desconectar joysticks ajenos (rechazados en onConnectedController) ---
    for (uint8_t i = 0; i < MAX_RECHAZOS; i++) {
      if (rechazados[i] != nullptr) {
        rechazados[i]->disconnect();
        rechazados[i] = nullptr;
      }
    }

    // --- Comando "olvidar" desde el Monitor Serie ---
    while (Serial.available()) {
      char ch = Serial.read();
      if (ch == '\n' || ch == '\r') {
        lineaSerie.trim();
        if (lineaSerie.equalsIgnoreCase("olvidar")) desvincular();
        lineaSerie = "";
      } else if (lineaSerie.length() < 20) {
        lineaSerie += ch;
      }
    }

    if (gamepad && gamepad->isConnected() && gamepad->isGamepad()) {
      bool modoCambio = false;

      // --- Desvincular: mantener Share + Options (Xbox: View + Menu) ---
      if (gamepad->miscSelect() && gamepad->miscStart()) {
        if (inicioDesvincular == 0) inicioDesvincular = millis();
        else if (millis() - inicioDesvincular > TIEMPO_DESVINCULAR_MS) {
          inicioDesvincular = 0;
          desvincular();
          vTaskDelay(10 / portTICK_PERIOD_MS);
          continue;                      // el joystick se está desconectando
        }
      } else {
        inicioDesvincular = 0;
      }

      // Joystick recién conectado: arrancar en el modo inicial
      if (gamepad != visto) {
        visto = gamepad;
        modo = MODO_INICIAL;
        aplicarModo(gamepad, modo);
        modoCambio = true;
      }

      // --- Modos de velocidad: L1 baja, R1 sube ---
      bool l1 = gamepad->l1();
      bool r1 = gamepad->r1();
      if (flanco(l1, pL1) && modo > 0) {
        modo--;
        aplicarModo(gamepad, modo);
        modoCambio = true;
      }
      if (flanco(r1, pR1) && modo < N_MODOS - 1) {
        modo++;
        aplicarModo(gamepad, modo);
        modoCambio = true;
      }

      // --- Calibración de motores ---
      if (flanco(gamepad->x(), pCuad)) enviarComando(CMD_CALIBRAR, 0, -0.02f,  0.0f);   // M1 bajar
      if (flanco(gamepad->y(), pTri))  enviarComando(CMD_CALIBRAR, 0,  0.02f,  0.0f);   // M1 subir
      if (flanco(gamepad->a(), pCruz)) enviarComando(CMD_CALIBRAR, 0,  0.0f,  -0.02f);  // M2 bajar
      if (flanco(gamepad->b(), pCirc)) enviarComando(CMD_CALIBRAR, 0,  0.0f,   0.02f);  // M2 subir
      if (flanco(gamepad->thumbL() && gamepad->thumbR(), pL3R3)) enviarComando(CMD_BORRAR_CAL, 0, 0, 0);

      // --- Luces ---
      uint8_t d = gamepad->dpad();
      if (flanco((d & DPAD_IZQUIERDA) != 0, pIzq)) enviarComando(CMD_LUCES, 0, 1, 0);   // verde
      if (flanco((d & DPAD_DERECHA)   != 0, pDer)) enviarComando(CMD_LUCES, 0, 0, 0);   // roja

      // --- Manejo analógico ---
      // Se envía cada vez que llega un reporte nuevo del joystick (también sirve de "latido"
      // para el watchdog de ControlTask) o cuando cambia el modo.
      if (datos || modoCambio) {
        // Stick izquierdo vertical: acelerar (+) / retroceder (-).
        // Si queda al revés, cambiar SENTIDO_AVANCE a -1 (arriba del todo).
        float t = curva(aplicarZonaMuerta(normalizar(gamepad->axisY()) * SENTIDO_AVANCE));
        // Stick derecho horizontal: giro. Positivo = derecha.
        float s = curva(aplicarZonaMuerta(normalizar(gamepad->axisRX()))) * GANANCIA_GIRO * INVERTIR_GIRO;

        // Mezcla tipo "arcade": M1 = rueda izquierda, M2 = rueda derecha
        float m1 = t + s;
        float m2 = t - s;

        // Si alguno se pasa de 1.0, se escalan los dos juntos (mantiene la proporción del giro)
        float mx = fmaxf(fabsf(m1), fabsf(m2));
        if (mx > 1.0f) { m1 /= mx; m2 /= mx; }

        float p1 = aPotencia(m1, MODOS[modo].maxPct);
        float p2 = aPotencia(m2, MODOS[modo].maxPct);
        enviarComando(CMD_MOTORES, modo, p1, p2);

#ifdef DEBUG_JOYSTICK
        if (millis() - ultimoDebug > 200) {
          ultimoDebug = millis();
          Serial.printf("axisY=%5d axisRX=%5d | M1=%6.1f%% M2=%6.1f%%\n",
                        gamepad->axisY(), gamepad->axisRX(), p1, p2);
        }
#endif
      }
    } else {
      // Sin joystick: dejamos el estado listo para la próxima conexión
      visto = nullptr;
      pL1 = pR1 = pCuad = pTri = pCruz = pCirc = pIzq = pDer = pL3R3 = false;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ------------------------------------------------------------
// Tarea de control: único lugar donde se manejan los motores.
// Corre con período fijo RAMPA_PERIODO_MS (vTaskDelayUntil), así la
// pendiente de las rampas es exacta aunque lleguen muchos comandos.
// Cada ciclo: 1) atiende los comandos pendientes (fijan el objetivo)
//             2) watchdog: si no hay señal del joystick, frena
//             3) avanza un paso cada rampa
//             4) escribe la salida a los motores
// ------------------------------------------------------------
void ControlTask(void *parameter) {
  Comando c;
  TickType_t ultimoCiclo = xTaskGetTickCount();
  TickType_t ultimoMando = xTaskGetTickCount();   // última vez que llegó CMD_MOTORES
  bool conectado = false;

#ifdef DEBUG_RAMPA
  uint32_t contador = 0;
  const uint32_t cada = (100 / RAMPA_PERIODO_MS) ? (100 / RAMPA_PERIODO_MS) : 1;   // ~10 muestras por segundo
#endif

  for (;;) {
    // 1) Comandos pendientes
    while (xQueueReceive(colaComandos, &c, 0) == pdTRUE) {
      if (c.tipo == CMD_CONECTADO)    { conectado = true;  ultimoMando = xTaskGetTickCount(); }
      if (c.tipo == CMD_DESCONECTADO) { conectado = false; }
      if (c.tipo == CMD_MOTORES)      { ultimoMando = xTaskGetTickCount(); }
      procesarComando(c.tipo, c.modo, c.a, c.b);
    }

#ifdef USAR_WATCHDOG_JOYSTICK
    // 2) Watchdog: el joystick está "conectado" pero dejó de enviar datos
    if (conectado && (objetivoM1 != 0 || objetivoM2 != 0) &&
        (xTaskGetTickCount() - ultimoMando) > pdMS_TO_TICKS(TIMEOUT_JOYSTICK_MS)) {
      objetivoM1 = objetivoM2 = 0;       // frena con la rampa de frenado
      Serial.println("Joystick sin señal: frenando");
    }
#endif

    // 3) Rampa de cada motor (arranque progresivo)
    actualM1 = avanzarRampa(actualM1, objetivoM1);
    actualM2 = avanzarRampa(actualM2, objetivoM2);

    // 4) Salida
    escribirMotores(actualM1, actualM2);

    // 5) [SO] Publicar los datos para la telemetría.
    //    Sección crítica corta: solo copiar 3 valores. Si el mutex está tomado,
    //    espera como máximo 2 ms; si no lo consigue, publica en el ciclo siguiente.
    if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(2)) == pdTRUE) {
      datosTel.modo = modoActual;
      datosTel.pwm1 = pwmAplicado1;
      datosTel.pwm2 = pwmAplicado2;
      xSemaphoreGive(mutexDatos);
    }

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
// [SO] Tarea de telemetría.
// Duerme bloqueada en el semáforo hasta que la ISR del timer lo libera (cada 100 ms).
// Copia los datos compartidos dentro del mutex y los envía FUERA del mutex,
// porque mandar la línea por la UART tarda unos 30 ms a 9600 baudios.
// Formato (CSV): T,<ms desde el arranque>,<modo>,<pwm1>,<pwm2>,<%M1>,<%M2>
// Ejemplo:       T,12350,1,180,-175,70.6,-68.6
// ------------------------------------------------------------
void TelemetriaTask(void *parameter) {
  DatosTelemetria copia;

  for (;;) {
    if (xSemaphoreTake(semTelemetria, portMAX_DELAY) == pdTRUE) {

      if (xSemaphoreTake(mutexDatos, pdMS_TO_TICKS(5)) == pdTRUE) {
        copia = datosTel;                 // copia rápida
        xSemaphoreGive(mutexDatos);       // se suelta enseguida
      } else {
        continue;                         // no se pudo leer a tiempo: se saltea este envío
      }

      Serial2.printf("T,%lu,%u,%d,%d,%.1f,%.1f\n",
                     (unsigned long)millis(), copia.modo,
                     copia.pwm1, copia.pwm2,
                     copia.pwm1 / 2.55f, copia.pwm2 / 2.55f);
    }
  }
}

// ------------------------------------------------------------
// Avanza un paso de rampa desde 'actual' hacia 'objetivo' (valores en % con signo).
// - Si la potencia se acerca a cero (frenar) usa rampaDecel.
// - Si se aleja de cero o arranca desde cero (acelerar) usa rampaAcel.
// - Pendiente 0 = sin rampa.
// Las pendientes dependen del modo de velocidad activo.
// ------------------------------------------------------------
float avanzarRampa(float actual, float objetivo) {
  float pendiente;

  if (actual == objetivo) return actual;

  bool haciaCero = (actual > 0 && objetivo < actual) || (actual < 0 && objetivo > actual);
  if (haciaCero) pendiente = rampaDecel;
  else pendiente = rampaAcel;

  if (pendiente <= 0) return objetivo;                      // Salto directo, sin rampa

  float paso  = RAMPA_PERIODO_MS * (pendiente / 1000.0f);   // % que se puede mover en este ciclo
  float delta = objetivo - actual;
  if (fabsf(delta) <= paso) return objetivo;                // falta menos de un paso

  if (delta > 0) return actual + paso;
  else return actual - paso;
}

// ------------------------------------------------------------
// Interpretación de comandos (se ejecuta dentro de ControlTask)
// Los comandos de movimiento solo fijan el OBJETIVO; la rampa lo alcanza gradualmente.
// ------------------------------------------------------------
void procesarComando(uint8_t tipo, uint8_t modo, float a, float b) {
  switch (tipo) {
    case CMD_CONECTADO:
      detenerYa();
      if (handleBlink != NULL) vTaskSuspend(handleBlink);   // [SO] deja de parpadear
      digitalWrite(led_interno, HIGH);                       // [SO] LED azul fijo = conectado
      Serial.println("Joystick conectado");
      break;

    case CMD_DESCONECTADO:
      detenerYa();               // por seguridad, parada inmediata (sin rampa)
      if (handleBlink != NULL) vTaskResume(handleBlink);    // [SO] vuelve a parpadear
      Serial.println("Joystick desconectado: motores detenidos");
      break;

    case CMD_MOTORES:
      if (modo < N_MODOS) {
        rampaAcel  = MODOS[modo].acel;
        rampaDecel = MODOS[modo].decel;
        modoActual = modo;       // [SO] para la telemetría
      }
      setMotors(a, b);
      break;

    case CMD_CALIBRAR:
      factorM1 = constrain(factorM1 + a, 0.50f, 1.0f);
      factorM2 = constrain(factorM2 + b, 0.50f, 1.0f);
      prefs.putFloat("fm1", factorM1);
      prefs.putFloat("fm2", factorM2);
      Serial.printf("Calibración: M1=%.2f  M2=%.2f\n", factorM1, factorM2);
      break;

    case CMD_BORRAR_CAL:
      factorM1 = 1.0f;
      factorM2 = 1.0f;
      prefs.putFloat("fm1", factorM1);
      prefs.putFloat("fm2", factorM2);
      Serial.println("Calibración borrada: M1=1.00  M2=1.00");
      break;

    case CMD_LUCES:
      if (a > 0.5f) { pwmWrite(pinLedRojo, 0);   pwmWrite(pinLedVerde, 255); }
      else          { pwmWrite(pinLedRojo, 255); pwmWrite(pinLedVerde, 0);   }
      break;
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
  pwmAplicado1 = pwm1;           // [SO] se guarda para la telemetría
  pwmAplicado2 = pwm2;

  // Controlar el motor 1
  if (pwm1 >= 0) {
    pwmWrite(pinMotor1A, pwm1);    // Sentido A
    pwmWrite(pinMotor1B, 0);       // Sentido B apagado
  } else {
    pwmWrite(pinMotor1A, 0);       // Sentido A apagado
    pwmWrite(pinMotor1B, -pwm1);   // Sentido B activado
  }

  // Controlar el motor 2
  if (pwm2 >= 0) {
    pwmWrite(pinMotor2A, pwm2);    // Sentido A
    pwmWrite(pinMotor2B, 0);       // Sentido B apagado
  } else {
    pwmWrite(pinMotor2A, 0);       // Sentido A apagado
    pwmWrite(pinMotor2B, -pwm2);   // Sentido B activado
  }
}
