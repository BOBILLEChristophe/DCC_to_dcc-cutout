/*

  Description :
    Ce programme reçoit un signal DCC, détecte les trames valides,
    et les retransmet en ajoutant :
      - un préambule fixe de 16 bits '1'
      - un cutout d'environ 464 µs (nécessaire pour RailCom)

    Il fonctionne sur un ESP32, utilise les interruptions pour détecter
    les alternances du signal DCC, et FreeRTOS pour répartir les tâches.

    La détection des courts-circuits a été ajoutée dans cette version mais nécessite
    une modification du hardware.

 */

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

#define PROJECT "DCC to DCC Railcom"
#define VERSION "v 0.9.7 - 10/06/2025"
#define AUTHOR "Christophe BOBILLE : christophe.bobille@gmail.com"

#include <Arduino.h>

// Broches ESP32
const gpio_num_t pinDCCin = GPIO_NUM_4;    // Entrée DCC (opto isolée via 6N137)
const gpio_num_t pinIn0 = GPIO_NUM_27;     // Sortie H-Bridge A
const gpio_num_t pinIn1 = GPIO_NUM_33;     // Sortie H-Bridge B
const gpio_num_t pinEnable = GPIO_NUM_32;  // Activation du pont en H
const gpio_num_t pinCC = GPIO_NUM_5;       // Broche sur tensions ATtiny
// Constantes
constexpr uint8_t NB_PREAMBLE_HALF_BIT = 32;
constexpr byte CUTOUT_EVT = 0;
constexpr byte PREAMBLE_EVT = 1;
constexpr byte DATA_EVT = 2;

// Structure pour recevoir les paquets DCC
struct DCC_PACKET {
  uint8_t count;
  uint64_t data;
};

// Objets globaux
QueueHandle_t dccInQueue;  // Queue pour stocker les bits détectés
QueueHandle_t dccOutQueue;
QueueHandle_t durationQueue;
DCC_PACKET dccOutPacketTimer = { 0, 0 };
DCC_PACKET idlePacket = { 28, 0xFF801FE };  // Packet idle =   0 11111111 0 00000000 0 11111111 1

// États de sortie
bool sens0 = HIGH;
bool sens1 = HIGH;
uint8_t cutoutCount = 0;
uint8_t preambuleCount = 0;
byte state = CUTOUT_EVT;
bool toggleState = true;
bool firstHalfBit = true;
uint8_t bitEntier = 0;
uint8_t bitCountIsr = 0;
uint8_t bitValIsr = 0;
uint64_t dccOutPacketTimerData;
uint8_t dccOutPacketTimerCount;

/*-----------------------------------------------------------------------
Routine d'interruption déclenchée sur changement d'état de GPIO pinDCCin
------------------------------------------------------------------------*/

void IRAM_ATTR dccInterruptHandler() {
  static uint32_t lastTime = 0;  // Dernière mesure de micros() pour calculer durée
  uint32_t now = micros();
  uint32_t duration = now - lastTime;

  constexpr uint32_t minDuration = 80;  // Ignore tout front trop rapide (< 80µs)
  lastTime = now;

  if (duration < minDuration)
    return;

  // constexpr uint32_t bit1Min = 100, bit1Max = 130;
  // constexpr uint32_t bit0Min = 170, bit0Max = 220;

  constexpr uint32_t bit1Min = 100, bit1Max = 130;
  constexpr uint32_t bit0Min = 170, bit0Max = 250;


  uint8_t bitVal = 2;  // Une valeur entière supérieure à 1.

  if (duration >= bit1Min && duration <= bit1Max)
    bitVal = 1;
  else if (duration >= bit0Min && duration <= bit0Max)
    bitVal = 0;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(dccInQueue, &bitVal, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}

/*-----------------------------------------------------------------------
Tâche FreeRTOS pour analyser les impulsions DCC
------------------------------------------------------------------------*/
void dccParserTask(void *pvParameters) {
  const byte PREAMBLE = 1;
  const byte PACKET_START_BIT = 2;
  const byte COLLECTING_DATA = 3;
  byte state = PREAMBLE;

  uint8_t bitVal = 2;
  int8_t dataBitCount = 0;
  uint8_t preambleCount = 0;
  DCC_PACKET dccPacket = { 0, 0 };

  uint8_t byte_1 = 0;
  uint8_t byte_2 = 0;
  uint8_t byte_3 = 0;
  uint8_t byte_4 = 0;
  uint8_t byte_5 = 0;
  uint8_t crc = 0;

  uint64_t dccPacketData = 0;
  uint8_t dccPacketCount = 0;

  auto resetFrame = [&]() {
    dataBitCount = 0;
    preambleCount = 0;
    state = PREAMBLE;
    dccPacket.data = 0;
    dccPacket.count = 0;
    dccPacketData = 0;
    dccPacketCount = 0;
  };

  while (true) {
    if (xQueueReceive(dccInQueue, &bitVal, portMAX_DELAY) == pdTRUE) {
      if (bitVal < 2) {
        switch (state) {
          case PREAMBLE:
            //   /*      Le programme va tourné en boucle ici jusqu'à détecter
            //           au moins 10 durées comprises entre 50 et 65 µs soit 10 bits à 1)
            //           https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-92-2004-07.pdf
            //           The preamble to a packet consists of a sequence of "1" bits. A digital decoder must not
            //           accept as a valid, any preamble that has less then 10 complete one bits, or require for
            //           proper reception of a packet with more than 12 complete one bits. A command station
            //           must send a minimum of 14 full preamble bits.
            //   */
            if (bitVal == 1) {
              preambleCount++;
              if (preambleCount >= 10)
                state = PACKET_START_BIT;
            } else
              preambleCount = 0;
            break;

          case PACKET_START_BIT:
            // Serial.println("PACKET_START_BIT");
            if (bitVal == 0) {
              state = COLLECTING_DATA;
              dataBitCount++;
            }
            break;

          case COLLECTING_DATA:
            dccPacket.count++;
            dataBitCount++;
            if (bitVal == 1)
              dccPacket.data |= (1ULL << dccPacket.count);

            if (dataBitCount == 10)  // Bit de séparation
            {
              if (bitVal == 1) {  // Fin du paquet
                // Serial.println("fin de paquet");
                // Serial.println(dccPacket.data, BIN);
                // Serial.printf("long %d\n", dccPacket.count);
                // On a un paquet valide si count = 28, 37 ou 46 bits
                dccPacket.count++;
                if (dccPacket.data != idlePacket.data) {
                  // Calcul du checksum utilisé pour vérifier l'intégrité des données.
                  byte_1 = (dccPacket.data & 0x000000001FE) >> 1;
                  byte_2 = (dccPacket.data & 0x0000003FC00) >> 10;
                  byte_3 = (dccPacket.data & 0x00007F80000) >> 19;
                  byte_4 = (dccPacket.data & 0x00FF0000000) >> 28;
                  byte_5 = (dccPacket.data & 0x1FE000000000) >> 37;
                  bool send = false;
                  switch (dccPacket.count) {
                    case 28:
                      crc = byte_1 ^ byte_2;
                      if (byte_3 == crc) {
                        Serial.println("send 28 bits");
                        send = true;
                      }
                      break;
                    case 37:
                      crc = (byte_1 ^ byte_2) ^ byte_3;
                      if (byte_4 == crc) {
                        Serial.println("send 37 bits");
                        send = true;
                      }
                      break;
                    case 46:
                      crc = ((byte_1 ^ byte_2) ^ byte_3) ^ byte_4;
                      if (byte_5 == crc) {
                        send = true;
                        Serial.println("send 46 bits");
                      }
                      break;
                  }
                  if (send)
                    xQueueSend(dccOutQueue, &dccPacket, portMAX_DELAY);
                  else
                    Serial.print("Error dccPacket\n");
                }
                resetFrame();
              } else  // bitVal == 0
                dataBitCount = 1;
            }
            break;
        }
      } else {
        resetFrame();
        Serial.println("error bit = 2\n");
      }
    }
  }
}

/*-----------------------------------------------------------------------
  Création du signal DCC
------------------------------------------------------------------------*/

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR timerHandler() {
  portENTER_CRITICAL_ISR(&timerMux);
  switch (state) {
    case CUTOUT_EVT:  // Insertion du cutout RailCom
      if (cutoutCount == 0) {
        // On désactive les deux sorties du pont en H
        gpio_set_level(pinIn0, LOW);
        gpio_set_level(pinIn1, LOW);
      }
      cutoutCount++;
      if (cutoutCount == 8)  // 58µ x 8  = 464 ( 454 > 464 < 488 )
      {
        cutoutCount = 0;
        state = PREAMBLE_EVT;
      }
      break;

    case PREAMBLE_EVT:  // Création du préambule
      sens1 = sens0;
      sens0 = !sens0;
      gpio_set_level(pinIn0, sens1);
      gpio_set_level(pinIn1, sens0);

      preambuleCount++;
      if (preambuleCount == NB_PREAMBLE_HALF_BIT) {
        preambuleCount = 0;
        state = DATA_EVT;
        bitCountIsr = 0;
        dccOutPacketTimerData = dccOutPacketTimer.data;
        dccOutPacketTimerCount = dccOutPacketTimer.count;
      }
      break;

    case DATA_EVT:  // Envoi des données
      bitValIsr = (dccOutPacketTimerData >> bitCountIsr) & 0x01;
      if (bitValIsr == 0) {
        if (firstHalfBit == true) {
          firstHalfBit = false;
          toggleState = false;
        } else {
          firstHalfBit = true;
          toggleState = true;
        }
      }

      if (toggleState) {  // Inversion d'etat
        bitEntier++;
        sens1 = sens0;
        sens0 = !sens0;
        gpio_set_level(pinIn0, sens1);
        gpio_set_level(pinIn1, sens0);
      }

      if (bitEntier == 2) {
        bitEntier = 0;
        bitCountIsr++;
      }

      if (bitCountIsr == dccOutPacketTimerCount) {
        state = CUTOUT_EVT;
        cutoutCount = 0;
      }
      break;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

/*-----------------------------------------------------------------------
  dccOutTask
------------------------------------------------------------------------*/

void dccOutTask(void *pvParameters) {
  DCC_PACKET tempPacket = { 0, 0 };
  while (true) {
    if (xQueueReceive(dccOutQueue, &tempPacket, pdMS_TO_TICKS(30)) == pdTRUE) {
      dccOutPacketTimer = tempPacket;
      //Serial.println(dccOutPacketTimer.data, BIN);
    }

    else {
      dccOutPacketTimer = idlePacket;
      //Serial.println("idle");
    }
  }
}

/*-----------------------------------------------------------------------
  dccStop // Coupure en cas de sur intensités sur la voie
------------------------------------------------------------------------*/

void dccStop(void *pvParameters) {
  while (true) {
    if (digitalRead(pinCC))
      gpio_set_level(pinEnable, LOW);  // Désactivation du pont en H
    else
      gpio_set_level(pinEnable, HIGH);  // Activation du pont en H
    vTaskDelay(1);
  }
}

/*-----------------------------------------------------------------------
  setup
------------------------------------------------------------------------*/

void setup() {
  Serial.begin(115200);

  Serial.printf("\nProject   :    %s", PROJECT);
  Serial.printf("\nVersion   :    %s", VERSION);
  Serial.printf("\nAuteur    :    %s", AUTHOR);
  Serial.printf("\nFichier   :    %s", __FILE__);
  Serial.printf("\nCompiled  :    %s", __DATE__);
  Serial.printf(" - %s\n\n", __TIME__);

  pinMode(pinDCCin, INPUT);
  pinMode(pinIn0, OUTPUT);
  pinMode(pinIn1, OUTPUT);
  pinMode(pinEnable, OUTPUT);
  gpio_set_level(pinEnable, HIGH);  // Activation du pont en H

  pinMode(pinCC, INPUT_PULLUP);

  // Création de la queue pour réception des bits
  dccInQueue = xQueueCreate(1024, sizeof(uint8_t));
  dccOutQueue = xQueueCreate(32, sizeof(DCC_PACKET));

  // Attachement de l'interruption DCC
  attachInterrupt(digitalPinToInterrupt(pinDCCin), dccInterruptHandler, FALLING);  // ou RISING selon câblage

  //************************ Timer ********************************* */
  /* Version 1 https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html */

  // Initialisation du timer à 1 MHz (1 tick = 1µs)
  timer = timerBegin(1000000);
  // Attachement de l'interruption
  timerAttachInterrupt(timer, timerHandler);
  // Définition de l'alarme à 58 µs (DCC timing)
  timerAlarm(timer, 58, true, 0);


  /* Version 2 */

  // timer = timerBegin(0, 80, true); // timer 0, prescaler 80, count up.
  // timerAttachInterrupt(timer, timerHandler, true);
  // timerAlarmWrite(timer, 58, true); // 58µS
  // timerAlarmEnable(timer);
  //******************************************************************* */

  // Création de la tâche de traitement DCC
  xTaskCreatePinnedToCore(dccParserTask, "DCC Parser", 4 * 1024, NULL, 2, NULL, 0);
  // Création de la tâche  pour l'envoi du nouveau signal DCC
  xTaskCreatePinnedToCore(dccOutTask, "DCC Out", 4 * 1024, NULL, 4, NULL, 0);
  // Création de la tâche  pour coupure en cas de sur tension
  xTaskCreatePinnedToCore(dccStop, "DCC stop", 1 * 1024, NULL, 6, NULL, 0);

  // Pour activer ou désactiver le debug, commenter ou décommenter la ligne ci-dessous
  // Serial.end();
}

/*-----------------------------------------------------------------------
  loop
------------------------------------------------------------------------*/

void loop() {
}  // nothing to do
