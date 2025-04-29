/*
    Modifie le signal DCC d’une centrale qui ne génère pas de cutout pour le rendre compatible avec la détection Railcom.

    -	Ajoute un préambule de 16 bits "1" pour synchroniser le décodeur.
    -	Ajoute un cutout de 464 µs

    Attention, la protéction contre les courts-circuits n'est pas implantée !

 */

#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif

#define PROJECT "DCC to DCC Railcom"
#define VERSION "v 0.9 - 29/04/2025"
#define AUTHOR "Christophe BOBILLE : christophe.bobille@gmail.com"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <driver/timer.h>
#include <driver/gpio.h>

// Broches ESP32
const gpio_num_t pinDCCin = GPIO_NUM_13;
const gpio_num_t pinIn0 = GPIO_NUM_27;
const gpio_num_t pinIn1 = GPIO_NUM_33;
const gpio_num_t pinEnable = GPIO_NUM_32;

// Constantes
constexpr uint8_t NB_PREAMBLE_HALF_BIT = 32;
constexpr byte CUTOUT_EVT = 0;
constexpr byte PREAMBLE_EVT = 1;
constexpr byte DATA_EVT = 2;

// Structure pour recevoir les paquets DCC
struct DCC_PACKET
{
  uint8_t count;
  uint64_t data;
};

// Objets globaux
QueueHandle_t dccInQueue;
QueueHandle_t dccOutQueue;
QueueHandle_t durationQueue;
DCC_PACKET dccOutPacketTimer = {0, 0};
DCC_PACKET idlePacket = {28, 0xFF801FE};

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



void IRAM_ATTR dccInterruptHandler()
{
  static uint32_t lastTime = 0;
  uint32_t now = micros();
  uint32_t duration = now - lastTime;

  constexpr uint32_t minDuration = 80; // Ignore tout front trop rapide (< 80µs)
  lastTime = now;

  if (duration < minDuration)
    return; 

  constexpr uint32_t bit1Min = 100, bit1Max = 130;
  constexpr uint32_t bit0Min = 170, bit0Max = 220;
  uint8_t bitVal = 2; // Une valeur entière supérieure à 1.

  if (duration >= bit1Min && duration <= bit1Max)
    bitVal = 1;
  else if (duration >= bit0Min && duration <= bit0Max)
    bitVal = 0;

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(dccInQueue, &bitVal, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken)
    portYIELD_FROM_ISR();
}

void dccParserTask(void *pvParameters)
{
  const byte PREAMBLE = 1;
  const byte PACKET_START_BIT = 2;
  const byte COLLECTING_DATA = 3;
  byte state = PREAMBLE;

  uint8_t bitVal = 2;
  int8_t dataBitCount = 0;
  uint8_t preambleCount = 0;
  DCC_PACKET dccPacket = {0, 0};

  uint64_t dccPacketData = 0;
  uint8_t dccPacketCount = 0;

  auto resetFrame = [&]()
  {
    dataBitCount = 0;
    preambleCount = 0;
    state = PREAMBLE;
    dccPacket.data = 0;
    dccPacket.count = 0;
    dccPacketData = 0;
    dccPacketCount = 0;
  };

  while (true)
  {
    if (xQueueReceive(dccInQueue, &bitVal, portMAX_DELAY) == pdTRUE)
    {
      if (bitVal < 2)
      {
        switch (state)
        {
        case PREAMBLE:
          //   /*      Le programme va tourné en boucle ici jusqu'à détecter
          //           au moins 10 durées comprises entre 50 et 65 µs soit 10 bits à 1)
          //           https://www.nmra.org/sites/default/files/standards/sandrp/DCC/S/s-92-2004-07.pdf
          //           The preamble to a packet consists of a sequence of "1" bits. A digital decoder must not
          //           accept as a valid, any preamble that has less then 10 complete one bits, or require for
          //           proper reception of a packet with more than 12 complete one bits. A command station
          //           must send a minimum of 14 full preamble bits.
          //   */
          if (bitVal == 1)
          {
            preambleCount++;
            if (preambleCount >= 10)
              state = PACKET_START_BIT;
          }
          else
            preambleCount = 0;
          break;

        case PACKET_START_BIT:
          // Serial.println("PACKET_START_BIT");
          if (bitVal == 0)
          {
            state = COLLECTING_DATA;
            // dccPacket.count++;
            dataBitCount++;
          }
          break;

        case COLLECTING_DATA:
          dccPacket.count++;
          dataBitCount++;
          if (bitVal == 1)
            dccPacket.data |= (1ULL << dccPacket.count);

          if (dataBitCount == 10) // Bit de séparation
          {
            if (bitVal == 1)
            { // Fin du paquet
              // Serial.println("fin de paquet");
              // Serial.println(dccPacket.data, BIN);
              // Serial.printf("long %d\n", dccPacket.count);
              //  On a un paquet valide si count = 28, 37 bits
              dccPacket.count++;
              if (dccPacket.data != idlePacket.data)
              {
                switch (dccPacket.count)
                {
                case 28:
                case 37:
                  xQueueSend(dccOutQueue, &dccPacket, portMAX_DELAY);
                  break;
                }
              }
              resetFrame();
            }
            else // bitVal == 0
            {
              // Serial.println("fin de byte");
              dataBitCount = 1;
            }
          }
          break;
        }
      }
      else
      {
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

void IRAM_ATTR timerHandler()
{
  portENTER_CRITICAL_ISR(&timerMux);
  switch (state)
  {
  case CUTOUT_EVT:
    if (cutoutCount == 0)
    {
      gpio_set_level(pinIn0, LOW);
      gpio_set_level(pinIn1, LOW);
    }
    cutoutCount++;
    if (cutoutCount == 8) // 58µ x 8  = 464 ( 454 > 464 < 488 )
    {
      cutoutCount = 0;
      state = PREAMBLE_EVT;
    }
    break;

  case PREAMBLE_EVT:
    sens1 = sens0;
    sens0 = !sens0;
    gpio_set_level(pinIn0, sens1);
    gpio_set_level(pinIn1, sens0);

    preambuleCount++;
    if (preambuleCount == NB_PREAMBLE_HALF_BIT)
    {
      preambuleCount = 0;
      state = DATA_EVT;
      bitCountIsr = 0;
      dccOutPacketTimerData = dccOutPacketTimer.data;
      dccOutPacketTimerCount = dccOutPacketTimer.count;
    }
    break;

  case DATA_EVT:
    bitValIsr = (dccOutPacketTimerData >> bitCountIsr) & 0x01;
    if (bitValIsr == 0)
    {
      if (firstHalfBit == true)
      {
        firstHalfBit = false;
        toggleState = false;
      }
      else
      {
        firstHalfBit = true;
        toggleState = true;
      }
    }

    if (toggleState)
    { // Inversion d'etat
      bitEntier++;
      sens1 = sens0;
      sens0 = !sens0;
      gpio_set_level(pinIn0, sens1);
      gpio_set_level(pinIn1, sens0);
    }

    if (bitEntier == 2)
    {
      bitEntier = 0;
      bitCountIsr++;
    }

    if (bitCountIsr == dccOutPacketTimerCount)
    {
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

void dccOutTask(void *pvParameters)
{
  DCC_PACKET tempPacket = {0, 0};
  while (true)
  {
    if (xQueueReceive(dccOutQueue, &tempPacket, pdMS_TO_TICKS(30)) == pdTRUE)
    {
      dccOutPacketTimer = tempPacket;
      // Serial.println(dccOutPacketTimer.data, BIN);
    }

    else
    {
      dccOutPacketTimer = idlePacket;
      Serial.println("idle");
    }
  }
}

/*-----------------------------------------------------------------------
  setup
------------------------------------------------------------------------*/

void setup()
{
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
  gpio_set_level(pinEnable, HIGH);

  dccInQueue = xQueueCreate(1024, sizeof(uint8_t));
  dccOutQueue = xQueueCreate(32, sizeof(DCC_PACKET));
  //durationQueue = xQueueCreate(1024, sizeof(uint32_t));

  attachInterrupt(digitalPinToInterrupt(pinDCCin), dccInterruptHandler, FALLING); // ou RISING selon câblage

  //************************ Timer ********************************* */
  /* Version 1 https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/api/timer.html

  // Initialisation du timer à 1 MHz (1 tick = 1µs)
  timer = timerBegin(1000000);
  // Attachement de l'interruption
  timerAttachInterrupt(timer, timerHandler);
  // Définition de l'alarme à 58 µs (DCC timing)
  timerAlarm(timer, 58, true, 0);
  */

  /* Version 2 */

  timer = timerBegin(0, 80, true); // timer 0, prescaler 80, count up.
  timerAttachInterrupt(timer, timerHandler, true);
  timerAlarmWrite(timer, 58, true); // 58µS
  timerAlarmEnable(timer);
  //******************************************************************* */

  xTaskCreatePinnedToCore(dccParserTask, "DCC Parser", 4 * 1024, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(dccOutTask, "DCC Out", 4 * 1024, NULL, 4, NULL, 0);

  // Pour activer ou désactiver le debug, commenter ou décommenter la ligne ci-dessous
  // Serial.end();
}

void loop()
{
} // nothing to do
