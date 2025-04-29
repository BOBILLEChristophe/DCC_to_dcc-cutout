#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

// Tâche pour le cœur 0
void core0_task(void *pvParameters) {
    while (true) {
        Serial.println("Core 0 Task");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}

// Tâche pour le cœur 1
void core1_task(void *pvParameters) {
    while (true) {
        Serial.println("Core 1 Task");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Délai de 1 seconde
    }
}

void setup() {
    Serial.begin(115200);

    // Créer une tâche sur le cœur 0
    xTaskCreate(core0_task, "Core 0 Task", 256, NULL, 1, NULL);

    // Créer une tâche sur le cœur 1
    //xTaskCreatePinnedToCore(core1_task, "Core 1 Task", 256, NULL, 1, NULL, 1);
}

void loop() {
    // Le planificateur FreeRTOS prend le contrôle ici
}
