#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Espfc.h>
#include <Kalman.hpp>
#include <Madgwick.hpp>
#include <Mahony.hpp>
#include <printf.h>
#include <blackbox/blackbox.h>
#include <EscDriver.h>
#include <EspWire.h>
#include <Gps.hpp>
#if defined(ESPFC_ESPNOW)
#include <EspNowRcLink/Receiver.h>
#endif
#ifdef ESPFC_WIFI_ALT
#include <ESP8266WiFi.h>
#elif defined(ESPFC_WIFI)
#include <WiFi.h>
#endif

#ifdef ESP32
void IRAM_ATTR serialEventRun(void) {}
#endif

Espfc::Espfc espfc;

#if defined(ESPFC_MULTI_CORE)
  #if defined(ESPFC_FREE_RTOS)

    // ESP32 multicore
    #include <freertos/FreeRTOS.h>
    #include <freertos/task.h>
    #include <driver/timer.h>

  TaskHandle_t gyroTaskHandle = NULL;
TaskHandle_t backgroundTaskHandle = NULL;
volatile uint32_t gyroTaskMissedDeadlines = 0;

static const timer_group_t TIMER_GROUP = TIMER_GROUP_0;
static const timer_idx_t TIMER_IDX = TIMER_0;

bool IRAM_ATTR gyroTimerIsr(void* args)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  vTaskNotifyGiveFromISR(
      gyroTaskHandle,
      &xHigherPriorityTaskWoken);

  return xHigherPriorityTaskWoken == pdTRUE;
}

void gyroTimerInit(bool (*isrCb)(void* args), int interval)
{
  timer_config_t config = {
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 80,
  };

  timer_init(
      TIMER_GROUP,
      TIMER_IDX,
      &config);

  timer_set_counter_value(
      TIMER_GROUP,
      TIMER_IDX,
      0);

  timer_set_alarm_value(
      TIMER_GROUP,
      TIMER_IDX,
      interval);

  timer_isr_callback_add(
      TIMER_GROUP,
      TIMER_IDX,
      isrCb,
      nullptr,
      ESP_INTR_FLAG_IRAM);

  timer_enable_intr(
      TIMER_GROUP,
      TIMER_IDX);

  timer_start(
      TIMER_GROUP,
      TIMER_IDX);
}

void gyroTask(void* pvParameters)
{
  (void)pvParameters;

  espfc.begin();

  gyroTimerInit(
      gyroTimerIsr,
      espfc.getGyroInterval());

  while (true)
  {
const uint32_t notifications =
    ulTaskNotifyTake(
        pdTRUE,
        portMAX_DELAY);

if (notifications > 1)
{
  gyroTaskMissedDeadlines +=
      notifications - 1;
}

espfc.update(true);
  }
}

void backgroundTask(void* pvParameters)
{
  (void)pvParameters;

  while (true)
  {
    while (espfc.updateOther())
    {
      // Drain non-flight-critical queued events.
    }

    vTaskDelay(1);
  }
}

void setup()
{
  disableCore0WDT();

  espfc.load();

  xTaskCreateUniversal(
      gyroTask,
      "gyroTask",
      8192,
      NULL,
      24,
      &gyroTaskHandle,
      1);

  xTaskCreateUniversal(
      backgroundTask,
      "backgroundTask",
      4096,
      NULL,
      1,
      &backgroundTaskHandle,
      0);

  vTaskDelete(NULL);
}

void loop()
{
}
  #elif defined(ESPFC_MULTI_CORE_RP2040)

    bool core1_separate_stack = true;
    volatile bool setup_done = false;

    // RP2040 multicore
    // TODO: https://emalliab.wordpress.com/2021/04/18/raspberry-pi-pico-arduino-core-and-timers/
    void setup()
    {
      espfc.load();
      espfc.begin();
      setup_done = true;
    }
    void loop()
    {
      espfc.update();
    }
    void setup1()
    {
      while(!setup_done);
    }
    void loop1()
    {
      espfc.updateOther();
    }

  #else
    #error "No RTOS defined for multicore board"
  #endif

#else

  // single core
  void setup()
  {
    espfc.load();
    espfc.begin();
  }
  void loop()
  {
    espfc.update();
    espfc.updateOther();
  }

#endif
