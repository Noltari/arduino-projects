#include "driver/rtc_io.h"
#include "driver/temp_sensor.h"
#include "esp_sleep.h"
#include "time.h"

#define BOOT_COUNT_INIT 1
#define GPIO_INPUT_PIN GPIO_NUM_0

RTC_FAST_ATTR uint32_t bootCount = 0;

RTC_FAST_ATTR int wakeupLevel = -1;

RTC_FAST_ATTR uint32_t pulseCount = 0;

void send_data(esp_sleep_wakeup_cause_t wakeup_reason, int sleep_time)
{
  float esp32_temp = -300.0;
  esp_err_t temp_err;

  temp_err = temp_sensor_set_config(TSENS_CONFIG_DEFAULT());
  temp_err |= temp_sensor_start();
  temp_err |= temp_sensor_read_celsius(&esp32_temp);

  Serial.begin(115200);
  sleep(sleep_time);
  Serial.printf("wakeup_reason=%lu bootCount=%lu pulseCount=%lu time=%lu esp32_temp=%.3f\n", wakeup_reason, bootCount, pulseCount, time(NULL), esp32_temp);
  Serial.flush();
  Serial.end();
}

void setup(void)
{
  const esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  bootCount++;

  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    if (wakeupLevel)
    {
      pulseCount++;

      send_data(wakeup_reason, 2);
    }

    wakeupLevel = !wakeupLevel;
  }
  else if (bootCount == BOOT_COUNT_INIT)
  {
    rtc_gpio_init(GPIO_INPUT_PIN);
    rtc_gpio_set_direction(GPIO_INPUT_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(GPIO_INPUT_PIN);
    rtc_gpio_pullup_en(GPIO_INPUT_PIN);
    wakeupLevel = !rtc_gpio_get_level(GPIO_INPUT_PIN);

    send_data(wakeup_reason, 5);
  }

  esp_sleep_enable_ext0_wakeup(GPIO_INPUT_PIN, wakeupLevel);
  esp_deep_sleep_start();
}

void loop(void)
{
  /* Not used */
}
