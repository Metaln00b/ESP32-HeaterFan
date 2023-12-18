#include <Arduino.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "soc/ledc_reg.h"

#define LEDC_TIMER_RES          LEDC_TIMER_10_BIT
#define DUTY_MAX                ((1 << LEDC_TIMER_10_BIT) -1 )
#define FREQ_MIN_Hz             1 /* Do not decrease it! */

#define FAN_PWM_PIN             4
#define PSU_ON_PIN              13

#define NTC_PIN                 34

const double on_temperature_degC = 40.0;
const double off_temperature_degC = on_temperature_degC - 10.0;

const double VCC = 3.3;             // On board 3.3v vcc
const double R2 = 10000;            // 10k ohm series resistor
const double adc_resolution = 4095; // 10-bit adc

const double A = 0.001129148;   // thermistor equation parameters
const double B = 0.000234125;
const double C = 0.0000000876741;

static bool psu_on = false;

void ledc_init(uint8_t pin, float freq_Hz, ledc_channel_t channel, ledc_timer_t timer) {
    const char * ME = __func__;

    esp_err_t err;
    periph_module_enable(PERIPH_LEDC_MODULE);

    uint32_t precision = DUTY_MAX + 1;
    uint32_t div_param = ((uint64_t) LEDC_REF_CLK_HZ << 8) / freq_Hz / precision;
    if (div_param < 256 || div_param > LEDC_DIV_NUM_HSTIMER0_V)
    {
        ESP_LOGE(ME, "requested frequency and duty resolution can not be achieved, try increasing freq_hz or duty_resolution. div_param=%d", (uint32_t ) div_param);
    }

    ledc_channel_config_t ledc_channel = {
      .gpio_num   = pin,
      .speed_mode = LEDC_HIGH_SPEED_MODE,
      .channel    = channel,
      .intr_type  = LEDC_INTR_DISABLE,
      .timer_sel  = timer,
      .duty       = DUTY_MAX,
      .hpoint     = 0         // TODO: AD 10.11.2018: new, does 0 work (0xfffff does not work)
    };
    err = ledc_channel_config(&ledc_channel);
    ESP_LOGD(ME,"ledc_channel_config returned %d",err);
    
    err = ledc_timer_set(LEDC_HIGH_SPEED_MODE, timer, div_param, LEDC_TIMER_RES, LEDC_REF_TICK);
    if (err)
    {
        ESP_LOGE(ME, "ledc_timer_set returned %d",err);
    }
    
    ledc_timer_rst(LEDC_HIGH_SPEED_MODE, timer);
    ESP_LOGD(ME, "ledc_timer_set: divider: 0x%05x duty_resolution: %d\n", (uint32_t) div_param, LEDC_TIMER_RES);
}

void power_on(bool turn_on)
{
    if (turn_on)
    {
        if (!psu_on)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 768);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            digitalWrite(PSU_ON_PIN, HIGH);

            psu_on = !psu_on;

            delay(4000);

            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 320);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        }
    }
    else
    {
        digitalWrite(PSU_ON_PIN, LOW);

        psu_on = !psu_on;
    }

}

void setup() {
    Serial.begin(9600);

    pinMode(PSU_ON_PIN, OUTPUT);
    ledc_init(FAN_PWM_PIN, 22, LEDC_CHANNEL_0, LEDC_TIMER_0);

    delay(1000);
}

void loop() {
    double Vout, Rth, temperature, adc_value; 

    adc_value = analogRead(NTC_PIN);
    Vout = (adc_value * VCC) / adc_resolution;
    Rth = (VCC * R2 / Vout) - R2;

    /*  Steinhart-Hart Thermistor Equation:
    *  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
    *  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
    temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

    temperature = temperature - 273.15;  // Temperature in degree celsius

    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" degC");

    if (temperature > on_temperature_degC)
    {
        power_on(true);
        
    }
    else if (temperature < off_temperature_degC)
    {
        power_on(false);
    }

    delay(500);
}