# Standard 3-Wire Servo Driver

Reusable ESP-IDF component for ordinary three-wire hobby servos. It uses the ESP32 LEDC peripheral to generate a configurable servo PWM signal.

## Wiring

| Servo wire | Connection |
| --- | --- |
| Black | External servo supply ground and ESP32 GND |
| Red | Separate regulated 5-6 V servo supply |
| Yellow | Configured ESP32 signal GPIO |

Never power the servo from an ESP32 GPIO or 3.3 V pin. A servo can draw large transient current; a `470-1000 uF` electrolytic capacitor across the servo supply is recommended. A three-wire servo does not provide position feedback, so the driver reports only the commanded pulse.

## API

```c
standard_servo_config_t config = {
    .signal_gpio = GPIO_NUM_27,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0,
    .channel = LEDC_CHANNEL_0,
    .duty_resolution = LEDC_TIMER_16_BIT,
    .frequency_hz = 50,
    .min_pulse_us = 1000,
    .max_pulse_us = 2000,
};
standard_servo_handle_t servo;

ESP_ERROR_CHECK(standard_servo_init(&config, &servo));
ESP_ERROR_CHECK(standard_servo_set_pulse_us(servo, 1500));
ESP_ERROR_CHECK(standard_servo_set_angle_deg(servo, 0));
```

The angle API maps `-90..90` degrees linearly to the configured pulse range. Servo pulse limits are model-specific; start near 1500 us and expand the range gradually.
