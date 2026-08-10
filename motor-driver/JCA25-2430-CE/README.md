# JGA25-2430-CE Motor Driver

Reusable ESP-IDF component for the five-wire JGA25-2430-CE brushless gear motor with integrated controller.

The directory name follows the requested library layout: `motor-driver/JCA25-2430-CE`. The motor marking is commonly written `JGA25-2430-CE`.

## Interface

The driver generates one active-low PWM signal for the motor's white wire and exposes a direction GPIO. It uses one timer, operator, comparator, and generator from an ESP32 MCPWM group.

```c
#include "jga25_2430_ce.h"

jga25_2430_ce_config_t config = {
    .pwm_gpio = GPIO_NUM_25,
    .direction_gpio = GPIO_NUM_26,
    .pwm_frequency_hz = 20000,
    .pwm_resolution_hz = 1000000,
    .mcpwm_group_id = 0,
};
jga25_2430_ce_handle_t motor;

ESP_ERROR_CHECK(jga25_2430_ce_init(&config, &motor));
ESP_ERROR_CHECK(jga25_2430_ce_set_direction(motor, false));
ESP_ERROR_CHECK(jga25_2430_ce_set_duty_percent(motor, 25));
ESP_ERROR_CHECK(jga25_2430_ce_stop(motor));
```

`0%` is implemented as a forced-high output because this motor's controller treats a low PWM input as the full-speed condition. The PWM input accepts the documented 15-25 kHz range; the component defaults to 20 kHz.

## Wiring

Use a regulated external supply matching the motor label, either 12 V or 24 V. Connect the supply negative to ESP32 GND.

| Motor wire | Connection |
| --- | --- |
| Red | External motor supply positive |
| Black | External supply negative and ESP32 GND |
| White | Configured PWM GPIO |
| Orange | Configured direction GPIO, or GND for fixed direction |
| Yellow | Optional pulse feedback; use a 3.3 V pull-up |

Never connect the red motor wire to an ESP32 GPIO or 3.3 V pin. Do not leave the orange direction input floating.

## 620 RPM variant

The published 620 RPM variant is listed with a gearbox ratio near 9.6:1, approximately 450 RPM at rated load, 0.26 kgf.cm rated output torque, and 1.0 kgf.cm stall torque. These values are variant-specific and should be treated as reference values until the motor label or datasheet is confirmed.

Four motors at that rating provide approximately 0.10 N.m combined rated output torque before drivetrain losses, which is suitable for a light 2 kg car on a flat smooth surface with small wheels. Stall torque is not a continuous operating rating.
