# Wired CRSF Controller (ESP32-WROOM, ESP-IDF 5.2)

Проект: проводной пульт управления дроном/автопилотом по протоколу CRSF через UART.  
ESP32 читает 4 аналоговые оси (Roll, Pitch, Throttle, Yaw) и 4 переключателя (AUX1..AUX4), формирует 16 каналов CRSF и отправляет кадры `RC_CHANNELS_PACKED (0x16)` с частотой `150 Hz`. Входящая телеметрия парсится и логируется в консоль.

**Ключевые возможности**
1. CRSF TX по UART2 (full duplex) и RX-парсер телеметрии с CRC8.
2. ADC1 + EMA-фильтрация и фиксированная калибровка min/max.
3. 4 AUX переключателя с `INPUT_PULLUP` (логика: ON=LOW).
4. Логирование этапов: raw ADC, filtered, calibrated, CRSF channels, TX и RX telemetry.

**Аппаратная часть**
- MCU: ESP32-WROOM
- 2 RC gimbal (4 аналоговые оси)
- 4 цифровых переключателя ON/OFF

**Подключение UART (CRSF)**
- ESP32 `TX` -> FC `RX`
- ESP32 `RX` -> FC `TX`
- `GND` -> `GND`

**Схема подключения (тумблеры, джойстики, UART)**
```
          +-------------------+                       +-------------------+
          |   RC Gimbal #1    |                       |   RC Gimbal #2    |
          | (Roll / Pitch)    |                       | (Throttle / Yaw)  |
          |                   |                       |                   |
3.3V -----+ VCC               |                       | VCC  +------------+----- 3.3V
GND -----+ GND               GND----------------------- GND  +------------+----- GND
GPIO32 --+ Roll OUT           |                       | Throttle OUT ----+---- GPIO34
GPIO33 --+ Pitch OUT          |                       | Yaw OUT ---------+---- GPIO35
          +-------------------+                       +-------------------+

        +------------------------------ ESP32-WROOM ------------------------------+
        |                                                                           |
        |  UART2 TX (GPIO17)  --------------------------->  FC RX (CRSF)            |
        |  UART2 RX (GPIO16)  <---------------------------  FC TX (CRSF)            |
        |  GND  ---------------------------------------------------- GND            |
        |                                                                           |
        |  AUX1 (GPIO21) ---[SW]--- GND    (INPUT_PULLUP, ON=LOW)                   |
        |  AUX2 (GPIO22) ---[SW]--- GND    (INPUT_PULLUP, ON=LOW)                   |
        |  AUX3 (GPIO23) ---[SW]--- GND    (INPUT_PULLUP, ON=LOW)                   |
        |  AUX4 (GPIO19) ---[SW]--- GND    (INPUT_PULLUP, ON=LOW)                   |
        |                                                                           |
        +---------------------------------------------------------------------------+

Примечания:
- Питание gimbal: `3.3V`, общий `GND` обязателен.
- Переключатели подключаются к `GND`, включён внутренний `PULLUP`.
- CRSF UART — full duplex: TX/RX перекрёстно.
```

**GPIO по умолчанию**
- ADC1 оси:
  - Roll: `GPIO32` (ADC1_CH4)
  - Pitch: `GPIO33` (ADC1_CH5)
  - Throttle: `GPIO34` (ADC1_CH6)
  - Yaw: `GPIO35` (ADC1_CH7)
- Переключатели (INPUT_PULLUP, active-low):
  - AUX1: `GPIO21`
  - AUX2: `GPIO22`
  - AUX3: `GPIO23`
  - AUX4: `GPIO19`
- UART2:
  - TX: `GPIO17`
  - RX: `GPIO16`
  - Baud: `416666 8N1`

**Каналы CRSF**
- `CH1` = Roll
- `CH2` = Pitch
- `CH3` = Throttle
- `CH4` = Yaw
- `CH5` = AUX1
- `CH6` = AUX2
- `CH7` = AUX3
- `CH8` = AUX4
- `CH9..CH16` = center (`992`)

**Структура проекта**
- `main/main.c` — запуск, задачи TX/RX, UART
- `main/crsf.c`, `main/crsf.h` — CRC8, упаковка RC channels, формат CRSF
- `main/crsf_parser.c`, `main/crsf_parser.h` — парсер входящих CRSF кадров
- `main/input.c`, `main/input.h` — ADC + фильтрация + калибровка + AUX
- `main/telemetry.c`, `main/telemetry.h` — обработка и логирование telemetry
- `main/configs.h` — все изменяемые `#define` (пины, калибровка, режимы)

**Сборка и прошивка**
```bash
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

**Настройки через `#define` (все находятся в `main/configs.h`)**

**UART / задачи / логи**
| Макрос | Значение по умолчанию | Назначение |
|---|---|---|
| `FPV_UART_PORT` | `UART_NUM_2` | UART порт для CRSF |
| `FPV_UART_TX_GPIO` | `GPIO17` | TX пин UART |
| `FPV_UART_RX_GPIO` | `GPIO16` | RX пин UART |
| `FPV_UART_BAUD_RATE` | `416666` | Скорость CRSF |
| `FPV_UART_RX_BUFFER_SIZE` | `1024` | Размер RX буфера UART |
| `FC_RX_ENABLE` | `0` | Включить/выключить RX парсер (0 = отключить) |
| `FPV_RC_SEND_RATE_HZ` | `150` | Частота отправки RC пакетов |
| `FPV_TASK_STACK_SIZE` | `4096` | Размер стека задач |
| `FPV_TASK_PRIORITY_TX` | `8` | Приоритет TX задачи |
| `FPV_TASK_PRIORITY_RX` | `7` | Приоритет RX задачи |
| `FPV_VERBOSE_LOG_EVERY_N_PACKETS` | `25` | Частота подробных логов |
| `FPV_INFO_LOG_EVERY_N_PACKETS` | `150` | Частота обычных логов |

**Режимы и базовые флаги**
| Макрос | Значение по умолчанию | Назначение |
|---|---|---|
| `calibration_joystick` | `0` | Включить режим калибровки при старте |
| `USING_CALIBRATE` | `1` | Использовать данные калибровки из NVS |
| `FPV_VERBOSE_DEBUG` | `1` | Включить подробный debug в консоль |
| `FPV_CRSF_DEADBAND` | `7` | Deadband (в CRSF единицах) для CH1..CH4 |

**Железо, фильтрация, калибровка**
| Макрос | Значение по умолчанию | Назначение |
|---|---|---|
| `FPV_ADC_*_GPIO` | `GPIO32/33/34/35` | Пины осей Roll/Pitch/Throttle/Yaw |
| `FPV_ADC_*_CH` | `ADC_CHANNEL_4..7` | Каналы ADC1 |
| `FPV_ADC_ATTEN` | `ADC_ATTEN_DB_12` | Аттенюация ADC |
| `FPV_ADC_BITWIDTH` | `ADC_BITWIDTH_DEFAULT` | Разрядность ADC |
| `FPV_ADC_*_MIN/MAX` | `200/3900` | Мин/макс сырого ADC (по умолчанию) |
| `FPV_*_INVERT` | `0` | Инверсия осей (0/1) |
| `FPV_ADC_EMA_ALPHA` | `0.20f` | EMA фильтр (чем больше — тем быстрее) |
| `FPV_SWITCH_AUX*_GPIO` | `GPIO21/22/23/19` | Пины AUX переключателей |
| `FPV_SWITCH_ON_LEVEL` | `0` | ON = LOW (pull-up) |
| `FPV_AUX_ON_VALUE` | `CRSF_CH_MAX` | Значение AUX при ON |
| `FPV_AUX_OFF_VALUE` | `CRSF_CH_MIN` | Значение AUX при OFF |
| `FPV_BUZZER_GPIO` | `GPIO5` | Пин пищалки |
| `FPV_BUZZER_ACTIVE_LEVEL` | `1` | Активный уровень пищалки |
| `FPV_BUZZER_BEEP_MS` | `80` | Длительность писка |
| `FPV_BUZZER_GAP_MS` | `60` | Пауза между писками |
| `FPV_BUZZER_STARTUP_ENABLE` | `1` | Включить мелодию при старте |
| `FPV_BUZZER_STARTUP_GAP_MS` | `80` | Пауза между писками мелодии |
| `FPV_BUZZER_STARTUP_BEEP1_MS` | `60` | Длительность писка 1 |
| `FPV_BUZZER_STARTUP_BEEP2_MS` | `90` | Длительность писка 2 |
| `FPV_BUZZER_STARTUP_BEEP3_MS` | `130` | Длительность писка 3 |
| `FPV_CALIB_CENTER_MIN/MAX` | `1600/2100` | Диапазон центра для калибровки |
| `FPV_CALIB_MIN_MIN/MAX` | `0/200` | Диапазон минимума для калибровки |
| `FPV_CALIB_MAX_MIN/MAX` | `3600/4096` | Диапазон максимума для калибровки |
| `FPV_CALIB_EXTREME_STABLE_DELTA` | `120` | Допуск стабильности в экстремуме |
| `FPV_CALIB_EXTREME_STABLE_MS` | `800` | Время удержания экстремума |

**Логирование**
- Базовые логи всегда через `ESP_LOGI/W/E`.
- Подробный debug включается через `#define FPV_VERBOSE_DEBUG 1` в `main/input.h`.

**CRSF формат**
CRSF кадр:
```
address | length | type | payload | crc
```
CRC8 рассчитывается по `type + payload` с полиномом `0xD5`.

**Важно**
- Используется `ADC1` и `adc_oneshot` (ESP-IDF 5.2).
- Полная совместимость с ESP-IDF 5.2 подтверждена сборкой `idf.py build`.
