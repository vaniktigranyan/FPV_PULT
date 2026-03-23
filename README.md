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
        |  AUX4 (GPIO25) ---[SW]--- GND    (INPUT_PULLUP, ON=LOW)                   |
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
  - AUX4: `GPIO25`
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

**Сборка и прошивка**
```bash
idf.py set-target esp32
idf.py build
idf.py -p COMx flash monitor
```

**Настройки через `#define`**
Основные макросы находятся в:
- `main/main.c` — UART, частота отправки, задачи
- `main/input.c` — GPIO, ADC, калибровка, EMA, уровни AUX

Типичные параметры:
```c
#define FPV_UART_BAUD_RATE 416666
#define FPV_RC_SEND_RATE_HZ 150
#define FPV_ADC_EMA_ALPHA 0.20f
#define FPV_ADC_ROLL_MIN 200
#define FPV_ADC_ROLL_MAX 3900
#define FPV_AUX_ON_VALUE CRSF_CH_MAX
#define FPV_AUX_OFF_VALUE CRSF_CH_MIN
```

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
