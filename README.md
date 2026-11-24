# Monitor de pulso, SpO₂ y temperatura con ESP32

Monitor educativo basado en **ESP32**, sensor óptico **MAX30102** (pulso y SpO₂), sensor infrarrojo **MLX90614** (temperatura superficial), pantalla **TFT ST7789** de 240×280 y un **buzzer activo** que suena con cada latido detectado.

Proyecto de **Ingeniería Electrónica – UNISANGIL, sede Yopal (Casanare, Colombia)**.

**Autores:** Cristian Enrique Cruz Fernandez · Samuel Felipe Diaz Galindo · Santiago Reina Socha

> ⚠️ Uso **educativo**. No es un dispositivo médico. La SpO₂ no está validada frente a un pulsioxímetro de referencia y, en modo canino, no se muestra porcentaje (sin calibración veterinaria).

## Videos de demostración

- [Demostración 1](https://youtube.com/shorts/BoFPV1nKkOI)
- [Demostración 2](https://youtube.com/shorts/UJpttTz2hjk)

## Estructura del repositorio

```
monitor-pulso-esp32/
├── firmware/
│   └── monitor_dual/
│       └── monitor_dual.ino     # Código del ESP32 (Arduino IDE)
├── docs/
│   └── Informe_IEEE_MAX30102.pdf # Informe en formato IEEE
├── README.md
├── LICENSE
└── .gitignore
```

## Hardware

| Módulo | Señal | ESP32 |
|---|---|---|
| MAX30102 | SDA / SCL (I²C, `Wire`) | GPIO 21 / GPIO 22 |
| MLX90614 | SDA / SCL (I²C, `Wire1`) | GPIO 32 / GPIO 33 |
| TFT ST7789 | CS / DC / RST | GPIO 5 / GPIO 2 / GPIO 4 |
| TFT ST7789 | SCLK / MOSI | GPIO 18 / GPIO 23 |
| Buzzer activo | Control | GPIO 25 |

Todos los módulos comparten GND. Alimenta cada módulo con 3,3 V (o según lo admita tu placa).
Direcciones I²C: MAX30102 `0x57`, MLX90614 `0x5A`.

## Bibliotecas (Arduino IDE)

- Adafruit GFX Library
- Adafruit ST7735 and ST7789 Library
- Adafruit BusIO
- SparkFun MAX3010x Pulse and Proximity Sensor Library

Placa: **ESP32 Dev Module**. Monitor serie a **115200** baudios.

## Cómo usarlo

1. Abre `firmware/monitor_dual/monitor_dual.ino` en Arduino IDE.
2. Instala las bibliotecas, selecciona *ESP32 Dev Module* y sube el código.
3. Abre el Monitor Serie (115200) o el Serial Plotter.

### Comandos por puerto serie

Envía una letra (se aceptan CR y LF):

| Letra | Acción |
|---|---|
| `H` | Modo humano |
| `C` | Modo canino |
| `A` | Capturar aire (calibración canina) |
| `T` | Capturar tejido (calibración canina) |
| `M` | Salida en texto |
| `P` | Salida para Serial Plotter |
| `I` | Escanear buses I²C (solo en modo texto) |

### Modo humano

Coloca la yema del dedo quieta sobre el MAX30102. La pantalla muestra la onda de pulso, BPM, SpO₂ y temperatura superficial.

### Modo canino (experimental)

1. Envía `C` y retira cualquier tejido del sensor.
2. Envía `A` y espera la indicación (4 s midiendo aire).
3. Apoya el tejido quieto y envía `T` (4 s midiendo tejido).
4. Repite `A`/`T` si cambia el sitio de medición o la luz.

La calibración se guarda solo en RAM. En este modo se muestra BPM y temperatura; la SpO₂ no se calcula.

## Documentación

El informe completo (fundamento teórico, conexiones, firmware y protocolo de pruebas) está en [`docs/Informe_IEEE_MAX30102.pdf`](docs/Informe_IEEE_MAX30102.pdf).

## Licencia

[MIT](LICENSE)
