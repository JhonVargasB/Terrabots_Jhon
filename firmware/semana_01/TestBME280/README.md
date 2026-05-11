# TestBME280 - ESP-IDF

Proyecto de prueba para la lectura del sensor BME280 utilizando ESP-IDF sobre ESP32-S3 mediante comunicación I2C.

## Placa utilizada

* Seeed Studio XIAO ESP32-S3

Repositorio:

[Repositorio del proyecto](https://github.com/JhonVargasB/Terrabots_Jhon?utm_source=chatgpt.com)

---

# Clonar el repositorio

```bash
git clone https://github.com/JhonVargasB/Terrabots_Jhon.git
```

Ingresar a la carpeta del proyecto:

```bash
cd Terrabots_Jhon/firmware/semana_01/TestBME280
```

---

# Seleccionar el target

El proyecto fue desarrollado para ESP32-S3.

Ejecutar:

```bash
idf.py set-target esp32s3
```

---

# Compilar el proyecto

```bash
idf.py build
```

---

# Flashear al ESP32

```bash
idf.py flash
```

---

# Monitor serial

```bash
idf.py monitor
```

---

# Conexiones BME280

| BME280    | XIAO ESP32-S3 |
| --------- | ------------- |
| VIN / VCC | 3.3V          |
| GND       | GND           |
| SDA       | GPIO5         |
| SCL       | GPIO6         |

---

# Estructura del proyecto

```txt
TestBME280/
├── components/
│   └── bme280/
│       ├── include/
│       │   ├── bme280.h
│       │   └── bme280_defs.h
│       ├── bme280.c
│       └── CMakeLists.txt
│
├── main/
│   ├── main.c
│   └── CMakeLists.txt
│
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

---

# Funcionalidades

* Lectura de temperatura
* Lectura de humedad
* Lectura de presión atmosférica
* Driver modular utilizando `components/`
* Comunicación I2C mediante ESP-IDF

---

# Configuración I2C

```c
SDA -> GPIO5
SCL -> GPIO6
```

Frecuencia I2C:

```c
100 kHz
```

---

# Salida esperada

```txt
datos obtenidos: T = 24.53 , H = 45.21 , P = 101325.00
```

---

# Framework utilizado

* ESP-IDF v5.x
* FreeRTOS
* Driver I2C Master API
