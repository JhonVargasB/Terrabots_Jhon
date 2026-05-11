# TestMPU6050 - ESP-IDF

Proyecto de prueba para la lectura del sensor MPU6050 utilizando ESP-IDF sobre ESP32-S3.

## Requisitos

* ESP-IDF instalado
* ESP32-S3
* Sensor MPU6050
* VSCode + extensión ESP-IDF (opcional)

Repositorio:

[Repositorio del proyecto](https://github.com/JhonVargasB/Terrabots_Jhon?utm_source=chatgpt.com)

---

# Clonar el repositorio

```bash
git clone https://github.com/JhonVargasB/Terrabots_Jhon.git
```

Ingresar a la carpeta del proyecto:

```bash
cd Terrabots_Jhon/firmware/semana_01/TestMPU6050
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

# Conexiones MPU6050

| MPU6050 | ESP32-S3 |
| ------- | -------- |
| VCC     | 3.3V     |
| GND     | GND      |
| SDA     | GPIO5    |
| SCL     | GPIO6    |

---

# Estructura del proyecto

```txt
TestMPU6050/
├── components/
│   └── mpu/
│       ├── include/
│       │   ├── mpu.h
│       │   └── mpu_defs.h
│       ├── mpu.c
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

* Lectura de acelerómetro
* Lectura de giroscopio
* Conversión a unidades físicas
* Corrección de offset
* Driver modular utilizando `components/`

---

# Configuración I2C

```c
SDA -> GPIO5
SCL -> GPIO6
```

Frecuencia I2C:

```c
400 kHz
```

---

# Framework utilizado

* ESP-IDF v5.x
* FreeRTOS
* Driver I2C Master API
