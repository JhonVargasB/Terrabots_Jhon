# Configuración básica de GPIO con ESP32

Proyecto básico utilizando ESP-IDF para la configuración y control de GPIO en la placa XIAO ESP32S3.

## Hardware utilizado

- Seeed Studio XIAO ESP32S3

## Software utilizado

- ESP-IDF v5.4.0

## Descripción

El código realiza:

- Configuración del GPIO21
- Inicialización del hardware GPIO
- Ejecución de un bucle infinito
- Impresión de mensajes por monitor serial

## Pines utilizados

| GPIO | Configuración |
|---|---|
| GPIO21 | GPIO configurado |

## Compilación

```bash id="w3b02c"
idf.py build