Parcial 3 Diego Solis (STM32 Nucleo L053R8)
Semáforo con botón peatonal y paso de tren 

## Descripción 
Proyecto de semáforo programado en C para la STM32L053R8 que usa **timers**, **interrupciones** y **USART**.  
Tiene **dos push buttons** (modo tren y modo peatonal) y permite **configurar el tiempo en ROJO** mediante **keypad 4×4** y **LCD 16×2**. Muestra **MM:SS** en un **display 7 segmentos (4 dígitos)** y posiciona un **stepper 28BYJ-48** según el estado (verde/rojo) y el modo seleccionado.

---

## Características principales
- Control 100% por **timers** e **interrupciones** (sin delays bloqueantes).
- **Keypad 4×4** para menú de configuración (tiempos en MM:SS para tren/peatón).
- **LCD 16×2** para estados y mensajes (libre, precaución, tren pasando, paso peatonal).
- **Display 7 segmentos (4 dígitos)** para conteo **MM:SS** en ROJO.
- **Stepper 28BYJ-48** (half-step) para posicionamiento asociado a VERDE↔ROJO.
- **Buzzer** para modo tren.
- **LEDs** de semáforo: rojo/amarillo/verde.
- **Botones dedicados**: tren y peatonal para lanzar el ciclo desde VERDE.
- **Logs serie** por **USART2 @9600**.

---

## Hardware utilizado (pines y utilidad)

| Módulo                    | Pines MCU                                      | Utilidad / Descripción                          |
|--------------------------|-------------------------------------------------|-------------------------------------------------|
| LCD 16×2 (4-bit)         | RS=PA0, E=PA1, D4=PA8, D5=PA10, D6=PA5, D7=PA6 | Mensajes de estado/menú                         |
| Keypad 4×4               | Filas PB8..PB11 (IN con pull-up), Cols PB12..PB15 (OUT) | Entrada de A/B/C/D, dígitos, `*`, `#`     |
| 7-segmentos 4 dígitos    | Segmentos PB0..PB7, Dígitos PC5/PC6/PC8/PC9     | Mostrar tiempo **MM:SS** en estado ROJO         |
| Stepper 28BYJ-48 + driver| PC3, PC4, PC7, PC11                             | Posición asociada a estados (VERDE/ROJO)        |
| LEDs semáforo            | Rojo=PC0, Amarillo=PC1, Verde=PC2               | Indicadores de estado                            |
| Buzzer                   | PA9                                             | Aviso sonoro (modo tren)                         |
| Botón “Tren”             | PA4 (EXTI)                                      | Inicia ciclo en modo tren desde VERDE            |
| Botón “Peatón”           | PA12 (EXTI)                                     | Inicia ciclo en modo peatonal desde VERDE        |
| USART2                   | PA2 (TX), PA3 (RX) **AF4**                      | Logs a 9600 baudios                              |
| Alimentación             | 5V lógica (MCU/driver)                          | GND común entre módulos                          |

---

## Diseño (maqueta / panel)
- Panel con LEDs de semáforo, keypad accesible y LCD visible.
- Ruteo limpio de señales de stepper y dígitos del 7-segmentos.
- GND común para todo el sistema, separación clara de señales de control y potencia del motor.

---

## Lógica de funcionamiento (resumen)
1. **Home (VERDE):** LCD muestra “Libre” (A: Tren / B: Peatón).  
2. **Selección modo:**  
   - Botón **Tren** (PA4) o **Peatón** (PA12) desde VERDE → pasa a **AMARILLO**.  
3. **AMARILLO (3 s):** LED amarillo activo, en tren suena **buzzer**. Se prepara la **posición del stepper** para ROJO.  
4. **ROJO (MM:SS):**  
   - Modo **Tren**: LCD “Tren pasando”, buzzer ON, cuenta MM:SS en 7-seg.  
   - Modo **Peatón**: LCD “Paso peatonal”, buzzer OFF, cuenta MM:SS.  
   - Al terminar el tiempo, regresa a **VERDE**.  
5. **Configuración de tiempos:**  
   - En **VERDE**: tecla **A** → configurar **Tren**; **B** → **Peatón**.  
   - Ingresar **MM:SS** con dígitos. `*` borra, `#` **guarda** (valida `SS<60`).  

---

## Timers e interrupciones

| Timer/IRQ          | Frecuencia aprox. | Función                  | Descripción breve                                                       |
|--------------------|-------------------|--------------------------|-------------------------------------------------------------------------|
| **TIM21_IRQn**     | **1 kHz**         | Tick del sistema         | Servicio **LCD no bloqueante**, **escaneo keypad**, **antirrebote**, stepper, contadores de estado (amarillo/rojo) y actualización cada 1 s (divisor por software). |
| **TIM22_IRQn**     | **~2 kHz**        | 7-segmentos              | Multiplexado de 4 dígitos (solo activo en estado ROJO).                |
| **EXTI4_15_IRQn**  | Por flanco        | Botones + Keypad (filas) | Lee botones **Tren/Peatón** (PA4/PA12) y detecta presión en filas del keypad. |
| **USART2**         | 9600 baudios      | Logs por TX (cola)       | Envío no bloqueante por interrupción TXE; RX se descarta para limpiar. |

> **Nota stepper:** en `TIM21` se atiende `stp_service_1ms()`. Se avanza un half-step efectivo cada ~3 ticks (≈333 Hz de micro-paso interno). Atajos: `stp_right_90()` / `stp_back_home_90()`.

---

## Estados y mensajes en LCD
- **Libre (VERDE):** “A: Tren  B: Peaton”.  
- **Precaución (AMARILLO):** “Esperando a Rojo”.  
- **Tren pasando (ROJO-TREN)** o **Paso peatonal (ROJO-PEATON)**.  
- Menús de **“Tiempo Tren (MM:SS)”** y **“Tiempo Peaton (MM:SS)”** con edición en vivo.

---

## Valores por defecto
- **Tiempo ROJO Tren:** 30 s  
- **Tiempo ROJO Peatón:** 20 s  
- **AMARILLO:** 3 s fijo

---

## Videos (YouTube)

