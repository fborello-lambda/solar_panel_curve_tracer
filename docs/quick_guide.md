---
geometry: a4paper, margin=1.3cm
fontsize: 9pt
mainfont: DejaVu Sans
header-includes: |
  \usepackage{graphicx}
  \usepackage{titlesec}
  \usepackage{enumitem}
  \usepackage{float}
  \usepackage{caption}
  \captionsetup{labelformat=empty}
  \floatplacement{figure}{H}
  \setlength{\textfloatsep}{1pt}
  \setlength{\intextsep}{0pt}
  \setlength{\floatsep}{1pt}
  \setlength{\abovecaptionskip}{1pt}
  \setlength{\belowcaptionskip}{0pt}
  \titlespacing*{\section}{0pt}{2pt}{1pt}
  \titlespacing*{\subsection}{0pt}{2pt}{1pt}
  \setlist{itemsep=0pt,parsep=0pt,topsep=1pt,partopsep=0pt}
  \setlength{\parskip}{1pt}
  \setlength{\parindent}{0pt}
  \renewcommand{\arraystretch}{0.85}
  \linespread{0.93}
  \pagenumbering{gobble}
---

# Guía rápida (Español)

## Qué hace

Este dispositivo mide y traza la curva I-V (corriente vs. tensión) de un
panel solar pequeño, calculando el punto de máxima potencia (MPP).

## Seguridad

- El MOSFET de carga se calienta durante el barrido: es normal.
- El barrido se detiene automáticamente si la potencia llega a 5 W.
- La carga está limitada a unos 780 mA (20% de la escala completa).
- Este prototipo no tiene interruptor de encendido: desconecte la batería
  para apagarlo.

## Conectar el panel y encender

- Encienda primero **sin el panel conectado**: al arrancar se calibra el cero del sensor.
- Luego conecte el panel: positivo a **PV+**, negativo a **PV-**.

## Medir desde la pantalla OLED

1. Gire el encoder para ir a **MEASURE**, presione para entrar.
2. Seleccione **CURVE TRACER**, presione para entrar.
3. Seleccione **START TRACE** y presione para iniciar el barrido.
4. Girar mueve la selección; presionar confirma.

## Medir desde un teléfono o laptop

1. Conéctese a la red Wi-Fi **ESP32_PLOT** (sin contraseña).
2. Abra `http://192.168.4.1` en el navegador.
3. Presione **Start** para iniciar el barrido y **Stop** para detenerlo
   (selector de idioma ES/EN y tema claro/oscuro incluidos).
4. Guía completa: `http://192.168.4.1/guide`, u OLED **NETWORK > SHOW GUIDE QR**.

## Qué sucede durante un barrido

1. Se mide la tensión de circuito abierto (Voc).
2. Se busca automáticamente el rango de corriente adecuado.
3. Se registran hasta 40 puntos, repartidos a lo largo de toda la
   curva.
4. El barrido completo toma entre 10 y 20 segundos.

## Leer el resultado

- **Voc**: tensión de circuito abierto.
- **Isc**: corriente de cortocircuito.
- **MPP**: punto de máxima potencia (tensión y corriente).
- La forma general de la curva indica el estado del panel.
- Setup de la práctica de laboratorio: Isc aprox. 50 mA, resolución aprox. 1 mA.

\begin{center}\includegraphics[width=0.34\textwidth]{docs/img/iv_example.png}\\[-2pt]{\small Ejemplo de curva I-V}\end{center}
\vspace{-14pt}

## Consejos

- Mantenga la iluminación estable durante el barrido.
- No mueva ni tape el panel mientras mide.
- El parpadeo de lámparas se promedia automáticamente, no afecta la medida.

## Solución de problemas

| Problema | Causa probable |
|---|---|
| No se registran puntos | Voc < 0.5 V: panel desconectado u oscuridad |
| La curva no llega a 0 V | El panel supera el límite de carga del 20% |
| El barrido se detiene antes | Se alcanzó el límite de seguridad de 5 W |
| La página web está vacía | Reconéctese a la red Wi-Fi ESP32_PLOT |

## Actualizar firmware, bajo consumo y guía en línea

- **OTA**: descargue `app-standard.bin` de la última versión en GitHub,
  conéctese a **ESP32_PLOT**, abra `http://192.168.4.1/ota` (o QR en OLED
  **SYSTEM > OTA**), suba el archivo y espere el reinicio (~30 s).
- **Bajo consumo**: OLED **SYSTEM > DEEP SLEEP**; despierta con el botón
  del encoder.
- **Guía en línea** (QR a la derecha): github.com/fborello-lambda/solar\_panel\_curve\_tracer
  \raisebox{-0.9\height}{\includegraphics[width=0.07\textwidth]{docs/img/guide_qr.png}}

\newpage

# Quick Guide (English)

## What it does

This device measures and traces the I-V curve (current vs. voltage) of a
small solar panel, computing the maximum power point (MPP).

## Safety

- The load MOSFET heats up during a sweep: this is normal.
- The sweep aborts automatically if power reaches 5 W.
- The load is capped at about 780 mA (20% of full scale).
- This prototype has no power switch: unplug the battery to turn it off.

## Connecting the panel and powering on

- Power on first **with the panel unplugged**: the sensor zero is calibrated at boot.
- Then connect the panel: positive to **PV+**, negative to **PV-**.

## Measuring from the OLED

1. Turn the encoder to **MEASURE**, press to select.
2. Select **CURVE TRACER**, press to enter.
3. Select **START TRACE** and press to begin the sweep.
4. Turning navigates; pressing selects.

## Measuring from a phone or laptop

1. Connect to Wi-Fi network **ESP32_PLOT** (no password).
2. Open `http://192.168.4.1` in a browser.
3. Press **Start** to begin the sweep and **Stop** to end it (ES/EN
   language switch and light/dark theme included).
4. Full on-device guide: `http://192.168.4.1/guide`, or OLED
   **NETWORK > SHOW GUIDE QR**.

## What happens during a sweep

1. Open-circuit voltage (Voc) is measured.
2. The current range is found automatically.
3. Up to 40 points are recorded, spread along the whole curve shape.
4. A full sweep takes about 10 to 20 seconds.

## Reading the result

- **Voc**: open-circuit voltage.
- **Isc**: short-circuit current.
- **MPP**: maximum power point (voltage and current).
- Overall curve shape indicates the panel's condition.
- Lab practice setup: Isc about 50 mA, resolution about 1 mA.

\begin{center}\includegraphics[width=0.34\textwidth]{docs/img/iv_example.png}\\[-2pt]{\small Example I-V curve}\end{center}
\vspace{-14pt}

## Tips

- Keep lighting steady during the sweep.
- Do not move or shade the panel while measuring.
- Flickering lamps are averaged out automatically; no need to worry.

## Troubleshooting

| Problem | Likely cause |
|---|---|
| No points recorded | Voc < 0.5 V: panel disconnected or dark |
| Curve does not reach 0 V | Panel is stronger than the 20% load cap |
| Sweep stopped early | Hit the 5 W safety limit |
| Web page is empty | Reconnect to the ESP32_PLOT Wi-Fi network |

## Updating firmware, deep sleep and online guide

- **OTA**: download `app-standard.bin` from the latest GitHub release,
  connect to **ESP32_PLOT**, open `http://192.168.4.1/ota` (or QR at OLED
  **SYSTEM > OTA**), upload the file, and wait for the reboot (~30 s).
- **Deep sleep**: OLED **SYSTEM > DEEP SLEEP**; wakes up with a press of
  the encoder button.
- **Online guide** (QR on the right): github.com/fborello-lambda/solar\_panel\_curve\_tracer
  \raisebox{-0.9\height}{\includegraphics[width=0.07\textwidth]{docs/img/guide_qr.png}}
