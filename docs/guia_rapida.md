---
geometry: a4paper, margin=1cm
fontsize: 11pt
mainfont: DejaVu Sans
header-includes: |
  \usepackage{graphicx}
  \usepackage{multicol}
  \usepackage{titlesec}
  \usepackage{enumitem}
  \titleformat{\section}{\large\bfseries}{}{0pt}{}
  \titleformat{\subsection}{\normalsize\bfseries}{}{0pt}{}
  \titlespacing*{\section}{0pt}{0pt}{2pt}
  \titlespacing*{\subsection}{0pt}{4pt}{1pt}
  \setlist{itemsep=0pt,parsep=0pt,topsep=1pt,partopsep=0pt,leftmargin=12pt}
  \setlength{\parskip}{1.5pt}
  \setlength{\parindent}{0pt}
  \setlength{\columnsep}{14pt}
  \renewcommand{\arraystretch}{0.9}
  \pagenumbering{gobble}
  \newcommand{\bmc}{\begin{multicols}{2}}
  \newcommand{\emc}{\end{multicols}}
---

# Trazador de curvas I-V: guía rápida

Mide la curva corriente-tensión (I-V) de un panel solar y calcula Voc, Isc y el punto de máxima
potencia (MPP). Funciona sin internet: se maneja desde la pantalla OLED con la perilla o desde un
teléfono por Wi-Fi.

\bmc

## 1. Encender y conectar

1. Encienda el equipo **con el panel desconectado**: al arrancar calibra el cero del sensor de
   corriente.
2. Conecte el panel: positivo a **PV+**, negativo a **PV-**.
3. LED **verde** = midiendo. LED **rojo** = falla (sin panel, panel invertido o carga sin
   respuesta); se apaga al iniciar otra medición.

## 2. Perilla y menús (pantalla OLED)

- **Girar**: mover la selección (se detiene en el primer y último ítem).
- **Presionar**: elegir. **Mantener** (0,7 s): volver (en el gráfico).
- La pantalla se apaga sola tras 60 s sin uso; el primer toque solo la enciende.

**NETWORK**: QR de la red Wi-Fi, QR de la dirección web y QR del repositorio en GitHub (guía en línea).

**MEASURE**:

- **CURVE TRACER**: *START/STOP TRACE* inicia o detiene un barrido. *MODE*: **REAL** mide el
  panel; **DEMO** muestra una curva de ejemplo sin usar el panel.
- **CURVE CHART**: la última curva en pantalla. Girar recorre los puntos (abajo se ven V e I);
  **doble pulsación** superpone la curva de potencia (abajo, mW); mantener para volver.
- **DYNAMIC LOAD**: carga manual. Girar sube o baja la corriente que se toma del panel; muestra
  I, potencia y tensión. Máx. 10 % de la escala.

**SYSTEM**: QR de actualización (OTA), reinicio y bajo consumo (despierta con la perilla).

## 3. Medir desde el teléfono

1. Conéctese a la Wi-Fi **ESP32_PLOT** (sin contraseña).
2. Abra `http://192.168.4.1` y presione **Start** (o **Stop**).
3. Se ven la curva, Voc, Isc, Pmax, Vmp e Imp; **CSV** descarga los datos; **Guía** abre la
   guía completa. Selector ES/EN y tema claro/oscuro.

## 4. Qué hace un barrido (10 a 20 s)

1. Mide la tensión sin carga.
2. Busca solo el rango de corriente (no hay que configurarlo).
3. Toma hasta **40 puntos** repartidos a lo largo de toda la curva y se detiene al llegar a Isc.

\columnbreak

## 5. Rango y precisión

- Diseñado para un **rango amplio**: hasta unos **26 V**, **780 mA** y **5 W** en la carga.
- **Más corriente, mejor medición.** Con 40 a 70 mA o más (buena luz) la curva sale fina y
  precisa.
- **Corrientes chicas** (menos de unos 20 mA): pocos puntos y error de **1 a 2 mA**.
- La carga nunca se apaga del todo: toma unos **4 mA** aun en reposo. Por eso el primer punto
  marca unos 4 mA, y en un panel con poca luz la tensión "sin carga" sale menor que la de un
  multímetro con el panel suelto. Es lo que realmente se mide.

## 6. Consejos

- Luz pareja y estable; no mueva ni tape el panel durante el barrido.
- Para bajar la corriente, sombree **todo** el panel por igual (papel o tela), nunca solo algunas
  celdas.
- El parpadeo de lámparas se promedia solo.

## 7. Problemas comunes

- **"No se detecta panel", LED rojo**: panel desconectado o **con muy poca luz** (menos de
  0,5 V). Ilumínelo mejor.
- **"Panel invertido", LED rojo**: intercambie PV+ y PV-.
- **La curva no llega a 0 V**: el panel supera el tope de 780 mA.
- **El barrido se corta**: se llegó al límite de 5 W.
- **Medición poco precisa**: corriente muy chica; más luz.
- **Página web vacía**: vuelva a conectarse a la red ESP32_PLOT.

## 8. Seguridad y actualización

- El MOSFET de carga se calienta durante el barrido: es normal.
- Sin interruptor: para apagar, desconecte la batería.
- **Actualizar**: descargue `app-standard.bin` de la última versión en GitHub, conéctese a
  ESP32_PLOT, abra `http://192.168.4.1/ota` (o QR en **SYSTEM > OTA**) y súbalo. Es un solo
  archivo; reinicia solo en unos 30 s.

\begin{center}
\includegraphics[width=0.12\textwidth]{docs/img/guide_qr.png}\\
{\small Guía en línea}
\end{center}

\emc
