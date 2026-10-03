---
geometry: a4paper, margin=1.6cm
fontsize: 12pt
mainfont: DejaVu Sans
header-includes: |
  \usepackage{graphicx}
  \usepackage{titlesec}
  \usepackage{enumitem}
  \usepackage{ragged2e}
  \AtBeginDocument{\RaggedRight}
  \titleformat{\section}{\Large\bfseries}{}{0pt}{}
  \titleformat{\subsection}{\large\bfseries}{}{0pt}{}
  \titlespacing*{\subsection}{0pt}{14pt}{5pt}
  \setlist{itemsep=3pt,topsep=3pt,leftmargin=18pt}
  \setlength{\parskip}{5pt}
  \setlength{\parindent}{0pt}
  \linespread{1.1}
  \pagenumbering{gobble}
---

\noindent\begin{minipage}[c]{0.80\textwidth}
{\LARGE\bfseries Trazador de curvas I-V}\\[4pt]
{\large Guía rápida}\\[8pt]
Mide la curva corriente-tensión (I-V) de un panel solar y calcula Voc, Isc y el punto de máxima
potencia (MPP). Se usa con la perilla y la pantalla, o desde el teléfono por Wi-Fi.
\end{minipage}\hfill
\begin{minipage}[c]{0.16\textwidth}\centering
\includegraphics[width=\linewidth]{docs/img/guide_qr.png}\\
{\small Guía en línea}
\end{minipage}

## 1. Encender y conectar

1. Encienda el equipo **con el panel desconectado**. Al arrancar calibra el sensor de corriente.
2. Conecte el panel: positivo a **PV+**, negativo a **PV-**.
3. LED **verde**: está midiendo. LED **rojo**: hubo una falla (ver "Problemas comunes").

## 2. Medir desde el teléfono

1. **Desactive los datos móviles** del teléfono. Si no, el teléfono puede ignorar la red del
   equipo porque no tiene internet.
2. Conéctese a la red Wi-Fi **ESP32_PLOT** (sin contraseña).
3. Abra `http://192.168.4.1` en el navegador.
4. Presione **Iniciar medición**. En 10 a 20 segundos aparece la curva.
5. Arriba se ven Voc, Isc, Pmax, Vmp e Imp. **Descargar CSV** guarda los datos.

## 3. La perilla

- **Girar**: mueve la selección.
- **Presionar**: elige.
- **Mantener presionado**: vuelve atrás (en el gráfico y en la carga manual).
- La pantalla se apaga sola después de 60 s. El primer toque solo la vuelve a encender.

## 4. Los menús

Los menús están en inglés. Entre paréntesis está su significado.

**MEASURE** (medir)

- **CURVE TRACER** (trazar curva): *START TRACE* inicia un barrido y *STOP TRACE* lo detiene.
  En *MODE*, **REAL** mide el panel y **DEMO** muestra una curva de ejemplo.
- **CURVE CHART** (ver la curva): muestra la última curva. Girar recorre los puntos.
  Doble pulsación agrega la curva de potencia.
- **DYNAMIC LOAD** (carga manual): girar sube o baja la carga en 10 pasos, hasta la Isc del
  último barrido. Doble pulsación vuelve a medir el rango (con otra luz u otro panel).
  Máximo 3 W.

**NETWORK** (red): códigos QR de la red Wi-Fi, de la página web y de esta guía en línea.

**SYSTEM** (sistema): **OTA** (actualizar), **RESET** (reiniciar) y **DEEP SLEEP**
(apagar la pantalla y ahorrar batería; se despierta con la perilla). **BACK** = volver.

\newpage

## 5. Qué hace un barrido

1. Mide la tensión del panel sin carga (Voc).
2. Busca solo el rango de corriente. No hay que configurar nada.
3. Toma hasta **40 puntos** a lo largo de la curva y termina al llegar a Isc.

## 6. Rango y precisión

- El equipo fue diseñado para un **rango amplio**: hasta unos **26 V**, **780 mA** y **10 W**.
- **Cuanta más corriente, mejor la medición.** Con 40 mA o más (buena luz) la curva sale
  precisa.
- **Con poca corriente** (menos de unos 20 mA) hay menos puntos y un error de **1 a 2 mA**.
- La carga siempre toma unos **4 mA**, aun en reposo. Por eso el primer punto marca unos 4 mA.
  Con poca luz, la tensión sin carga puede salir menor que la que mide un multímetro.

## 7. Consejos

- Use luz pareja y estable. No mueva ni tape el panel mientras mide.
- Para bajar la corriente, sombree **todo** el panel por igual (con papel o tela), nunca solo
  algunas celdas.
- El parpadeo de las lámparas no afecta: se promedia solo.

## 8. Problemas comunes

- **"No se detecta panel" y LED rojo**: el panel está desconectado o tiene **muy poca luz**.
  Ilumínelo mejor.
- **"Panel invertido" y LED rojo**: intercambie los cables de PV+ y PV-.
- **La página no carga**: desactive los datos móviles y vuelva a conectarse a ESP32_PLOT.
- **La curva no llega a 0 V**: el panel da más de 780 mA, el máximo del equipo.
- **El barrido se corta antes**: se llegó al límite de 10 W.
- **La medición es poco precisa**: la corriente es muy chica. Use más luz.

## 9. Seguridad

- El transistor de carga (MOSFET) se calienta durante el barrido. Es normal.
- El equipo no tiene interruptor. Para apagarlo, desconecte la batería.

## 10. Actualizar el programa

1. Con internet, descargue `app-standard.bin` de la última versión en GitHub.
2. Desactive los datos móviles y conéctese a **ESP32_PLOT**.
3. Abra `http://192.168.4.1/ota` (o el QR en **SYSTEM > OTA**) y suba el archivo.
4. El equipo se reinicia solo en unos 30 segundos.
