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
{\LARGE\bfseries I-V curve tracer}\\[4pt]
{\large Quick guide}\\[8pt]
Measures the current-voltage (I-V) curve of a solar panel and computes Voc, Isc and the maximum
power point (MPP). Use it with the knob and the screen, or from a phone over Wi-Fi.
\end{minipage}\hfill
\begin{minipage}[c]{0.16\textwidth}\centering
\includegraphics[width=\linewidth]{docs/img/guide_qr.png}\\
{\small Online guide}
\end{minipage}

## 1. Power on and connect

1. Power on **with the panel unplugged**. At boot it calibrates the current sensor.
2. Connect the panel: positive to **PV+**, negative to **PV-**.
3. **Green** LED: measuring. **Red** LED: a fault (see "Troubleshooting").

## 2. Measuring from a phone

1. **Turn off mobile data** on the phone. Otherwise the phone may ignore the device's network
   because it has no internet.
2. Join the Wi-Fi network **ESP32_PLOT** (no password).
3. Open `http://192.168.4.1` in the browser.
4. Press **Start measurement**. The curve shows up in 10 to 20 seconds.
5. Voc, Isc, Pmax, Vmp and Imp are shown below it. **Download CSV** saves the data.

## 3. The knob

- **Turn**: moves the selection.
- **Press**: selects.
- **Hold**: goes back (in the chart and the manual load).
- The screen turns off after 60 s. The first touch only turns it back on.

## 4. Menus

**MEASURE**

- **CURVE TRACER**: *START TRACE* starts a sweep and *STOP TRACE* stops it. In *MODE*, **REAL**
  measures the panel and **DEMO** shows an example curve.
- **CURVE CHART**: shows the last curve. Turning walks the points. A double press adds the
  power curve.
- **DYNAMIC LOAD** (manual load): turning raises or lowers the load in 10 steps, up to the last
  sweep's Isc. A double press re-measures the range (other light or another panel). Max. 3 W.

**NETWORK**: QR codes for the Wi-Fi network, the web page and this online guide.

**SYSTEM**: **OTA** (update), **RESET** and **DEEP SLEEP** (screen off, saves battery; wakes
with the knob). **BACK** goes back.

\newpage

## 5. What a sweep does

1. Measures the panel's voltage with no load (Voc).
2. Finds the current range by itself. Nothing to configure.
3. Takes up to **40 points** along the curve and stops at Isc.

## 6. Range and precision

- The device was designed for a **wide range**: up to about **26 V**, **780 mA** and **10 W**.
- **The more current, the better the measurement.** With 40 mA or more (good light) the curve
  is accurate.
- **With little current** (below about 20 mA) there are fewer points and a **1 to 2 mA** error.
- The load always draws about **4 mA**, even at rest. So the first point reads about 4 mA. In
  dim light, the no-load voltage can read lower than a multimeter's.

## 7. Tips

- Use even, steady light. Do not move or cover the panel while measuring.
- To lower the current, shade the **whole** panel evenly (paper or cloth), never just a few
  cells.
- Lamp flicker does not matter: it is averaged out.

## 8. Troubleshooting

- **"No panel detected" and red LED**: the panel is unplugged or has **too little light**.
  Light it better.
- **"Panel reversed" and red LED**: swap the PV+ and PV- leads.
- **The page does not load**: turn off mobile data and join ESP32_PLOT again.
- **The curve does not reach 0 V**: the panel gives more than 780 mA, the device's maximum.
- **The sweep stops early**: it hit the 10 W limit.
- **The measurement is imprecise**: the current is very small. Use more light.

## 9. Safety

- The load transistor (MOSFET) heats up during a sweep. This is normal.
- There is no power switch. To turn it off, unplug the battery.

## 10. Updating the firmware

1. While online, download `app-standard.bin` from the latest GitHub release.
2. Turn off mobile data and join **ESP32_PLOT**.
3. Open `http://192.168.4.1/ota` (or the QR in **SYSTEM > OTA**) and upload the file.
4. The device reboots on its own in about 30 seconds.
