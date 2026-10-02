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

# I-V curve tracer: quick guide

Measures the current-voltage (I-V) curve of a solar panel and computes Voc, Isc and the maximum
power point (MPP). Works without internet: use the OLED screen and the knob, or a phone over
Wi-Fi.

\bmc

## 1. Power on and connect

1. Power on **with the panel unplugged**: at boot the current sensor zero is calibrated.
2. Connect the panel: positive to **PV+**, negative to **PV-**.
3. **Green** LED = measuring. **Red** LED = fault (no panel, reversed panel or load not
   responding); it clears when a new measurement starts.

## 2. Knob and menus (OLED screen)

- **Turn**: move the selection (stops at the first and last item).
- **Press**: select. **Hold** (0.7 s): go back (in the chart).
- The screen turns off after 60 s idle; the first touch only wakes it.

**NETWORK**: Wi-Fi QR, web address QR and GitHub repository QR (online guide).

**MEASURE**:

- **CURVE TRACER**: *START/STOP TRACE* starts or stops a sweep. *MODE*: **REAL** measures the
  panel; **DEMO** shows an example curve without using the panel.
- **CURVE CHART**: the last curve on screen. Turning walks the points (V and I shown below);
  **double press** overlays the power curve (mW below); hold to go back.
- **DYNAMIC LOAD**: manual load. Turning raises or lowers the current drawn from the panel;
  shows I, power and voltage. Max. 10 % of full scale and 3 W.

**SYSTEM**: update QR (OTA), reset and deep sleep (wakes with the knob).

## 3. Measuring from a phone

1. Join the Wi-Fi **ESP32_PLOT** (no password).
2. Open `http://192.168.4.1` and press **Start** (or **Stop**).
3. Shows the curve, Voc, Isc, Pmax, Vmp and Imp; **CSV** downloads the data; **Guide** opens the
   full guide. ES/EN switch and light/dark theme.

## 4. What a sweep does (10 to 20 s)

1. Measures the open-circuit voltage.
2. Finds the current range by itself (nothing to configure).
3. Takes up to **40 points** along the whole curve and stops at Isc.

\columnbreak

## 5. Range and precision

- Built for a **wide range**: up to about **26 V**, **780 mA** and **10 W** in the load during a sweep.
- **More current, better measurement.** With 40 to 70 mA or more (good light) the curve is
  smooth and accurate.
- **Small currents** (below about 20 mA): fewer points and a **1 to 2 mA** error.
- The load never turns fully off: it draws about **4 mA** even at rest. So the first point reads
  about 4 mA, and a dim panel's "no load" voltage reads lower than a multimeter on the bare
  panel. That is what is really being measured.

## 6. Tips

- Even, steady light; do not move or cover the panel during the sweep.
- To lower the current, shade the **whole** panel evenly (paper or cloth), never just a few
  cells.
- Lamp flicker is averaged out automatically.

## 7. Troubleshooting

- **"No panel detected", red LED**: panel unplugged or **too little light** (below 0.5 V).
  Light it better.
- **"Panel reversed", red LED**: swap PV+ and PV-.
- **Curve does not reach 0 V**: the panel exceeds the 780 mA cap.
- **Sweep stops early**: hit the 10 W limit.
- **Imprecise measurement**: current too small; more light.
- **Web page is empty**: join the ESP32_PLOT network again.

## 8. Safety and updates

- The load MOSFET heats up during a sweep: this is normal.
- No power switch: unplug the battery to turn it off.
- **Update**: download `app-standard.bin` from the latest GitHub release, join ESP32_PLOT, open
  `http://192.168.4.1/ota` (or the QR in **SYSTEM > OTA**) and upload it. It is a single file;
  it reboots on its own in about 30 s.

\begin{center}
\includegraphics[width=0.12\textwidth]{docs/img/guide_qr.png}\\
{\small Online guide}
\end{center}

\emc
