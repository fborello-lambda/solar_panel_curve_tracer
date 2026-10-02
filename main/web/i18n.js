(function (global) {
  "use strict";

  const LANG_KEY = "solar-lang";

  const dict = {
    en: {
      title: "Solar Panel I-V Curve",
      status_idle: "Idle",
      status_measuring: "Measuring",
      status_error: "Error",
      start_measurement: "Start measurement",
      stop_measurement: "Stop",
      download_csv: "Download CSV",
      guide: "Guide",
      stat_voc: "Voc",
      stat_isc: "Isc",
      stat_isc_vmin: "I @ Vmin",
      stat_pmax: "Pmax",
      stat_vmp: "Vmp",
      stat_imp: "Imp",
      stat_points: "Points",
      firmware_update: "Firmware update",
      toggle_theme: "Toggle dark mode",
      toggle_lang: "Switch language",
      chart_axis_v: "V [V]",
      chart_axis_i: "I [mA]",
      chart_axis_p: "P [mW]",
      legend_i: "I(V)",
      legend_p: "P(V)",
      legend_mpp: "MPP",
      tooltip_v: "V",
      tooltip_i: "I",
      tooltip_p: "P",
      tooltip_mpp: "MPP",
      err_chart_load: "Chart.js failed to load.",
      err_no_csv_data: "No data to export yet.",
      err_csv_failed: "CSV export failed: ",
      err_connection: "Connection issue, retrying...",
      err_start_stop: "Start/stop failed: ",
      err_reason_already_running: "already running",
      err_reason_dynamic_load_active: "dynamic load active",
      err_reason_sensor_not_ready: "sensor not ready",
      err_reason_unknown: "unknown error",
      note_sensor_not_detected: "Sensor not detected",
      note_no_panel: "No panel detected: check the PV connection and the light",
      note_no_load: "Load not responding: the current did not rise, check the load circuit",
      note_reversed: "Panel reversed or shorted: current flows at 0 V, swap PV+ and PV-",
      back_to_measurements: "Back to measurements",
      version_loading: "Loading version...",
      version_unavailable: "Version: unavailable",
      version_label: "Version",
      firmware_card_title: "Firmware (app-standard.bin)",
      firmware_hint: "Flashes the next OTA slot and reboots into it.",
      firmware_only_file: "The web UI is included in the firmware, so this is the only file you need.",
      firmware_download_note: "Download app-standard.bin from the latest GitHub release while online, then join the ESP32_PLOT Wi-Fi network (the device is offline on ESP32_PLOT):",
      firmware_secure_warn: "app-secure-boot.bin is not for standard boards. Do not upload it here.",
      upload_and_flash: "Upload and flash",
      reboot_note: "The device reboots automatically after a successful upload.",
      select_file_first: "Select a .bin file first.",
      err_bad_magic: "This does not look like a valid firmware image (bad header byte).",
      err_too_large: "File is too large for the OTA partition.",
      uploading: "Uploading {name} ({size} bytes)...",
      upload_done: "Done. Device is rebooting...",
      upload_error: "Error: ",
      upload_failed: "Upload failed or device rebooted.",
      guide_title: "Quick guide",
      guide_back: "Back",
      guide_what_h: "What it does",
      guide_what_p: "This device measures and traces the I-V curve (current vs. voltage) of a small solar panel, computing the maximum power point (MPP).",
      guide_safety_h: "Safety",
      guide_safety_1: "The load MOSFET heats up during a sweep: this is normal.",
      guide_safety_2: "The sweep aborts automatically if power reaches 10 W; the dynamic load is capped at 3 W.",
      guide_safety_3: "The load is capped at about 780 mA (20% of full scale).",
      guide_safety_4: "This prototype has no power switch: unplug the battery to turn it off.",
      guide_connect_h: "Connecting the panel",
      guide_connect_1: "Power on first with the panel unplugged: the current sensor zero is calibrated at boot.",
      guide_connect_2: "Then connect the panel: positive lead to PV+, negative lead to PV-.",
      guide_connect_3: "Green LED: measuring. Red LED: fault (no panel, reversed panel or load not responding).",
      guide_oled_h: "Measuring from the OLED",
      guide_oled_1: "Turn the encoder to MEASURE, press to select.",
      guide_oled_2: "Select CURVE TRACER, press to enter.",
      guide_oled_3: "Select START TRACE and press to begin the sweep.",
      guide_oled_4: "Turning navigates; pressing selects.",
      guide_oled_5: "CURVE CHART shows the last curve: turn to move the cursor, double press for the power curve, hold to go back. DYNAMIC LOAD is a manual load in 10 knob steps up to the last sweep's Isc: double press re-measures the range for whatever is connected, hold to go back (max. 3 W).",
      guide_phone_h: "Measuring from a phone or laptop",
      guide_phone_1: "Connect to Wi-Fi network ESP32_PLOT (no password).",
      guide_phone_2: "Open http://192.168.4.1 in a browser.",
      guide_phone_3: "Press Start / Stop. Switch language with ES / EN.",
      guide_sweep_h: "What happens during a sweep",
      guide_sweep_1: "Open-circuit voltage (Voc) is measured.",
      guide_sweep_2: "The current range is found automatically.",
      guide_sweep_3: "Up to 40 points are recorded, spread along the whole curve.",
      guide_sweep_4: "A full sweep takes about 10 to 20 seconds.",
      guide_read_h: "Reading the result",
      guide_read_1: "Voc: open-circuit voltage.",
      guide_read_2: "Isc: short-circuit current.",
      guide_read_3: "MPP: maximum power point (voltage and current).",
      guide_read_4: "Built for a wide range (up to about 26 V, 780 mA, 10 W): the more current, the better the measurement. Below about 20 mA expect fewer points and a 1 to 2 mA error.",
      guide_read_5: "The load always draws about 4 mA, so the first point shows about 4 mA and a dim panel's Voc reads lower than a multimeter on the bare panel.",
      guide_tips_h: "Tips",
      guide_tips_1: "Keep lighting steady during the sweep.",
      guide_tips_2: "Do not move or shade the panel while measuring. To lower the current, shade the whole panel evenly, never a few cells.",
      guide_tips_3: "Flickering lamps are averaged out automatically.",
      guide_trouble_h: "Troubleshooting",
      guide_trouble_1_p: "No panel detected, red LED",
      guide_trouble_1_c: "Panel unplugged or too little light (Voc < 0.5 V): light it better",
      guide_trouble_5_p: "Panel reversed, red LED",
      guide_trouble_5_c: "Swap PV+ and PV-",
      guide_trouble_2_p: "Curve doesn't reach 0 V",
      guide_trouble_2_c: "Panel is stronger than the 20% load cap",
      guide_trouble_3_p: "Sweep stops early",
      guide_trouble_3_c: "Hit the 10 W safety limit",
      guide_trouble_4_p: "Page is empty",
      guide_trouble_4_c: "Reconnect to the ESP32_PLOT Wi-Fi network",
      guide_ota_h: "Updating firmware",
      guide_ota_1: "Download app-standard.bin from the latest GitHub release while online.",
      guide_ota_2: "Join the ESP32_PLOT Wi-Fi network.",
      guide_ota_3: "Open /ota, upload the file, and wait about 30 seconds.",
      guide_sleep_h: "Deep sleep",
      guide_sleep_1: "SYSTEM > DEEP SLEEP. Wake up with a press of the encoder button.",
      guide_online_h: "Online guide",
      guide_chart_title: "Example I-V and P-V curve",
    },
    es: {
      title: "Curva I-V del panel solar",
      status_idle: "Inactivo",
      status_measuring: "Midiendo",
      status_error: "Error",
      start_measurement: "Iniciar medición",
      stop_measurement: "Detener",
      download_csv: "Descargar CSV",
      guide: "Guía",
      stat_voc: "Voc",
      stat_isc: "Isc",
      stat_isc_vmin: "I @ Vmin",
      stat_pmax: "Pmax",
      stat_vmp: "Vmp",
      stat_imp: "Imp",
      stat_points: "Puntos",
      firmware_update: "Actualizar firmware",
      toggle_theme: "Cambiar modo oscuro",
      toggle_lang: "Cambiar idioma",
      chart_axis_v: "V [V]",
      chart_axis_i: "I [mA]",
      chart_axis_p: "P [mW]",
      legend_i: "I(V)",
      legend_p: "P(V)",
      legend_mpp: "MPP",
      tooltip_v: "V",
      tooltip_i: "I",
      tooltip_p: "P",
      tooltip_mpp: "MPP",
      err_chart_load: "No se pudo cargar Chart.js.",
      err_no_csv_data: "Todavía no hay datos para exportar.",
      err_csv_failed: "Error al exportar CSV: ",
      err_connection: "Problema de conexión, reintentando...",
      err_start_stop: "Error al iniciar/detener: ",
      err_reason_already_running: "ya está en curso",
      err_reason_dynamic_load_active: "carga dinámica activa",
      err_reason_sensor_not_ready: "sensor no listo",
      err_reason_unknown: "error desconocido",
      note_sensor_not_detected: "Sensor no detectado",
      note_no_panel: "No se detecta panel: revise la conexión PV y la luz",
      note_no_load: "La carga no responde: la corriente no aumentó, revise el circuito de carga",
      note_reversed: "Panel invertido o en corto: circula corriente a 0 V, invierta PV+ y PV-",
      back_to_measurements: "Volver a mediciones",
      version_loading: "Cargando versión...",
      version_unavailable: "Versión: no disponible",
      version_label: "Versión",
      firmware_card_title: "Firmware (app-standard.bin)",
      firmware_hint: "Graba el próximo slot OTA y reinicia en él.",
      firmware_only_file: "La interfaz web viene incluida en el firmware, así que este es el único archivo necesario.",
      firmware_download_note: "Descargue app-standard.bin desde el último release de GitHub estando en línea, luego conéctese a la red Wi-Fi ESP32_PLOT (el dispositivo queda sin internet en ESP32_PLOT):",
      firmware_secure_warn: "app-secure-boot.bin no es para placas estándar. No lo suba aquí.",
      upload_and_flash: "Subir y grabar",
      reboot_note: "El dispositivo se reinicia automáticamente tras una carga exitosa.",
      select_file_first: "Seleccione primero un archivo .bin.",
      err_bad_magic: "Esto no parece una imagen de firmware válida (byte de cabecera incorrecto).",
      err_too_large: "El archivo es demasiado grande para la partición OTA.",
      uploading: "Subiendo {name} ({size} bytes)...",
      upload_done: "Listo. El dispositivo se está reiniciando...",
      upload_error: "Error: ",
      upload_failed: "Falló la carga o el dispositivo se reinició.",
      guide_title: "Guía rápida",
      guide_back: "Volver",
      guide_what_h: "Qué hace",
      guide_what_p: "Este dispositivo mide y traza la curva I-V (corriente vs. tensión) de un panel solar pequeño, calculando el punto de máxima potencia (MPP).",
      guide_safety_h: "Seguridad",
      guide_safety_1: "El MOSFET de carga se calienta durante el barrido: es normal.",
      guide_safety_2: "El barrido se detiene automáticamente si la potencia llega a 10 W; la carga manual se limita a 3 W.",
      guide_safety_3: "La carga está limitada a unos 780 mA (20% de la escala completa).",
      guide_safety_4: "Este prototipo no tiene interruptor de encendido: desconecte la batería para apagarlo.",
      guide_connect_h: "Conectar el panel",
      guide_connect_1: "Encienda primero con el panel desconectado: al arrancar se calibra el cero del sensor de corriente.",
      guide_connect_2: "Luego conecte el panel: positivo a PV+, negativo a PV-.",
      guide_connect_3: "LED verde: midiendo. LED rojo: falla (sin panel, panel invertido o carga sin respuesta).",
      guide_oled_h: "Medir desde la pantalla OLED",
      guide_oled_1: "Gire el encoder para ir a MEASURE, presione para entrar.",
      guide_oled_2: "Seleccione CURVE TRACER, presione para entrar.",
      guide_oled_3: "Seleccione START TRACE y presione para iniciar el barrido.",
      guide_oled_4: "Girar mueve la selección; presionar confirma.",
      guide_oled_5: "CURVE CHART muestra la última curva: girar mueve el cursor, doble pulsación superpone la potencia, mantener para volver. DYNAMIC LOAD es una carga manual en 10 pasos de la perilla hasta la Isc del último barrido: doble pulsación vuelve a medir el rango con lo que esté conectado, mantener para volver (máx. 3 W).",
      guide_phone_h: "Medir desde un teléfono o laptop",
      guide_phone_1: "Conéctese a la red Wi-Fi ESP32_PLOT (sin contraseña).",
      guide_phone_2: "Abra http://192.168.4.1 en el navegador.",
      guide_phone_3: "Presione Start / Stop. Cambie de idioma con ES / EN.",
      guide_sweep_h: "Qué sucede durante un barrido",
      guide_sweep_1: "Se mide la tensión de circuito abierto (Voc).",
      guide_sweep_2: "Se busca automáticamente el rango de corriente adecuado.",
      guide_sweep_3: "Se registran hasta 40 puntos, repartidos a lo largo de toda la curva.",
      guide_sweep_4: "El barrido completo toma entre 10 y 20 segundos.",
      guide_read_h: "Leer el resultado",
      guide_read_1: "Voc: tensión de circuito abierto.",
      guide_read_2: "Isc: corriente de cortocircuito.",
      guide_read_3: "MPP: punto de máxima potencia (tensión y corriente).",
      guide_read_4: "Diseñado para un rango amplio (hasta unos 26 V, 780 mA, 10 W): cuanta más corriente, mejor la medición. Por debajo de unos 20 mA hay menos puntos y un error de 1 a 2 mA.",
      guide_read_5: "La carga siempre toma unos 4 mA: el primer punto marca unos 4 mA y la Voc de un panel con poca luz sale menor que con un multímetro sobre el panel suelto.",
      guide_tips_h: "Consejos",
      guide_tips_1: "Mantenga la iluminación estable durante el barrido.",
      guide_tips_2: "No mueva ni tape el panel mientras mide. Para bajar la corriente, sombree todo el panel por igual, nunca solo algunas celdas.",
      guide_tips_3: "El parpadeo de lámparas se promedia automáticamente.",
      guide_trouble_h: "Solución de problemas",
      guide_trouble_1_p: "No se detecta panel, LED rojo",
      guide_trouble_1_c: "Panel desconectado o con muy poca luz (Voc < 0,5 V): ilumínelo mejor",
      guide_trouble_5_p: "Panel invertido, LED rojo",
      guide_trouble_5_c: "Intercambie PV+ y PV-",
      guide_trouble_2_p: "La curva no llega a 0 V",
      guide_trouble_2_c: "El panel supera el límite de carga del 20%",
      guide_trouble_3_p: "El barrido se detiene antes",
      guide_trouble_3_c: "Se alcanzó el límite de seguridad de 10 W",
      guide_trouble_4_p: "La página está vacía",
      guide_trouble_4_c: "Reconéctese a la red Wi-Fi ESP32_PLOT",
      guide_ota_h: "Actualizar el firmware",
      guide_ota_1: "Descargue app-standard.bin desde el último release de GitHub estando en línea.",
      guide_ota_2: "Conéctese a la red Wi-Fi ESP32_PLOT.",
      guide_ota_3: "Abra /ota, suba el archivo y espere unos 30 segundos.",
      guide_sleep_h: "Modo de bajo consumo",
      guide_sleep_1: "SYSTEM > DEEP SLEEP. Despierte presionando el botón del encoder.",
      guide_online_h: "Guía en línea",
      guide_chart_title: "Ejemplo de curva I-V y P-V",
    },
  };

  function detectDefault() {
    try {
      const nav = (global.navigator && global.navigator.language) || "en";
      return nav.toLowerCase().indexOf("es") === 0 ? "es" : "en";
    } catch (e) {
      return "en";
    }
  }

  function getStored() {
    try {
      return global.localStorage.getItem(LANG_KEY);
    } catch (e) {
      return null;
    }
  }

  function setStored(v) {
    try {
      global.localStorage.setItem(LANG_KEY, v);
    } catch (e) {
      // ignore
    }
  }

  let currentLang = getStored() || detectDefault();
  if (!dict[currentLang]) currentLang = "en";

  function t(key) {
    const d = dict[currentLang] || dict.en;
    if (Object.prototype.hasOwnProperty.call(d, key)) return d[key];
    return (dict.en && dict.en[key]) || key;
  }

  function setLang(lang) {
    if (!dict[lang]) return;
    currentLang = lang;
    setStored(lang);
    apply();
    try {
      global.dispatchEvent(new CustomEvent("i18n:change", { detail: { lang: currentLang } }));
    } catch (e) {
      // ignore (old browsers without CustomEvent constructor support)
    }
  }

  function getLang() {
    return currentLang;
  }

  function apply(root) {
    const scope = root || (global.document && global.document.body);
    if (!scope || !scope.querySelectorAll) return;
    scope.querySelectorAll("[data-i18n]").forEach(function (el) {
      const key = el.getAttribute("data-i18n");
      el.textContent = t(key);
    });
    scope.querySelectorAll("[data-i18n-aria]").forEach(function (el) {
      const key = el.getAttribute("data-i18n-aria");
      el.setAttribute("aria-label", t(key));
    });
    scope.querySelectorAll("[data-i18n-placeholder]").forEach(function (el) {
      const key = el.getAttribute("data-i18n-placeholder");
      el.setAttribute("placeholder", t(key));
    });
    if (global.document && global.document.documentElement) {
      global.document.documentElement.setAttribute("lang", currentLang);
    }
  }

  global.i18n = { t: t, setLang: setLang, getLang: getLang, apply: apply };
})(window);
