(function () {
  "use strict";

  const THEME_KEY = "solar-theme";

  function getStoredTheme() {
    try {
      return window.localStorage.getItem(THEME_KEY);
    } catch (e) {
      return null;
    }
  }

  function setStoredTheme(v) {
    try {
      window.localStorage.setItem(THEME_KEY, v);
    } catch (e) {
      // ignore, e.g. private browsing
    }
  }

  function systemPrefersDark() {
    try {
      return window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
    } catch (e) {
      return false;
    }
  }

  function isDarkActive() {
    const stored = getStoredTheme();
    if (stored === "dark") return true;
    if (stored === "light") return false;
    return systemPrefersDark();
  }

  function applyTheme() {
    const dark = isDarkActive();
    const stored = getStoredTheme();
    if (stored === "light") {
      document.documentElement.setAttribute("data-theme", "light");
    } else if (stored === "dark") {
      document.documentElement.setAttribute("data-theme", "dark");
    } else {
      document.documentElement.removeAttribute("data-theme");
    }
    document.body.classList.toggle("starfield", dark);
    return dark;
  }

  let currentDark = applyTheme();

  const themeToggle = document.getElementById("themeToggle");
  if (themeToggle) {
    themeToggle.addEventListener("click", function () {
      const next = isDarkActive() ? "light" : "dark";
      setStoredTheme(next);
      currentDark = applyTheme();
      if (typeof updateChartTheme === "function") updateChartTheme();
    });
  }

  try {
    const mq = window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)");
    if (mq && mq.addEventListener) {
      mq.addEventListener("change", function () {
        if (!getStoredTheme()) {
          currentDark = applyTheme();
          if (typeof updateChartTheme === "function") updateChartTheme();
        }
      });
    }
  } catch (e) {
    // ignore
  }

  const infoEl = document.getElementById("info");
  const canvas = document.getElementById("chartCanvas");

  if (!canvas) {
    return;
  }

  const i18n = window.i18n || { t: (k) => k, apply: () => {}, getLang: () => "en", setLang: () => {} };

  // Language toggle
  const langToggle = document.getElementById("langToggle");
  const langOpts = langToggle ? Array.from(langToggle.querySelectorAll(".lang-opt")) : [];
  function updateLangBtn() {
    const current = i18n.getLang();
    langOpts.forEach(function (btn) {
      btn.setAttribute("aria-pressed", btn.getAttribute("data-lang") === current ? "true" : "false");
    });
  }
  langOpts.forEach(function (btn) {
    btn.addEventListener("click", function () {
      i18n.setLang(btn.getAttribute("data-lang"));
    });
  });
  window.addEventListener("i18n:change", function () {
    updateLangBtn();
    if (typeof updateChartTheme === "function") updateChartTheme();
    if (chart) {
      chart.data.datasets[0].label = i18n.t("legend_i");
      chart.data.datasets[1].label = i18n.t("legend_p");
      chart.data.datasets[2].label = i18n.t("legend_mpp");
      chart.options.scales.x.title.text = i18n.t("chart_axis_v");
      chart.options.scales.y.title.text = i18n.t("chart_axis_i");
      chart.options.scales.p.title.text = i18n.t("chart_axis_p");
      chart.update("none");
    }
    updateStartBtn();
    setStatus(lastStatusState, lastStatusLabelKey ? i18n.t(lastStatusLabelKey) : statusText.textContent);
  });
  i18n.apply();
  updateLangBtn();

  const isTouch = "ontouchstart" in window || navigator.maxTouchPoints > 0;

  function showError(msg) {
    const banner = document.getElementById("errBanner");
    if (!banner) return;
    if (!msg) {
      banner.style.display = "none";
      banner.textContent = "";
      return;
    }
    banner.style.display = "block";
    banner.textContent = msg;
  }

  function readChartColors() {
    const css = getComputedStyle(document.documentElement);
    return {
      i: css.getPropertyValue("--chart-i").trim() || "#D97706",
      p: css.getPropertyValue("--chart-p").trim() || "#7C3AED",
      mpp: css.getPropertyValue("--chart-mpp").trim() || "#0891B2",
      grid: css.getPropertyValue("--chart-grid").trim() || "#e4e4e7",
      tick: css.getPropertyValue("--chart-tick").trim() || "#71717a",
      fg: css.getPropertyValue("--fg").trim() || "#171717",
    };
  }

  let chart = null;
  let updateChartTheme = null;

  if (!window.Chart) {
    showError(i18n.t("err_chart_load"));
  } else {
    const ctx = canvas.getContext("2d");
    const colors = readChartColors();

    chart = new window.Chart(ctx, {
      type: "line",
      data: {
        datasets: [
          {
            label: i18n.t("legend_i"),
            data: [],
            parsing: false,
            borderColor: colors.i,
            pointBackgroundColor: colors.i,
            pointBorderColor: colors.i,
            pointRadius: isTouch ? 5 : 3,
            hoverRadius: isTouch ? 9 : 6,
            borderWidth: 2,
            tension: 0.12,
            yAxisID: "y",
          },
          {
            label: i18n.t("legend_p"),
            data: [],
            parsing: false,
            borderColor: colors.p,
            backgroundColor: colors.p + "20",
            pointBackgroundColor: colors.p,
            pointBorderColor: colors.p,
            pointRadius: isTouch ? 4 : 2,
            hoverRadius: isTouch ? 8 : 5,
            borderWidth: 2,
            tension: 0.12,
            yAxisID: "p",
          },
          {
            label: i18n.t("legend_mpp"),
            data: [],
            parsing: false,
            showLine: false,
            pointStyle: "circle",
            backgroundColor: colors.mpp,
            borderColor: colors.mpp,
            pointRadius: isTouch ? 8 : 6,
            pointHoverRadius: isTouch ? 10 : 8,
            yAxisID: "p",
          },
        ],
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: { mode: "nearest", axis: "xy", intersect: true },
        plugins: {
          legend: { display: true, position: "top", labels: { color: colors.tick } },
          tooltip: {
            enabled: true,
            callbacks: {
              title: (items) => i18n.t("tooltip_v") + ": " + Number(items[0]?.raw?.x ?? 0).toFixed(3) + " V",
              label: (c) => {
                const raw = c.raw || {};
                if (c.dataset.label === i18n.t("legend_i")) {
                  return i18n.t("tooltip_i") + ": " + Number(raw.y).toFixed(3) + " mA";
                } else if (c.dataset.label === i18n.t("legend_p")) {
                  return i18n.t("tooltip_p") + ": " + Number(raw.y).toFixed(3) + " mW";
                } else if (c.dataset.label === i18n.t("legend_mpp")) {
                  const v = Number(raw.x || 0);
                  const p = Number(raw.y || 0);
                  return i18n.t("tooltip_mpp") + ": " + v.toFixed(3) + " V, " + p.toFixed(3) + " mW";
                }
                return c.formattedValue;
              },
            },
          },
        },
        scales: {
          x: {
            type: "linear",
            beginAtZero: true,
            title: { display: true, text: i18n.t("chart_axis_v"), color: colors.tick },
            ticks: { color: colors.tick },
            grid: { color: colors.grid },
          },
          y: {
            position: "left",
            beginAtZero: true,
            title: { display: true, text: i18n.t("chart_axis_i"), color: colors.i },
            ticks: { color: colors.i },
            grid: { color: colors.grid },
          },
          p: {
            position: "right",
            beginAtZero: true,
            title: { display: true, text: i18n.t("chart_axis_p"), color: colors.p },
            ticks: { color: colors.p },
            grid: { drawOnChartArea: false },
          },
        },
      },
    });

    updateChartTheme = function () {
      try {
        const c = readChartColors();
        chart.data.datasets[0].borderColor = c.i;
        chart.data.datasets[0].pointBackgroundColor = c.i;
        chart.data.datasets[0].pointBorderColor = c.i;
        chart.data.datasets[1].borderColor = c.p;
        chart.data.datasets[1].backgroundColor = c.p + "20";
        chart.data.datasets[1].pointBackgroundColor = c.p;
        chart.data.datasets[1].pointBorderColor = c.p;
        chart.data.datasets[2].backgroundColor = c.mpp;
        chart.data.datasets[2].borderColor = c.mpp;

        chart.options.plugins.legend.labels.color = c.tick;
        chart.options.scales.x.title.color = c.tick;
        chart.options.scales.x.ticks.color = c.tick;
        chart.options.scales.x.grid.color = c.grid;
        chart.options.scales.y.title.color = c.i;
        chart.options.scales.y.ticks.color = c.i;
        chart.options.scales.y.grid.color = c.grid;
        chart.options.scales.p.title.color = c.p;
        chart.options.scales.p.ticks.color = c.p;
        chart.update("none");
      } catch (e) {
        console.error("updateChartTheme failed", e);
      }
    };
  }

  // Status pill
  const statusPill = document.getElementById("statusPill");
  const statusText = document.getElementById("statusText");
  const sensorNote = document.getElementById("sensorNote");
  let lastStatusState = "idle";
  let lastStatusLabelKey = "status_idle";

  function setStatus(state, label) {
    lastStatusState = state;
    if (statusPill) statusPill.setAttribute("data-state", state);
    if (statusText) statusText.textContent = label;
  }

  const REASON_KEY_BY_TEXT = {
    "already running": "err_reason_already_running",
    "dynamic load active": "err_reason_dynamic_load_active",
    "sensor not ready": "err_reason_sensor_not_ready",
  };
  function reasonText(raw) {
    const key = REASON_KEY_BY_TEXT[raw];
    return key ? i18n.t(key) : raw || i18n.t("err_reason_unknown");
  }

  // Start/Stop measurement, driven by server state via /status
  const startMeasBtn = document.getElementById("startMeasBtn");
  let measuring = false;

  function updateStartBtn() {
    if (!startMeasBtn) return;
    startMeasBtn.textContent = measuring ? i18n.t("stop_measurement") : i18n.t("start_measurement");
  }

  async function startMeas() {
    if (!startMeasBtn) return;
    const endpoint = measuring ? "/measurement/stop" : "/measurement/start";
    try {
      startMeasBtn.disabled = true;
      const resp = await fetch(endpoint, { method: "POST" });
      const payload = await resp.json().catch(() => ({}));
      if (!resp.ok) {
        lastStatusLabelKey = null;
        setStatus("error", i18n.t("status_error"));
        showError(i18n.t("err_start_stop") + reasonText(payload.error));
        return;
      }
      if (typeof payload.running === "boolean") {
        measuring = payload.running;
        lastStatusLabelKey = measuring ? "status_measuring" : "status_idle";
        setStatus(measuring ? "measuring" : "idle", i18n.t(lastStatusLabelKey));
        showError(null);
      }
    } catch (e) {
      lastStatusLabelKey = null;
      setStatus("error", i18n.t("status_error"));
      showError(i18n.t("err_start_stop") + (e.message || e));
    } finally {
      startMeasBtn.disabled = false;
      updateStartBtn();
      pollStatus();
    }
  }
  if (startMeasBtn) {
    startMeasBtn.addEventListener("click", startMeas);
  }

  // /status polling: every ~2s, keeps the pill and start/stop button in
  // sync with what the firmware is actually doing.
  async function pollStatus() {
    try {
      const r = await fetch("/status", { cache: "no-store" });
      if (!r.ok) return;
      const s = await r.json();
      measuring = !!s.running;
      lastStatusLabelKey = measuring ? "status_measuring" : "status_idle";
      setStatus(measuring ? "measuring" : "idle", i18n.t(lastStatusLabelKey));
      updateStartBtn();
      if (sensorNote) {
        const showNote = s.mode === "REAL" && s.ina_ready === false;
        sensorNote.style.display = showNote ? "flex" : "none";
      }
    } catch (e) {
      // leave last known state; the /data poller already surfaces connection errors
    }
  }
  pollStatus();
  setInterval(pollStatus, 2000);

  // Summary elements
  const vocEl = document.getElementById("voc");
  const iscEl = document.getElementById("isc");
  const pmaxEl = document.getElementById("pmax");
  const mpptCurrentEl = document.getElementById("mpptCurrent");
  const mpptVoltageEl = document.getElementById("mpptVoltage");
  const iscLabelEl = document.getElementById("iscLabel");

  let lastMpp = null;

  function refreshSummary(data) {
    if (!data || !data.length) {
      if (vocEl) vocEl.textContent = "-- V";
      if (iscEl) iscEl.textContent = "-- mA";
      if (pmaxEl) pmaxEl.textContent = "-- mW";
      if (mpptCurrentEl) mpptCurrentEl.textContent = "-- mA";
      if (mpptVoltageEl) mpptVoltageEl.textContent = "-- V";
      if (iscLabelEl) iscLabelEl.setAttribute("data-i18n", "stat_isc");
      i18n.apply();
      lastMpp = null;
      if (chart) chart.data.datasets[2].data = [];
      return;
    }

    const byCurrent = [...data].sort((a, b) => a.y - b.y);
    const byVoltage = [...data].sort((a, b) => a.x - b.x);
    const voc = byCurrent[0];
    const isc = byVoltage[0];
    const mpp = [...data].sort((a, b) => b.x * b.y - a.x * a.y)[0];

    if (vocEl) vocEl.textContent = `${voc.x.toFixed(3)} V`;
    if (iscEl) iscEl.textContent = `${isc.y.toFixed(3)} mA`;
    if (pmaxEl) pmaxEl.textContent = `${(mpp.x * mpp.y).toFixed(1)} mW`;
    if (mpptCurrentEl) mpptCurrentEl.textContent = `${mpp.y.toFixed(3)} mA`;
    if (mpptVoltageEl) mpptVoltageEl.textContent = `${mpp.x.toFixed(3)} V`;

    // When the lowest recorded voltage is well above zero (>5% of Voc), this
    // isn't a true short-circuit current: label it as the current at the
    // lowest swept voltage instead of Isc.
    if (iscLabelEl) {
      const isRealIsc = voc.x <= 0 || isc.x <= 0.05 * voc.x;
      iscLabelEl.setAttribute("data-i18n", isRealIsc ? "stat_isc" : "stat_isc_vmin");
    }
    i18n.apply();

    lastMpp = { x: mpp.x, y: mpp.x * mpp.y };
    if (chart) chart.data.datasets[2].data = [lastMpp];
  }

  // polling state
  let currentCount = 0;
  const DEFAULT_POLL_MS = isTouch ? 800 : 400;
  const IDLE_POLL_MS = isTouch ? 5000 : 3000;
  let pollIntervalMs = DEFAULT_POLL_MS;

  async function tick() {
    try {
      const url = "/data?have=" + currentCount;
      const r = await fetch(url, { cache: "no-store" });
      if (!r.ok) {
        throw new Error("HTTP " + r.status);
      }
      const txt = await r.text();
      if (!txt) {
        throw new Error("empty response");
      }

      if (txt[0] === "[") {
        const arr = JSON.parse(txt);
        if (Array.isArray(arr)) {
          if (chart) {
            chart.data.datasets[0].data = arr;
            chart.data.datasets[1].data = arr.map((pt) => ({
              x: pt.x,
              y: pt.x * pt.y,
            }));
          }
          if (infoEl) infoEl.textContent = String(arr.length);
          refreshSummary(arr);
          if (chart) chart.update("none");

          currentCount = arr.length;
          pollIntervalMs = DEFAULT_POLL_MS;
          showError(null);
        }
      } else {
        let small = {};
        try {
          small = JSON.parse(txt);
        } catch (e) {
          small = {};
        }
        const serverCount = Number.isFinite(small.count) ? small.count : 0;

        if (serverCount === 0 && currentCount === 0) {
          // Nothing has ever been measured: back off quietly, no warnings.
          pollIntervalMs = IDLE_POLL_MS;
        } else if (serverCount > currentCount) {
          currentCount = 0;
          pollIntervalMs = DEFAULT_POLL_MS;
        } else if (serverCount === currentCount && currentCount > 0) {
          pollIntervalMs = Math.min(5000, pollIntervalMs + 200);
        } else {
          currentCount = 0;
          pollIntervalMs = DEFAULT_POLL_MS;
        }
        showError(null);
      }
    } catch (e) {
      console.error("tick failed", e);
      pollIntervalMs = Math.min(5000, pollIntervalMs + 500);
      showError(i18n.t("err_connection"));
    } finally {
      setTimeout(tick, pollIntervalMs);
    }
  }

  tick();

  // Download CSV
  const downloadBtn = document.getElementById("downloadCsvBtn");
  if (downloadBtn) {
    downloadBtn.addEventListener("click", function () {
      try {
        const points = (chart && chart.data.datasets[0].data) || [];
        if (!points.length) {
          showError(i18n.t("err_no_csv_data"));
          return;
        }
        let csv = "V,I_mA,P_mW\n";
        for (const pt of points) {
          const v = Number(pt.x);
          const i = Number(pt.y);
          csv += `${v},${i},${(v * i).toFixed(6)}\n`;
        }
        const blob = new Blob([csv], { type: "text/csv" });
        const a = document.createElement("a");
        const ts = new Date().toISOString().replace(/[:.]/g, "-");
        a.href = URL.createObjectURL(blob);
        a.download = `iv_curve_${ts}.csv`;
        document.body.appendChild(a);
        a.click();
        a.remove();
        setTimeout(() => URL.revokeObjectURL(a.href), 1000);
      } catch (e) {
        showError(i18n.t("err_csv_failed") + (e.message || e));
      }
    });
  }

  // Firmware version in footer
  const fwVersionEl = document.getElementById("fwVersion");
  if (fwVersionEl) {
    fetch("/version")
      .then((r) => r.json())
      .then((d) => {
        if (d && d.version) fwVersionEl.textContent = "v" + d.version;
      })
      .catch(() => {
        // leave default placeholder
      });
  }

  // resize on orientation change / viewport resize
  window.addEventListener(
    "orientationchange",
    () => setTimeout(() => { if (chart) chart.resize(); }, 250),
    { passive: true }
  );
  window.addEventListener(
    "resize",
    () => { if (chart) chart.resize(); },
    { passive: true }
  );
})();
