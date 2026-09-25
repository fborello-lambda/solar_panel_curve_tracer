(function () {
  // read theme from CSS variables so JS does not hardcode hex strings
  const css = getComputedStyle(document.documentElement);
  const ACCENT = css.getPropertyValue("--accent").trim() || "#ff9800";
  const FG = css.getPropertyValue("--fg").trim() || "#fff8ec";
  const MUTED = css.getPropertyValue("--muted").trim() || "#ffcc99";

  const infoEl = document.getElementById("info");
  const canvas = document.getElementById("chartCanvas");
  const isTouch = "ontouchstart" in window || navigator.maxTouchPoints > 0;

  if (!window.Chart) {
    document.body.insertAdjacentHTML(
      "beforeend",
      '<pre style="color:#f88">Chart.js not found</pre>'
    );
    return;
  }

  const ctx = canvas.getContext("2d");

  // create chart in outer scope so tick() can access it
  const chart = new Chart(ctx, {
    type: "line",
    data: {
      datasets: [
        {
          label: "I(V)",
          data: [],
          parsing: false,
          borderColor: ACCENT,
          pointBackgroundColor: ACCENT,
          pointBorderColor: ACCENT,
          pointRadius: isTouch ? 6 : 4,
          hoverRadius: isTouch ? 10 : 6,
          borderWidth: 2,
          tension: 0.12,
          yAxisID: "y", // current axis (left)
        },
        {
          label: "P(V)",
          data: [],
          parsing: false,
          borderColor: "red",
          backgroundColor: "rgba(255,0,0,0.12)",
          pointBackgroundColor: "red",
          pointBorderColor: "red",
          pointRadius: isTouch ? 4 : 2,
          hoverRadius: isTouch ? 8 : 4,
          borderWidth: 2,
          tension: 0.12,
          yAxisID: "p", // power axis (right)
        },
      ],
    },
    options: {
      responsive: true,
      maintainAspectRatio: false,
      animation: false,
      interaction: { mode: "nearest", axis: "xy", intersect: true },
      plugins: {
        legend: { display: true },
        tooltip: {
          enabled: true,
          backgroundColor: "rgba(0,0,0,0.8)",
          titleColor: FG,
          bodyColor: FG,
          callbacks: {
            title: (items) => {
              return "V: " + (items[0]?.raw?.x ?? "");
            },
            label: (ctx) => {
              if (ctx.dataset.label === "I(V)") {
                return "I: " + Number(ctx.raw.y).toFixed(3) + " mA";
              } else {
                return "P: " + Number(ctx.raw.y).toFixed(3) + " mW";
              }
            },
          },
        },
      },
      scales: {
        x: {
          type: "linear",
          beginAtZero: true,
          title: { display: true, text: "Voltage [V]", color: MUTED },
          ticks: { color: MUTED },
        },
        y: {
          position: "left",
          beginAtZero: true,
          title: { display: true, text: "Current [mA]", color: ACCENT },
          ticks: { color: ACCENT },
        },
        p: {
          id: "p",
          // power axis on the right
          position: "right",
          beginAtZero: true,
          title: { display: true, text: "Power [mW]", color: "red" },
          ticks: { color: "red" },
        },
      },
    },
  });

  // Used for start-measurement POST requests
  const START_MEASUREMENT_POST_ENDPOINT = "/start-measurement";
  const startMeasBtn = document.getElementById("startMeasBtn");
  const startMeasStatus = document.getElementById("startMeasStatus");

  function setMeasStatus(msg, ok = null) {
    if (ok === true) startMeasStatus.style.color = "#8f8";
    else if (ok === false) startMeasStatus.style.color = "#f88";
    else startMeasStatus.style.color = "";
    startMeasStatus.textContent = `Status: ${msg}`;
  }

  async function startMeas() {
    try {
      startMeasBtn.disabled = true;
      const resp = await fetch(START_MEASUREMENT_POST_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({}),
      });

      if (!resp.ok) {
        const msg = await resp.text().catch(() => resp.statusText);
        throw new Error(msg || "HTTP " + resp.status);
      }
      const payload = await resp.json().catch(() => ({}));
      if (typeof payload.running === "boolean") {
        if (payload.running) {
          setMeasStatus("Measurement started", true);
        } else {
          setMeasStatus("Measurement stopped", false);
        }
      } else if (payload.error) {
        throw new Error(payload.error);
      } else {
        setMeasStatus("Measurement status unknown", null);
      }
    } catch (e) {
      setMeasStatus("Error: " + (e.message || e), false);
    } finally {
      startMeasBtn.disabled = false;
    }
  }
  startMeasBtn.addEventListener("click", startMeas);

  // Summary display elements (Voc/Isc/Pmax/Vmp/Imp)
  const vocEl = document.getElementById("voc");
  const iscEl = document.getElementById("isc");
  const pmaxEl = document.getElementById("pmax");
  const mpptCurrentEl = document.getElementById("mpptCurrent");
  const mpptVoltageEl = document.getElementById("mpptVoltage");

  function refreshSummary(data) {
    if (!data || !data.length) {
      vocEl.textContent = "Voc: --";
      iscEl.textContent = "Isc: --";
      pmaxEl.textContent = "Pmax: --";
      mpptCurrentEl.textContent = "Imp: --";
      mpptVoltageEl.textContent = "Vmp: --";
      return;
    }

    // Simple approximations: Voc is the voltage of the lowest-current
    // point, Isc is the current of the lowest-voltage point.
    const byCurrent = [...data].sort((a, b) => a.y - b.y);
    const byVoltage = [...data].sort((a, b) => a.x - b.x);
    const voc = byCurrent[0];
    const isc = byVoltage[0];
    const mpp = [...data].sort((a, b) => b.x * b.y - a.x * a.y)[0];

    vocEl.textContent = `Voc: ${voc.x.toFixed(3)} V`;
    iscEl.textContent = `Isc: ${isc.y.toFixed(3)} mA`;
    pmaxEl.textContent = `Pmax: ${(mpp.x * mpp.y).toFixed(1)} mW`;
    mpptCurrentEl.textContent = `Imp: ${mpp.y.toFixed(3)} mA`;
    mpptVoltageEl.textContent = `Vmp: ${mpp.x.toFixed(3)} V`;
  }

  // polling state
  let currentCount = 0;
  const DEFAULT_POLL_MS = isTouch ? 800 : 400;
  let pollIntervalMs = DEFAULT_POLL_MS;

  async function tick() {
    try {
      const url = "/data?have=" + currentCount;
      const r = await fetch(url, { cache: "no-store" });
      const txt = await r.text();
      if (!txt) {
        throw new Error("empty response");
      }

      if (txt[0] === "[") {
        const arr = JSON.parse(txt);
        if (Array.isArray(arr)) {
          // replace dataset with server snapshot
          chart.data.datasets[0].data = arr;
          // power dataset (mW = V * mA)
          chart.data.datasets[1].data = arr.map((pt) => ({
            x: pt.x,
            y: pt.x * pt.y,
          }));
          infoEl.textContent = "points: " + arr.length;
          chart.update("none");

          // update currentCount to match server snapshot and reset poll interval
          currentCount = arr.length;
          pollIntervalMs = DEFAULT_POLL_MS;
        }
      } else {
        // small JSON like {"count":N}
        let small = {};
        try {
          small = JSON.parse(txt);
        } catch (e) {
          small = {};
        }
        const serverCount = Number.isFinite(small.count) ? small.count : 0;

        if (serverCount > currentCount) {
          console.warn(
            "serverCount > currentCount - incremental fetch not implemented"
          );
          // server has more points than we reported -> request full snapshot next tick
          currentCount = 0;
          pollIntervalMs = DEFAULT_POLL_MS;
        } else if (serverCount === currentCount && currentCount > 0) {
          console.log("data counts in sync");
          refreshSummary(chart.data.datasets[0].data);
          // gentle backoff when idle
          pollIntervalMs = Math.min(5000, pollIntervalMs + 200);
        } else {
          console.warn(
            "serverCount <= currentCount - data likely rolled/cleared"
          );
          // serverCount <= currentCount: server likely rolled/cleared -> force full fetch
          currentCount = 0;
          pollIntervalMs = DEFAULT_POLL_MS;
        }
      }
    } catch (e) {
      console.error("tick failed", e);
      // on error back off, but keep a cap
      pollIntervalMs = Math.min(5000, pollIntervalMs + 500);
    } finally {
      setTimeout(tick, pollIntervalMs);
    }
  }

  // start
  tick();

  // resize on orientation change
  window.addEventListener(
    "orientationchange",
    () => setTimeout(() => chart.resize(), 250),
    { passive: true }
  );
})();
