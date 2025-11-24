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
                // show current with units depending on mode
                return (
                  "I: " +
                  Number(ctx.raw.y).toFixed(3) +
                  (unitIsMilli ? " mA" : " A")
                );
              } else {
                // power in matching units
                return (
                  "P: " +
                  Number(ctx.raw.y).toFixed(3) +
                  (unitIsMilli ? " mW" : " W")
                );
              }
            },
          },
        },
      },
      scales: {
        x: {
          type: "linear",
          min: 0,
          max: 25,
          title: { display: true, text: "Voltage [V]", color: MUTED },
          ticks: { color: MUTED },
        },
        y: {
          position: "left",
          min: 0,
          max: 10,
          title: { display: true, text: "Current [mA]", color: ACCENT },
          ticks: { color: ACCENT },
        },
        p: {
          id: "p",
          // power axis on the right
          position: "right",
          title: { display: true, text: "Power [mW]", color: "red" },
          ticks: { color: "red" },
        },
      },
    },
  });

  const autoBtn = document.getElementById("autoBtn");
  const autoPowerBtn = document.getElementById("autoPowerBtn");

  // Used for set-current POST requests
  const SET_CURRENT_POST_ENDPOINT = "/set-current";
  const setI = document.getElementById("setI");
  const setIUnit = document.getElementById("setIUnit");
  const sendIBtn = document.getElementById("sendI");
  const sendIStatus = document.getElementById("sendIStatus");
  const currentRange = document.getElementById("currentRange");

  async function refreshCurrentRange() {
    try {
      const r = await fetch("/current", { cache: "no-store" });
      if (!r.ok) return;
      const j = await r.json();
      currentRange.textContent = `Range: ${j.current_mA.toFixed(3)} mA`;
    } catch (_) {}
  }
  refreshCurrentRange();
  setInterval(refreshCurrentRange, 5000);

  function setCurrentStatus(msg, ok = null) {
    sendIStatus.textContent = msg;
    if (ok === true) sendIStatus.style.color = "#8f8";
    else if (ok === false) sendIStatus.style.color = "#f88";
    else sendIStatus.style.color = "";
  }

  async function sendCurrent() {
    const raw = parseFloat(setI.value);
    if (!Number.isFinite(raw)) {
      setCurrentStatus("Enter a number", false);
      return;
    }
    let value_mA = setIUnit.value === "A" ? raw * 1000.0 : raw;

    try {
      sendIBtn.disabled = true;
      setCurrentStatus("Sending...");
      let resp;
      resp = await fetch(SET_CURRENT_POST_ENDPOINT, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ current_mA: value_mA }),
      });

      if (!resp.ok) {
        const msg = await resp.text().catch(() => resp.statusText);
        throw new Error(msg || "HTTP " + resp.status);
      }
      value_resp_mA = await resp
        .text()
        .catch(() => resp.json().then((j) => j.current_mA));
      setCurrentStatus(`Ok: ${value_mA} mA`, true);
      currentRange.textContent = `Range: ${value_mA.toFixed(3)} mA`;
    } catch (e) {
      setCurrentStatus("Error: " + (e.message || e), false);
    } finally {
      sendIBtn.disabled = false;
    }
  }
  sendIBtn.addEventListener("click", sendCurrent);
  setI.addEventListener("keydown", (ev) => {
    if (ev.key === "Enter") sendCurrent();
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

  // true = chart data in mA, false = in A
  // The server always sends mA, so we start in mA mode
  let unitIsMilli = true;
  const unitBtn = document.getElementById("unitBtn");

  function autoScale() {
    // remove fixed limits so Chart.js autoscale uses data
    delete chart.options.scales.x.min;
    delete chart.options.scales.x.max;
    delete chart.options.scales.y.min;
    delete chart.options.scales.y.max;
    delete chart.options.scales.p.min;
    delete chart.options.scales.p.max;
    chart.update("none");
  }
  autoBtn.addEventListener("click", autoScale);

  function autoScalePower() {
    // autoscale only the power axis so both can be fullscreen
    delete chart.options.scales.p.min;
    delete chart.options.scales.p.max;
    chart.update("none");
  }
  autoPowerBtn.addEventListener("click", autoScalePower);

  function toggleUnits() {
    // convert current dataset values to the other unit
    const data = chart.data.datasets[0].data;
    const oldIsMilli = unitIsMilli;
    const newIsMilli = !oldIsMilli;

    if (data && data.length) {
      for (let i = 0; i < data.length; i++) {
        // oldIsMilli == true means current values are in mA -> convert to A
        if (oldIsMilli) {
          data[i] = { x: data[i].x, y: data[i].y / 1000.0 };
        } else {
          data[i] = { x: data[i].x, y: data[i].y * 1000.0 };
        }
      }
    }

    // recompute power dataset based on new units:
    // - if newIsMilli === true -> current in mA, want power in mW: P[mW] = V * I[mA]
    // - if newIsMilli === false -> current in A, want power in W: P[W] = V * I[A]
    const cur = chart.data.datasets[0].data || [];
    chart.data.datasets[1].data = cur.map((pt) => {
      if (newIsMilli) {
        return { x: pt.x, y: pt.x * pt.y }; // mW
      } else {
        return { x: pt.x, y: pt.x * pt.y }; // W
      }
    });

    // Update axis label & convert fixed limits if present
    if (newIsMilli) {
      // switching to milliamperes
      chart.options.scales.y.title.text = "Current [mA]";
      // convert power axis limits from W->mW if they exist
      if (typeof chart.options.scales.p.min === "number")
        chart.options.scales.p.min *= 1000.0;
      if (typeof chart.options.scales.p.max === "number")
        chart.options.scales.p.max *= 1000.0;
      chart.options.scales.p.title.text = "Power [mW]";
    } else {
      // switching to Amperes
      chart.options.scales.y.title.text = "Current [A]";
      // convert power axis limits from mW->W if they exist
      if (typeof chart.options.scales.p.min === "number")
        chart.options.scales.p.min /= 1000.0;
      if (typeof chart.options.scales.p.max === "number")
        chart.options.scales.p.max /= 1000.0;
      chart.options.scales.p.title.text = "Power [W]";
    }

    unitIsMilli = newIsMilli;
    chart.update("none");
  }
  unitBtn.addEventListener("click", toggleUnits);

  function maybeConvertIncoming(arr) {
    if (!unitIsMilli) {
      // Current chart is displaying Amperes; server sends mA -> convert
      for (let i = 0; i < arr.length; i++) {
        arr[i].y = arr[i].y / 1000.0;
      }
    }
  }

  // MPPT display elements
  const mpptCurrentEl = document.getElementById("mpptCurrent");
  const mpptVoltageEl = document.getElementById("mpptVoltage");

  function refreshMPPT(data) {
    if (!data || !data.length) {
      mpptCurrentEl.textContent = "MPPT Current: --";
      mpptVoltageEl.textContent = "MPPT Voltage: --";
      return;
    }

    // I'm passing a const array to sort() and it even mutates the array... JS :p
    // An explicit copy is needed.
    const best = [...data].sort((a, b) => b.x * b.y - a.x * a.y)[0];

    const currentDisplay = unitIsMilli
      ? `${best.y.toFixed(3)} mA`
      : `${best.y.toFixed(3)} A`;
    mpptCurrentEl.textContent = `MPPT Current: ${currentDisplay}`;
    mpptVoltageEl.textContent = `MPPT Voltage: ${best.x.toFixed(3)} V`;
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
          maybeConvertIncoming(arr);
          // replace dataset with server snapshot
          chart.data.datasets[0].data = arr;
          // power dataset
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
          // in sync -> autoscale both datasets (current and power)
          autoScale(); // autoscale current and reset power as well
          autoScalePower(); // autoscale power axis independently
          refreshMPPT(chart.data.datasets[0].data);
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
