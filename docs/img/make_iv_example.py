#!/usr/bin/env python3
"""Generates docs/img/iv_example.png: a labelled I-V and P-V curve for a
representative ~50 mA panel, with Voc, Isc, and the MPP marked. Language
neutral (axis labels only, no prose) so the same image is reused on both
the Spanish and English pages of docs/quick_guide.md.

Run manually with:  python3 docs/img/make_iv_example.py
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

VOC = 21.5     # V
ISC = 50.0     # mA
N = 1.5        # diode ideality-ish shape factor

v = np.linspace(0, VOC, 400)
# Simple single-diode-like shape: flat current source, sharp knee near Voc.
i = ISC * (1.0 - np.exp((v - VOC) / (VOC / (18 * N))))
i = np.clip(i, 0, None)
p = v * i

mpp_idx = int(np.argmax(p))
v_mpp, i_mpp, p_mpp = v[mpp_idx], i[mpp_idx], p[mpp_idx]

fig, ax1 = plt.subplots(figsize=(5.4, 2.6), dpi=200)
ax2 = ax1.twinx()

color_i = "#1f6f43"
color_p = "#7a4fae"

ax1.plot(v, i, color=color_i, linewidth=1.8, label="I")
ax2.plot(v, p, color=color_p, linewidth=1.8, linestyle="--", label="P")

ax1.set_xlabel("V [V]", fontsize=8)
ax1.set_ylabel("I [mA]", color=color_i, fontsize=8)
ax2.set_ylabel("P [mW]", color=color_p, fontsize=8)
ax1.tick_params(axis="y", labelcolor=color_i, labelsize=7)
ax2.tick_params(axis="y", labelcolor=color_p, labelsize=7)
ax1.tick_params(axis="x", labelsize=7)

ax1.set_xlim(0, VOC * 1.05)
ax1.set_ylim(0, ISC * 1.15)
ax2.set_ylim(0, p_mpp * 1.35)

# Mark Voc, Isc, MPP.
ax1.axvline(VOC, color="#999999", linewidth=0.6, linestyle=":")
ax1.scatter([VOC], [0], color="#333333", s=14, zorder=5)
ax1.annotate("Voc", xy=(VOC, 0), xytext=(VOC - 3.6, ISC * 0.06),
             fontsize=7, color="#333333")

ax1.scatter([0], [ISC], color="#333333", s=14, zorder=5)
ax1.annotate("Isc", xy=(0, ISC), xytext=(0.6, ISC * 0.97),
             fontsize=7, color="#333333")

ax1.scatter([v_mpp], [i_mpp], color="#c0392b", s=18, zorder=6)
ax1.annotate("MPP", xy=(v_mpp, i_mpp), xytext=(v_mpp + 0.8, i_mpp + ISC * 0.12),
             fontsize=7, color="#c0392b")

lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc="lower left",
           fontsize=7, frameon=False)

fig.tight_layout(pad=0.4)
fig.savefig("docs/img/iv_example.png", dpi=200)
print("wrote docs/img/iv_example.png")
