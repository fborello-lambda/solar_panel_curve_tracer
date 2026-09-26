#!/usr/bin/env python3
"""Regenerate docs/img/guide_qr.png: a QR code linking to the online quick guide.

Usage: python3 docs/img/make_guide_qr.py
Requires the `segno` package (pip install segno).
"""
import os

import segno

URL = "https://github.com/fborello-lambda/solar_panel_curve_tracer/blob/main/docs/quick_guide.md"

if __name__ == "__main__":
    out_path = os.path.join(os.path.dirname(__file__), "guide_qr.png")
    qr = segno.make(URL, error="m")
    qr.save(out_path, scale=8, border=2)
    print(f"wrote {out_path} for {URL}")
