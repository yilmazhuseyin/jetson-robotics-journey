# Vision Gate — OCR-controlled twin-servo door (Jetson friendly)

Smooth, **butter-continuous** 120 Hz motion + **robust OCR** that reads small, skewed text.  
Built for Jetson/USB cams + PCA9685 servos.

## Demo
https://youtube.com/shorts/KqwLz06xrPo

## Why this works well
- **Low-latency camera**: frame-grabber thread keeps only the latest frame → no lag buildup. :contentReference[oaicite:1]{index=1}
- **Robust OCR**: progressive tesseract config, upscaling, threshold variants, optional `tesserocr` fast path. :contentReference[oaicite:2]{index=2}
- **Fuzzy allow-list**: accepts tiny OCR slips (edit-distance ≤ 1). :contentReference[oaicite:3]{index=3}
- **Silky motion**: dedicated servo thread at 100–120 Hz with cosine ease-in/out; camera/OCR never blocks motion. :contentReference[oaicite:4]{index=4}

## Hardware
- Jetson Orin Nano (tested) / any Linux machine
- USB camera (or CSI); plate printed on paper
- PCA9685 16-ch PWM
- 2x standard servos + 5–6V PSU (2–3A)
- Dupont wires, horn hardware

## Configure
Edit top of vision_gate.py:
- ALLOWED_PLATES = {"JETSON"} (add your tokens)
- PLATE_ROI (fractional box; make the paper fill most of it)
- LEFT_CLOSED, RIGHT_CLOSED, OPEN_SWEEP_DEG
- OPEN_DURATION_S, CLOSE_DURATION_S, HOLD_OPEN_SECONDS
- Optional OCR knobs: MIN_PLATE_LEN, UPSCALE_HEIGHT

## Troubleshooting
- Window is gray → check camera index, try CSI/USB pipeline; ensure no other app holds the cam.
- Buzzing at ends → reduce OPEN_SWEEP_DEG or narrow pulses MIN_US=1100, MAX_US=1900.
- Missed reads → move paper closer, increase UPSCALE_HEIGHT, or add more allowed strings.
