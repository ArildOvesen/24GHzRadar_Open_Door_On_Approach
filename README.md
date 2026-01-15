\# 24GHzRadar\_Open\_Door



ESP32-S2 (LOLIN S2 Mini) + HLK-LD2410C 24 GHz radar project that opens a door when a person approaches.



This project is designed for \*\*real physical environments\*\* (reflections, occlusions, “presence but no stable distance”, varying Wi-Fi latency), and focuses on \*\*robust behavior\*\* rather than fancy algorithms.



\## What it does



\- Reads presence + distance from an \*\*HLK-LD2410C\*\* radar sensor over UART

\- Detects “approaching the door” using a simple \*\*state machine\*\*

&nbsp; - Early tracking (build history before the critical zone)

&nbsp; - Fast-pass gate based on distance + velocity consistency (2 of last 3 samples)

&nbsp; - Hysteresis + “wait clear” re-arm to avoid re-triggers

\- Sends an HTTPS request to an external backend to open the door

\- Optional: sends periodic debug snapshots to a Home Assistant webhook (headless monitoring)



\## Hardware



\- ESP32-S2: LOLIN S2 Mini

\- HLK-LD2410C 24 GHz radar

\- UART wiring:

&nbsp; - Radar TX → ESP32 GPIO33

&nbsp; - Radar RX → ESP32 GPIO18

&nbsp; - VBUS → 5V



\## Software



\- PlatformIO (Arduino framework)

\- Wi-Fi: open network supported (SSID configured in `secrets.h`)

\- HTTPS: configurable insecure mode for development environments



\## Configuration (required)



This repository intentionally \*\*does not\*\* commit secrets.



1\. Copy the template:

&nbsp;  - `include/secrets.example.h` → `include/secrets.h`

2\. Edit `include/secrets.h` and set:

&nbsp;  - `WIFI\_SSID`

&nbsp;  - `HOST`

&nbsp;  - `OPEN\_URL`

&nbsp;  - `DEBUG\_WEBHOOK\_URL` (optional)

&nbsp;  - `HTTPS\_INSECURE`, `DEBUG\_WEBHOOK\_INSECURE`



> `include/secrets.h` is ignored by Git via `.gitignore`.



\## Build \& Upload



Using PlatformIO:



\- Build: `pio run`

\- Upload: `pio run -t upload`

\- Monitor (optional): `pio device monitor`



\## Notes on latency



In many deployments the end-to-end “open” action is dominated by network + backend latency.

This project can be tuned to trigger earlier (larger pre-enter distance) while keeping false triggers low.



\## License



This project is licensed under the \*\*GNU General Public License v3.0 (GPL-3.0)\*\*. See `LICENSE`.



"# 24GHzRadar_Open_Door" 
