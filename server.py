#!/usr/bin/env python3
"""
O'CELL BMS CAN Dashboard — Backend Server
==========================================
Reads ESP32 serial output, decodes O'CELL 19S1P CAN frames,
streams live JSON to the browser via WebSocket.

Battery: IFS60.8-500-F-E3  19S1P LiFePO4  60.8V / 50Ah
CAN baud: 250 kbps

Requirements:  pip install fastapi uvicorn pyserial

Usage:
    python server.py                      # auto-detect USB port
    python server.py --port COM3
    python server.py --port /dev/ttyUSB0
    python server.py --baud 115200        # default
    python server.py --web-port 8765      # default

─────────────────────────────────────────────────────────────────
Extra Sensor Serial Protocol (ESP32 → PC)
─────────────────────────────────────────
Alongside CAN frames the ESP32 sends simple key=value lines:

  SENSOR:hv_iso_v=58.34        MOD1150-3  isolation module voltage (V)
  SENSOR:aux12v=12.41          C11A063    12 V auxiliary bus voltage (V)
  SENSOR:mppt_i1=3.12          ACS712 #1  MPPT current channel 1 (A)
  SENSOR:mppt_i2=2.87          ACS712 #2  MPPT current channel 2 (A)
  SENSOR:bat_t1=28.5           DS18B20 #1 battery surface temp (°C)
  SENSOR:bat_t2=29.1           DS18B20 #2 battery surface temp (°C)
  SENSOR:mppt_t=42.3           NTC #1     MPPT heatsink temp (°C)
  SENSOR:dcdc_t=38.7           NTC #2     DC-DC heatsink temp (°C)
  SENSOR:handbrake=1           LJ12A3     handbrake engaged(1)/released(0)
  SENSOR:oil_level=75          (TBD)      oil level 0-100 %
─────────────────────────────────────────────────────────────────
"""

import re, sys, time, asyncio, argparse, threading
from collections import deque
from datetime import datetime
from typing import Optional

try:
    import serial, serial.tools.list_ports
except ImportError:
    print("ERROR: pip install pyserial"); sys.exit(1)

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
except ImportError:
    print("ERROR: pip install fastapi uvicorn"); sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# O'CELL Frame Decoder
# ─────────────────────────────────────────────────────────────────────────────

FRAME_RE = re.compile(
    r'\[(\d+)ms\]\s+ID:\s+(0x[0-9A-Fa-f]+)\s+DLC:\s+(\d+)\s+Data:\s+([0-9A-Fa-f\s]+)'
)

# SENSOR:key=value  — extra GPIO/ADC sensors from ESP32
SENSOR_RE = re.compile(r'^SENSOR:(\w+)=(.+)$')

def parse_line(line):
    m = FRAME_RE.search(line)
    if not m: return None
    return (int(m.group(1)), int(m.group(2), 16), int(m.group(3)),
            bytes(int(b, 16) for b in m.group(4).split()))

def u16be(d, o): return (d[o] << 8) | d[o+1]
def u16le(d, o): return d[o] | (d[o+1] << 8)


class BMSState:
    # ── CAN / BMS constants ──────────────────────────────────────────────────
    CELL_OV       = 3750
    CELL_UV       = 2500
    CELL_FULL     = 3650
    CELL_NOMINAL  = 3200
    CELL_BAL_THR  = 3300
    CELL_BAL_DELT = 20
    PACK_FULL_V   = 69.35
    PACK_EMPTY_V  = 47.50
    PACK_NOM_V    = 60.80
    MAX_CHARGE_A  = 25.0
    MAX_DISCH_A   = 50.0
    NUM_CELLS     = 19

    # ── Extra-sensor limits (for UI colour coding) ───────────────────────────
    HV_ISO_NOM    = 60.80   # V  — nominal HV isolation sense voltage
    AUX12_NOM     = 12.0    # V  — target 12 V aux
    AUX12_LOW     = 11.5    # V  — low warning
    AUX12_HIGH    = 14.5    # V  — high warning
    MPPT_MAX_A    = 5.0     # A  — ACS712-5A full-scale
    BAT_SURF_WARN = 40.0    # °C — battery surface warn
    BAT_SURF_ERR  = 55.0    # °C — battery surface error
    HS_WARN       = 60.0    # °C — heatsink warning
    HS_ERR        = 80.0    # °C — heatsink error

    def __init__(self):
        self.lock         = threading.Lock()

        # ── CAN / BMS state ──────────────────────────────────────────────────
        self.cell_mv      = {}
        self.temp_c       = {}
        self.pack_v       = None
        self.soc          = None
        self.soc_ffe5     = None
        self.chg_i_req    = None
        self.disch_i_lim  = None
        self.cell_max_mv  = None
        self.cell_min_mv  = None

        # ── Extra sensor state ────────────────────────────────────────────────
        self.hv_iso_v     = None   # MOD1150-3  isolation module HV voltage (V)
        self.aux12v       = None   # C11A063    12 V auxiliary bus (V)
        self.mppt_i1      = None   # ACS712 #1  MPPT current ch1 (A)
        self.mppt_i2      = None   # ACS712 #2  MPPT current ch2 (A)
        self.bat_t1       = None   # DS18B20 #1 battery surface temp (°C)
        self.bat_t2       = None   # DS18B20 #2 battery surface temp (°C)
        self.mppt_t       = None   # NTC #1  MPPT heatsink temp (°C)
        self.dcdc_t       = None   # NTC #2  DC-DC heatsink temp (°C)
        self.handbrake    = None   # 1 = engaged, 0 = released
        self.oil_level    = None   # 0-100 %

        # ── Housekeeping ──────────────────────────────────────────────────────
        self.frame_count  = 0
        self.connected    = False
        self.port_name    = ""
        self.last_update  = None
        self.raw_log      = deque(maxlen=300)

    # ── CAN frame decoder ─────────────────────────────────────────────────────
    def decode(self, ts, can_id, dlc, data):
        func = (can_id >> 16) & 0xFF
        sub  = (can_id >>  8) & 0xFF

        with self.lock:
            self.frame_count += 1
            self.last_update  = datetime.now()

            if 0xC8 <= func <= 0xCC and dlc == 8:
                group = func - 0xC8
                base  = group * 4 + 1
                for i in range(4):
                    o = i * 2
                    if o + 1 < len(data):
                        v = u16be(data, o)
                        if v != 0:
                            self.cell_mv[base + i] = v

            elif func == 0xB4 and dlc == 8:
                for i in range(3):
                    raw = data[i]
                    if raw != 0:
                        self.temp_c[i + 1] = float(raw) - 40.0

            elif func == 0xFF and sub == 0xE5 and dlc == 8 and len(data) >= 4:
                self.soc_ffe5  = u16le(data, 0) / 10.0
                self.chg_i_req = u16le(data, 2) / 10.0

            elif func == 0xFF and sub == 0x28 and dlc == 8 and len(data) >= 6:
                self.pack_v      = u16le(data, 0) / 100.0
                self.disch_i_lim = u16le(data, 2) / 100.0
                self.soc         = u16le(data, 4) / 10.0

            elif func == 0xFE and sub == 0x28 and dlc == 8 and len(data) >= 8:
                self.cell_max_mv = u16le(data, 0)
                self.cell_min_mv = u16le(data, 2)
                for i in range(2):
                    raw = data[4 + i]
                    if raw != 0:
                        self.temp_c[i + 1] = float(raw) - 40.0
                self.disch_i_lim = u16le(data, 6) / 10.0

    # ── Extra sensor decoder ──────────────────────────────────────────────────
    def decode_sensor(self, key: str, raw_val: str):
        """Parse a SENSOR:key=value line and update the matching attribute."""
        try:
            val = float(raw_val)
        except ValueError:
            return
        known = {
            "hv_iso_v", "aux12v",
            "mppt_i1", "mppt_i2",
            "bat_t1", "bat_t2",
            "mppt_t", "dcdc_t",
            "handbrake", "oil_level",
        }
        if key in known:
            with self.lock:
                self.last_update = datetime.now()
                setattr(self, key, val)

    def cell_status(self, mv):
        if mv >= self.CELL_OV:      return "overvoltage"
        if mv >= self.CELL_FULL:    return "full"
        if mv >= self.CELL_BAL_THR: return "good"
        if mv >= self.CELL_NOMINAL: return "normal"
        if mv >= self.CELL_UV:      return "low"
        return "undervoltage"

    def to_dict(self):
        with self.lock:
            cv   = dict(self.cell_mv)
            temp = dict(self.temp_c)
            soc  = self.soc_ffe5 if self.soc_ffe5 is not None else self.soc

            def r(v, d=2): return round(v, d) if v is not None else None

            cells_out = {str(k): {"mv": cv[k], "status": self.cell_status(cv[k])}
                         for k in sorted(cv)}
            return {
                # ── standard BMS fields ──────────────────────────────────────
                "connected":      self.connected,
                "port":           self.port_name,
                "frame_count":    self.frame_count,
                "timestamp":      self.last_update.isoformat() if self.last_update else None,
                "soc":            r(soc, 1),
                "pack_v":         r(self.pack_v, 2),
                "chg_i_req":      r(self.chg_i_req, 1),
                "disch_i_lim":    r(self.disch_i_lim, 1),
                "cells":          cells_out,
                "cell_count":     len(cv),
                "cell_max_mv":    self.cell_max_mv,
                "cell_min_mv":    self.cell_min_mv,
                "cell_spread_mv": (max(cv.values()) - min(cv.values())) if len(cv) > 1 else None,
                "cell_avg_mv":    round(sum(cv.values()) / len(cv))     if cv else None,
                "temps":          {str(k): round(v, 1) for k, v in sorted(temp.items())},
                "avg_temp":       round(sum(temp.values()) / len(temp), 1) if temp else None,
                "thresh": {
                    "cell_ov":    self.CELL_OV,
                    "cell_full":  self.CELL_FULL,
                    "cell_nom":   self.CELL_NOMINAL,
                    "cell_bal":   self.CELL_BAL_THR,
                    "cell_uv":    self.CELL_UV,
                    "pack_full":  self.PACK_FULL_V,
                    "pack_empty": self.PACK_EMPTY_V,
                    "max_chg_a":  self.MAX_CHARGE_A,
                    "max_dch_a":  self.MAX_DISCH_A,
                },
                "log": list(self.raw_log)[-80:],

                # ── extra sensors ────────────────────────────────────────────
                "ext": {
                    "hv_iso_v":  r(self.hv_iso_v),
                    "aux12v":    r(self.aux12v),
                    "mppt_i1":   r(self.mppt_i1),
                    "mppt_i2":   r(self.mppt_i2),
                    "bat_t1":    r(self.bat_t1, 1),
                    "bat_t2":    r(self.bat_t2, 1),
                    "mppt_t":    r(self.mppt_t, 1),
                    "dcdc_t":    r(self.dcdc_t, 1),
                    "handbrake": int(self.handbrake) if self.handbrake is not None else None,
                    "oil_level": r(self.oil_level, 0),
                },
                "ext_thresh": {
                    "hv_iso_nom":    self.HV_ISO_NOM,
                    "aux12_nom":     self.AUX12_NOM,
                    "aux12_low":     self.AUX12_LOW,
                    "aux12_high":    self.AUX12_HIGH,
                    "mppt_max_a":    self.MPPT_MAX_A,
                    "bat_surf_warn": self.BAT_SURF_WARN,
                    "bat_surf_err":  self.BAT_SURF_ERR,
                    "hs_warn":       self.HS_WARN,
                    "hs_err":        self.HS_ERR,
                },
            }


# ─────────────────────────────────────────────────────────────────────────────
# Serial reader
# ─────────────────────────────────────────────────────────────────────────────

def auto_detect_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if any(k in desc + hwid for k in ["cp210", "ch340", "ftdi", "esp32", "silicon labs", "uart"]):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


_start_time = time.time()
def millis() -> int:
    return int((time.time() - _start_time) * 1000)


def normalize_line(line: str) -> str:
    if line.startswith("[") and "ID:" in line and "Data:" in line:
        return line
    line = line.replace("Extended ID:", "ID:").replace("extended ID:", "ID:")
    if "ID:" in line and "Data:" in line:
        head, data_part = line.split("Data:", 1)
        clean_bytes = " ".join(
            tok.replace("0x", "").replace("0X", "")
            for tok in data_part.split()
            if tok.strip()
        )
        return f"[{millis()}ms] {head.strip()} Data: {clean_bytes}"
    return line


def serial_reader(port, baud, state, stop):
    while not stop.is_set():
        try:
            with serial.Serial(port, baud, timeout=1) as ser:
                state.connected = True
                state.port_name = port
                state.raw_log.append(f"[INFO] Connected to {port} @ {baud}")
                while not stop.is_set():
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if not line:
                        continue

                    # ── Extra sensor line? ────────────────────────────────────
                    m = SENSOR_RE.match(line)
                    if m:
                        state.raw_log.append(line)
                        state.decode_sensor(m.group(1), m.group(2).strip())
                        continue

                    # ── CAN frame line ────────────────────────────────────────
                    line = normalize_line(line)
                    state.raw_log.append(line)
                    result = parse_line(line)
                    if result:
                        state.decode(*result)

        except serial.SerialException as e:
            state.connected = False
            state.raw_log.append(f"[ERR] {e}")
            time.sleep(2)
        except Exception as e:
            state.raw_log.append(f"[ERR] {e}")
            time.sleep(1)


# ─────────────────────────────────────────────────────────────────────────────
# FastAPI
# ─────────────────────────────────────────────────────────────────────────────

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

bms   = BMSState()
_stop = threading.Event()


@app.get("/", response_class=HTMLResponse)
async def index():
    with open("index.html", encoding="utf-8") as f:
        return f.read()


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json(bms.to_dict())
            await asyncio.sleep(0.1)   # 10 Hz
    except (WebSocketDisconnect, Exception):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",     "-p")
    parser.add_argument("--baud",     "-b", type=int, default=115200)
    parser.add_argument("--host",           default="0.0.0.0")
    parser.add_argument("--web-port",       type=int, default=8765, dest="web_port")
    args = parser.parse_args()

    port = args.port or auto_detect_port()
    if not port:
        print("❌  No serial port found. Use --port to specify one.")
        for p in serial.tools.list_ports.comports():
            print(f"    {p.device}  {p.description}")
        sys.exit(1)

    print(f"🔌  Serial : {port} @ {args.baud} baud")
    print(f"🌐  Open   : http://localhost:{args.web_port}")

    threading.Thread(target=serial_reader, args=(port, args.baud, bms, _stop),
                     daemon=True).start()
    uvicorn.run(app, host=args.host, port=args.web_port, log_level="warning")


if __name__ == "__main__":
    main()
