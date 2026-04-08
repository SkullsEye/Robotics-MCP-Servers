"""
ROSClaw Dance Demo — Rhythm Engine (The Cerebellum)

Converts high-level semantic behaviors (headbang @ 120 BPM) into
50Hz serial commands to GCU gimbals. The LLM never touches this layer.

Protocol: GCU Private Communication Protocol V2.0.6
Reuses the same serial protocol as rosclaw-gimbal-mcp.

Waveforms:
  headbang     — Square wave on tilt (drums: snap up/down per beat)
  sweep        — Sine wave on pan (vocals: smooth left/right)
  heartbeat    — Double-pulse on tilt (accent/snare hits)
  figure8      — Lissajous on both axes (spotlight: figure-8)
  wave         — Sine with phase offset across group (ocean ripple)
  strobe_center— Rapid flicker then snap to center (drop moment)
  freeze       — Hold current position
"""

from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

try:
    import serial
    _HAS_SERIAL = True
except ImportError:
    _HAS_SERIAL = False


# ─── GCU Protocol (same as rosclaw-gimbal-mcp) ───────────────────────────────

_CRC_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
]


def _crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= _CRC_TABLE[da ^ ((byte >> 4) & 0x0F)]
        da = (crc >> 12) & 0x0F
        crc = (crc << 4) & 0xFFFF
        crc ^= _CRC_TABLE[da ^ (byte & 0x0F)]
    return crc


def _build_angle_packet(pitch_cdeg: int, yaw_cdeg: int) -> bytes:
    """
    Build GCU angle control packet.
    pitch_cdeg, yaw_cdeg: angle in centidegrees (0.01°), signed int16
    """
    HEADER = bytes([0xA8, 0xE5])
    VERSION = 0x01
    FLAG = 0x04 | 0x01  # control_valid | imu_valid

    packet = bytearray()
    packet.extend(HEADER)
    packet.extend(struct.pack("<H", 72))  # base length
    packet.append(VERSION)

    # Data frame (32 bytes): roll, pitch, yaw, flag, aircraft data...
    packet.extend(struct.pack("<h", 0))            # roll control
    packet.extend(struct.pack("<h", pitch_cdeg))   # pitch control
    packet.extend(struct.pack("<h", yaw_cdeg))     # yaw control
    packet.append(FLAG)
    # aircraft attitude + accel + vel (zeros for ground demo)
    packet.extend(bytes(24))

    # Sub-frame (32 bytes)
    packet.append(0x01)
    packet.extend(bytes(31))

    # Command 0x00 = angle mode (no extra params)
    packet.append(0x00)

    crc = _crc16(packet)
    packet.extend(struct.pack(">H", crc))
    return bytes(packet)


# ─── Waveform Engine ──────────────────────────────────────────────────────────

@dataclass
class BehaviorConfig:
    behavior: str         # waveform name
    bpm: float            # beats per minute
    intensity: float      # amplitude 0.0–1.0
    phase_offset: float   # radians (for wave effect)
    duration: float       # seconds, 0 = infinite
    start_time: float = field(default_factory=time.perf_counter)


# Maximum swing angles per axis (degrees)
MAX_PAN_DEG  = 120.0
MAX_TILT_DEG = 60.0


def compute_angles(cfg: BehaviorConfig, t: float) -> tuple[float, float]:
    """
    Compute (pan, tilt) in degrees at time t for the given behavior.
    t is seconds since epoch (use time.perf_counter()).
    Returns (pan_deg, tilt_deg).
    """
    elapsed = t - cfg.start_time
    period  = 60.0 / cfg.bpm          # seconds per beat
    phi     = (elapsed / period) * (2 * math.pi) + cfg.phase_offset
    amp_pan  = MAX_PAN_DEG  * cfg.intensity
    amp_tilt = MAX_TILT_DEG * cfg.intensity

    b = cfg.behavior

    if b == "headbang":
        # Square wave on tilt — snap up on beat, snap down on off-beat
        snap = 1.0 if (elapsed % period) < (period * 0.35) else -1.0
        return 0.0, amp_tilt * snap

    elif b == "sweep":
        # Sine on pan — smooth left/right sweep
        return amp_pan * math.sin(phi), 0.0

    elif b == "wave":
        # Sine on pan with phase offset intact (used for ripple across a group)
        return amp_pan * math.sin(phi), amp_tilt * 0.2 * math.sin(phi * 2)

    elif b == "heartbeat":
        # Double pulse: two quick tilt pulses per beat (like lub-dub)
        t_in = (elapsed % period) / period
        pulse = (math.exp(-30 * (t_in - 0.1)**2) + math.exp(-30 * (t_in - 0.35)**2))
        return 0.0, amp_tilt * min(pulse, 1.0)

    elif b == "figure8":
        # Lissajous: pan = sin(φ), tilt = sin(2φ)
        return amp_pan * math.sin(phi), amp_tilt * math.sin(2 * phi)

    elif b == "strobe_center":
        # Fast random-ish flicker at 4× BPM, then snap to center at beat
        flicker_phi = phi * 4
        jitter = math.sin(flicker_phi) * math.cos(flicker_phi * 1.618)
        return amp_pan * jitter * 0.5, amp_tilt * jitter * 0.5

    elif b == "slow_circle":
        # Very slow circular pan — calm ambient motion
        slow_phi = phi * 0.25
        return amp_pan * 0.4 * math.sin(slow_phi), amp_tilt * 0.2 * math.cos(slow_phi)

    else:  # "freeze" or unknown
        return 0.0, 0.0


# ─── Per-Gimbal Dancer ────────────────────────────────────────────────────────

class GimbalDancer:
    """
    Controls a single gimbal at 50Hz.
    In simulation mode, prints angles instead of writing to serial.
    """

    TICK_HZ = 50  # hardware command frequency

    def __init__(self, gimbal_id: int, port: str, simulation: bool = False):
        self.gimbal_id = gimbal_id
        self.port = port
        self.simulation = simulation

        self._serial: Optional["serial.Serial"] = None
        self._cfg: Optional[BehaviorConfig] = None
        self._cfg_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        # Telemetry (for status display)
        self.current_pan: float  = 0.0
        self.current_tilt: float = 0.0
        self.tick_count: int     = 0

    def connect(self) -> bool:
        if self.simulation:
            return True
        if not _HAS_SERIAL:
            return False
        try:
            self._serial = serial.Serial(
                port=self.port, baudrate=115200,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,
            )
            return True
        except Exception as e:
            print(f"[Gimbal {self.gimbal_id}] Serial error: {e}")
            return False

    def disconnect(self):
        self.stop()
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None

    def start(self):
        """Start the 50Hz dance loop in a daemon thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name=f"Dancer-{self.gimbal_id}"
        )
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        # Return to center
        self._send_angles(0.0, 0.0)

    def set_behavior(self, cfg: BehaviorConfig):
        with self._cfg_lock:
            self._cfg = cfg

    def clear_behavior(self):
        with self._cfg_lock:
            self._cfg = None

    def _loop(self):
        interval = 1.0 / self.TICK_HZ
        next_tick = time.perf_counter()
        while not self._stop_event.is_set():
            now = time.perf_counter()
            with self._cfg_lock:
                cfg = self._cfg

            if cfg is not None:
                # Check duration
                if cfg.duration > 0 and (now - cfg.start_time) >= cfg.duration:
                    self._cfg = None
                    self._send_angles(0.0, 0.0)
                else:
                    pan, tilt = compute_angles(cfg, now)
                    self._send_angles(pan, tilt)
                    self.current_pan  = pan
                    self.current_tilt = tilt
                    self.tick_count  += 1

            next_tick += interval
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _send_angles(self, pan_deg: float, tilt_deg: float):
        pan_cdeg  = max(-17000, min(17000, int(pan_deg  * 100)))
        tilt_cdeg = max(-8000,  min(3000,  int(tilt_deg * 100)))

        if self.simulation:
            # Visual bar for terminal display
            bar_pan  = _angle_bar(pan_deg,  MAX_PAN_DEG)
            bar_tilt = _angle_bar(tilt_deg, MAX_TILT_DEG)
            print(f"\r[G{self.gimbal_id:02d}] PAN {bar_pan} {pan_deg:+7.1f}°  "
                  f"TILT {bar_tilt} {tilt_deg:+6.1f}°", end="", flush=True)
        elif self._serial and self._serial.is_open:
            try:
                pkt = _build_angle_packet(tilt_cdeg, pan_cdeg)
                self._serial.write(pkt)
            except Exception:
                pass  # Best-effort; re-connect handled at higher level


def _angle_bar(angle: float, max_angle: float, width: int = 10) -> str:
    """ASCII progress bar for terminal visualization."""
    ratio = (angle / max_angle + 1.0) / 2.0  # 0..1
    filled = int(ratio * width)
    return "[" + "█" * filled + "░" * (width - filled) + "]"


# ─── Rhythm Engine (manages all dancers) ─────────────────────────────────────

class RhythmEngine:
    """
    The Cerebellum: manages N GimbalDancers, translates semantic group
    behaviors into per-gimbal 50Hz waveforms.
    """

    def __init__(self):
        self._dancers: Dict[int, GimbalDancer] = {}
        self._lock = threading.Lock()

    def add_gimbal(self, gimbal_id: int, port: str, simulation: bool = False) -> bool:
        dancer = GimbalDancer(gimbal_id, port, simulation)
        if not dancer.connect():
            return False
        dancer.start()
        with self._lock:
            self._dancers[gimbal_id] = dancer
        return True

    def execute_group(
        self,
        gimbal_ids: List[int],
        behavior: str,
        bpm: float,
        intensity: float,
        duration: float,
        phase_wave: bool = False,
    ) -> Dict[str, object]:
        """
        Assign a behavior to a group of gimbals.
        With phase_wave=True, each gimbal gets a shifted phase for a ripple effect.
        """
        n = len(gimbal_ids)
        results = {}
        for i, gid in enumerate(gimbal_ids):
            phase = (i / n) * (2 * math.pi) if (phase_wave and n > 1) else 0.0
            cfg = BehaviorConfig(
                behavior=behavior,
                bpm=bpm,
                intensity=intensity,
                phase_offset=phase,
                duration=duration,
                start_time=time.perf_counter(),
            )
            with self._lock:
                dancer = self._dancers.get(gid)
            if dancer:
                dancer.set_behavior(cfg)
                results[gid] = "ok"
            else:
                results[gid] = "not_connected"
        return results

    def stop_all(self):
        with self._lock:
            dancers = list(self._dancers.values())
        for d in dancers:
            d.clear_behavior()
            d._send_angles(0.0, 0.0)

    def shutdown(self):
        self.stop_all()
        with self._lock:
            dancers = list(self._dancers.values())
        for d in dancers:
            d.disconnect()

    def status(self) -> List[Dict]:
        with self._lock:
            dancers = list(self._dancers.values())
        return [
            {
                "id":   d.gimbal_id,
                "port": d.port,
                "pan":  round(d.current_pan, 1),
                "tilt": round(d.current_tilt, 1),
                "ticks": d.tick_count,
                "sim":  d.simulation,
            }
            for d in sorted(dancers, key=lambda x: x.gimbal_id)
        ]
