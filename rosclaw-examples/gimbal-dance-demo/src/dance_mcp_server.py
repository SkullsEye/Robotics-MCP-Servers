"""
ROSClaw Dance Demo — Dance MCP Server (The Brain ↔ Cerebellum Bridge)

Exposes high-level semantic choreography tools to the LLM (OpenClaw).
The LLM acts as "Choreographer": it analyzes music, assigns souls to gimbal
groups, and dispatches concurrent rhythm behaviors. All hardware-level
50Hz waveform computation happens in the Rhythm Engine below.

Multi-Agent Groups:
  🥁 drummers   (IDs 1-3)  — headbang, square wave, tilt axis
  🎤 vocals     (IDs 4-7)  — sweep, sine wave, pan axis
  ✨ spotlight  (IDs 8-10) — figure8 / strobe_center

Usage:
  python src/dance_mcp_server.py              # real hardware
  python src/dance_mcp_server.py --sim        # simulation mode (no serial needed)
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from typing import Dict, List, Optional

from mcp.server.fastmcp import FastMCP

from music_analyzer import analyze_music
from rhythm_engine import RhythmEngine, BehaviorConfig

# ─── MCP Server ───────────────────────────────────────────────────────────────

mcp = FastMCP("rosclaw-dance")

# ─── Global State ─────────────────────────────────────────────────────────────

_engine: Optional[RhythmEngine] = None
_simulation: bool = "--sim" in sys.argv

# Agent group definitions (soul assignments)
_GROUPS: Dict[str, Dict] = {
    "drummers": {
        "ids":  [1, 2, 3],
        "soul": "Aggressive metal drummers — explosive tilt snaps on every beat",
        "default_behavior": "headbang",
    },
    "vocals": {
        "ids":  [4, 5, 6, 7],
        "soul": "Elegant lead vocalists — smooth wide pan sweeps, like ocean waves",
        "default_behavior": "wave",
    },
    "spotlight": {
        "ids":  [8, 9, 10],
        "soul": "Godlike spotlight operators — slow circles at rest, strobe to center on Drop",
        "default_behavior": "slow_circle",
    },
}

VALID_BEHAVIORS = [
    "headbang", "sweep", "wave", "heartbeat",
    "figure8", "strobe_center", "slow_circle", "freeze",
]


# ─── MCP Tools ────────────────────────────────────────────────────────────────

@mcp.tool()
async def connect_dance_floor(config_path: str = "config/gimbal_config.yaml") -> str:
    """
    Connect to all gimbals on the dance floor and initialize the Rhythm Engine.

    Reads gimbal port configuration from YAML. Pass simulation=True (via --sim
    flag at startup) to run without physical hardware.

    Args:
        config_path: Path to gimbal_config.yaml
    """
    global _engine

    if _engine is not None:
        return "Dance floor already connected. Use disconnect_dance_floor to reset."

    # Load config
    ports = _load_gimbal_config(config_path)

    _engine = RhythmEngine()
    connected = []
    failed = []

    for gimbal_id, port in ports.items():
        ok = _engine.add_gimbal(gimbal_id, port, simulation=_simulation)
        (connected if ok else failed).append(gimbal_id)

    mode_str = "SIMULATION MODE 🖥️" if _simulation else "HARDWARE MODE 🔌"
    result = (
        f"✓ Dance floor connected [{mode_str}]\n"
        f"  Connected gimbals: {connected}\n"
        f"  Groups:\n"
    )
    for name, g in _GROUPS.items():
        result += f"    {name}: IDs {g['ids']} — \"{g['soul']}\"\n"

    if failed:
        result += f"\n  ⚠ Failed to connect: {failed}"

    return result.strip()


@mcp.tool()
async def analyze_music_file(file_path: str) -> str:
    """
    Analyze an audio file and extract music structure for choreography.

    Extracts BPM, beat grid, segment boundaries (intro/buildup/drop/outro),
    energy curve, and emotional vibe. Use the results to decide which behaviors
    to assign to each group and when transitions should happen.

    Args:
        file_path: Absolute or relative path to the audio file.
                   Supports .mp3, .wav, .flac, .ogg
                   Pass "demo" or "simulate" to use a synthetic 128 BPM EDM track.
    """
    if file_path.lower() in ("demo", "simulate", "test"):
        file_path = ""  # triggers mock analysis

    result = analyze_music(file_path)

    if "error" in result:
        return f"Error: {result['error']}"

    # Format for LLM readability
    segs = "\n".join(
        f"  [{s['label'].upper():<8}] {s['start']:5.1f}s → {s['end']:5.1f}s  "
        f"intensity={s['intensity']:.1f}  bpm_mult={s['bpm_multiplier']}"
        for s in result["segments"]
    )
    return (
        f"Music Analysis:\n"
        f"  {result['summary']}\n\n"
        f"Segments:\n{segs}\n\n"
        f"First beats: {result['beat_times'][:8]} ...\n"
        f"JSON (for choreograph_dance):\n{json.dumps(result, indent=2)}"
    )


@mcp.tool()
async def execute_rhythm_behavior(
    group: str,
    behavior: str,
    bpm: float,
    intensity: float,
    duration: float,
    phase_wave: bool = False,
) -> str:
    """
    Command a gimbal group to perform a rhythm behavior.

    This is the core choreography tool. Call it concurrently for multiple
    groups to create synchronized multi-group performances.

    Args:
        group:      "drummers", "vocals", or "spotlight"
                    (or a comma-separated list of IDs like "1,2,3")
        behavior:   "headbang"     — Square wave tilt snaps (drums)
                    "sweep"        — Sine pan sweep (slow, smooth)
                    "wave"         — Sine pan with group ripple effect
                    "heartbeat"    — Double-pulse tilt (accent hits)
                    "figure8"      — Lissajous pan+tilt (spotlight)
                    "strobe_center"— Rapid flicker → snap to center (drop)
                    "slow_circle"  — Ambient slow circular scan
                    "freeze"       — Hold current position
        bpm:        Beat tempo in BPM (from music analysis)
        intensity:  Motion amplitude 0.0 (minimal) to 1.0 (full range)
        duration:   Seconds to run this behavior. 0 = run until stop_all.
        phase_wave: If True, applies progressive phase offsets across the
                    group for a ripple/wave visual effect (great for vocals).
    """
    global _engine

    if _engine is None:
        return "Error: Dance floor not connected. Call connect_dance_floor first."

    # Resolve group → gimbal IDs
    if group in _GROUPS:
        ids = _GROUPS[group]["ids"]
        group_desc = f"group '{group}'"
    else:
        try:
            ids = [int(x.strip()) for x in group.split(",")]
            group_desc = f"gimbals {ids}"
        except ValueError:
            return (f"Error: Unknown group '{group}'. "
                    f"Valid: {list(_GROUPS.keys())} or comma-separated IDs")

    if behavior not in VALID_BEHAVIORS:
        return f"Error: Unknown behavior '{behavior}'. Valid: {VALID_BEHAVIORS}"

    if not (0.0 <= intensity <= 1.0):
        return "Error: intensity must be 0.0 to 1.0"

    if not (20.0 <= bpm <= 300.0):
        return "Error: bpm must be 20–300"

    results = _engine.execute_group(
        gimbal_ids=ids,
        behavior=behavior,
        bpm=bpm,
        intensity=intensity,
        duration=duration,
        phase_wave=phase_wave,
    )

    ok_ids   = [gid for gid, r in results.items() if r == "ok"]
    fail_ids = [gid for gid, r in results.items() if r != "ok"]

    status = f"✓ {group_desc}: {behavior} @ {bpm} BPM, intensity={intensity}, duration={duration}s"
    if phase_wave:
        status += " (wave mode)"
    if fail_ids:
        status += f"\n  ⚠ Not connected: {fail_ids}"

    return status


@mcp.tool()
async def choreograph_dance(music_json: str, style: str = "auto") -> str:
    """
    Generate a full choreography script from music analysis.

    The LLM (you) should use this to create a timed sequence of
    execute_rhythm_behavior calls that match the music structure.

    Args:
        music_json: JSON string from analyze_music_file
        style:      "auto"     — LLM decides based on vibe
                    "dramatic" — Maximum contrast between intro and drop
                    "smooth"   — Gentle continuous motion throughout
                    "chaotic"  — Maximum energy at all times

    Returns a choreography script as JSON with a sequence of timed commands
    that can be fed into run_choreography.
    """
    try:
        music = json.loads(music_json) if isinstance(music_json, str) else music_json
    except json.JSONDecodeError:
        return "Error: music_json must be valid JSON from analyze_music_file"

    bpm      = music.get("bpm", 120.0)
    segments = music.get("segments", [])
    vibe     = music.get("vibe", "Unknown")

    # Build choreography script
    script = {"title": f"Dance to {vibe}", "bpm": bpm, "style": style, "commands": []}

    for seg in segments:
        label     = seg["label"]
        start     = seg["start"]
        end       = seg["end"]
        intensity = seg["intensity"]
        dur       = round(end - start, 1)
        bpm_seg   = round(bpm * seg.get("bpm_multiplier", 1.0), 1)

        if label == "intro":
            script["commands"].append({
                "t": start, "group": "vocals",    "behavior": "slow_circle",
                "bpm": bpm_seg * 0.5, "intensity": intensity * 0.5,
                "duration": dur, "phase_wave": True,
            })
            script["commands"].append({
                "t": start, "group": "spotlight", "behavior": "slow_circle",
                "bpm": bpm_seg * 0.25, "intensity": intensity * 0.3,
                "duration": dur, "phase_wave": False,
            })

        elif label == "buildup":
            script["commands"].append({
                "t": start, "group": "drums",    "behavior": "heartbeat",
                "bpm": bpm_seg, "intensity": intensity * 0.7, "duration": dur,
            })
            script["commands"].append({
                "t": start, "group": "vocals",   "behavior": "wave",
                "bpm": bpm_seg, "intensity": intensity * 0.8,
                "duration": dur, "phase_wave": True,
            })
            script["commands"].append({
                "t": start, "group": "spotlight", "behavior": "figure8",
                "bpm": bpm_seg, "intensity": intensity * 0.6, "duration": dur,
            })

        elif label == "drop":
            script["commands"].append({
                "t": start, "group": "drummers",  "behavior": "headbang",
                "bpm": bpm_seg, "intensity": 1.0, "duration": dur,
            })
            script["commands"].append({
                "t": start, "group": "vocals",    "behavior": "wave",
                "bpm": bpm_seg, "intensity": 1.0, "duration": dur, "phase_wave": True,
            })
            script["commands"].append({
                "t": start, "group": "spotlight", "behavior": "strobe_center",
                "bpm": bpm_seg * 2, "intensity": 1.0, "duration": dur,
            })

        elif label == "outro":
            script["commands"].append({
                "t": start, "group": "vocals",    "behavior": "sweep",
                "bpm": bpm_seg * 0.5, "intensity": intensity * 0.4,
                "duration": dur, "phase_wave": True,
            })
            script["commands"].append({
                "t": start, "group": "spotlight", "behavior": "slow_circle",
                "bpm": bpm_seg * 0.25, "intensity": intensity * 0.3, "duration": dur,
            })

    return (
        f"Choreography Script ({len(script['commands'])} commands):\n"
        + json.dumps(script, indent=2)
        + "\n\nCall run_choreography with this JSON to execute."
    )


@mcp.tool()
async def run_choreography(script_json: str) -> str:
    """
    Execute a full timed choreography script.

    Schedules all commands from choreograph_dance at the correct timestamps
    and runs the full performance end-to-end.

    Args:
        script_json: JSON choreography script from choreograph_dance
    """
    global _engine

    if _engine is None:
        return "Error: Dance floor not connected. Call connect_dance_floor first."

    try:
        script = json.loads(script_json)
    except json.JSONDecodeError:
        return "Error: script_json must be valid JSON from choreograph_dance"

    commands = script.get("commands", [])
    if not commands:
        return "Error: No commands in script"

    title = script.get("title", "Dance Performance")
    start_wall = time.perf_counter()

    async def _schedule(cmd: dict):
        target_t = cmd["t"]
        now_elapsed = time.perf_counter() - start_wall
        wait_for = target_t - now_elapsed
        if wait_for > 0:
            await asyncio.sleep(wait_for)
        await execute_rhythm_behavior(
            group=cmd["group"],
            behavior=cmd["behavior"],
            bpm=cmd["bpm"],
            intensity=cmd["intensity"],
            duration=cmd["duration"],
            phase_wave=cmd.get("phase_wave", False),
        )

    # Schedule all commands concurrently
    tasks = [asyncio.create_task(_schedule(cmd)) for cmd in commands]
    total_duration = max((c["t"] + c["duration"]) for c in commands)
    await asyncio.gather(*tasks)

    # Wait for all behaviors to finish
    remaining = total_duration - (time.perf_counter() - start_wall)
    if remaining > 0:
        await asyncio.sleep(remaining)

    return (
        f"✓ '{title}' performance complete!\n"
        f"  Duration: {total_duration:.1f}s | Commands: {len(commands)}"
    )


@mcp.tool()
async def stop_all_gimbals() -> str:
    """
    Immediately stop all gimbals and return to center position.
    Use in emergencies or after a performance.
    """
    global _engine

    if _engine is None:
        return "Not connected"

    _engine.stop_all()
    return "✓ All gimbals stopped and returned to center (0°, 0°)"


@mcp.tool()
async def disconnect_dance_floor() -> str:
    """Disconnect from all gimbals and shut down the Rhythm Engine."""
    global _engine

    if _engine is None:
        return "Not connected"

    _engine.shutdown()
    _engine = None
    return "✓ Dance floor disconnected"


# ─── MCP Resources ────────────────────────────────────────────────────────────

@mcp.resource("dance://status")
async def get_dance_status() -> str:
    """Real-time status of all 10 gimbals"""
    global _engine

    if _engine is None:
        return "Dance floor not connected"

    dancers = _engine.status()
    if not dancers:
        return "No gimbals connected"

    lines = ["Dance Floor Status:", ""]
    for d in dancers:
        group = next((g for g, info in _GROUPS.items() if d["id"] in info["ids"]), "?")
        bar_p = _bar(d["pan"],  120)
        bar_t = _bar(d["tilt"],  60)
        sim   = " [SIM]" if d["sim"] else ""
        lines.append(
            f"  G{d['id']:02d} [{group:<10}]{sim}  "
            f"PAN {bar_p} {d['pan']:+6.1f}°  TILT {bar_t} {d['tilt']:+5.1f}°  "
            f"ticks={d['ticks']}"
        )

    return "\n".join(lines)


@mcp.resource("dance://groups")
async def get_groups() -> str:
    """Agent group assignments and souls"""
    lines = ["Gimbal Groups & Souls:\n"]
    for name, g in _GROUPS.items():
        lines.append(f"  {name}  IDs={g['ids']}")
        lines.append(f"    Soul: \"{g['soul']}\"")
        lines.append(f"    Default: {g['default_behavior']}")
        lines.append("")
    lines.append(f"Valid behaviors: {VALID_BEHAVIORS}")
    return "\n".join(lines)


@mcp.resource("dance://behaviors")
async def get_behaviors() -> str:
    """Available behaviors and their descriptions"""
    return """Available Rhythm Behaviors:

  headbang     — Square wave tilt.  Drums. Snaps up/down each beat.
  sweep        — Sine pan.          Vocals. Smooth left/right oscillation.
  wave         — Sine pan + phase.  Vocals. Ripple wave across group.
  heartbeat    — Double-pulse tilt. Accents. Two quick tilt pulses per beat.
  figure8      — Lissajous.         Spotlight. Pan=sin(φ) Tilt=sin(2φ).
  strobe_center— Flicker + snap.    Drop moment. Random flicker → center.
  slow_circle  — Slow sine circle.  Ambient. Calm background motion.
  freeze       — Hold position.     Pause. All axes locked at 0°.

Phase wave (phase_wave=True):
  Each gimbal in the group gets a shifted phase offset,
  creating a ripple effect across the group. Best with "wave" or "sweep".

Axis orientation:
  PAN  (yaw):  ±120° horizontal sweep
  TILT (pitch): -80° (down) to +30° (up)
"""


# ─── Helpers ──────────────────────────────────────────────────────────────────

def _bar(val: float, max_val: float, w: int = 8) -> str:
    ratio  = max(0.0, min(1.0, (val / max_val + 1.0) / 2.0))
    filled = int(ratio * w)
    return "[" + "█" * filled + "░" * (w - filled) + "]"


def _load_gimbal_config(config_path: str) -> Dict[int, str]:
    """Load gimbal ID → serial port mapping from YAML config."""
    if not os.path.exists(config_path):
        # Fall back to default simulation ports
        return {i: f"SIM{i}" for i in range(1, 11)}

    try:
        import yaml  # type: ignore
        with open(config_path) as f:
            cfg = yaml.safe_load(f)
        gimbals = cfg.get("gimbals", [])
        return {int(g["id"]): g["port"] for g in gimbals}
    except Exception as e:
        print(f"[Config] Failed to load {config_path}: {e} — using simulation ports")
        return {i: f"SIM{i}" for i in range(1, 11)}


if __name__ == "__main__":
    mode = "SIMULATION" if _simulation else "HARDWARE"
    print(f"ROSClaw Dance MCP Server starting [{mode} MODE]...")
    print("Tools: connect_dance_floor, analyze_music_file, execute_rhythm_behavior,")
    print("       choreograph_dance, run_choreography, stop_all_gimbals")
    mcp.run(transport="stdio")
