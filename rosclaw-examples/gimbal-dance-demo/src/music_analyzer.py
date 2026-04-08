"""
ROSClaw Dance Demo — Music Analyzer (The Brain's Ears)

Extracts semantic music structure from an audio file using librosa.
The LLM calls this to understand the music before choreographing.

Returns:
  - BPM (tempo)
  - Beat timestamps
  - Segment boundaries with labels (intro/buildup/drop/outro)
  - Energy curve (for intensity mapping)
  - Emotional vibe (derived from spectral features)
"""

from __future__ import annotations

import json
import os
from typing import Any, Dict, List

try:
    import librosa
    import numpy as np
    _HAS_LIBROSA = True
except ImportError:
    _HAS_LIBROSA = False


def analyze_music(file_path: str, hop_length: int = 512) -> Dict[str, Any]:
    """
    Analyze an audio file and return semantic music structure.

    Args:
        file_path: Path to audio file (.mp3, .wav, .flac, .ogg, etc.)
        hop_length: librosa hop length (controls time resolution)

    Returns:
        dict with keys:
          bpm          — float, detected tempo
          beat_times   — list of float, beat timestamps in seconds
          duration     — float, total audio duration in seconds
          segments     — list of segment dicts
          energy_curve — list of (time, energy) tuples, normalized 0-1
          vibe         — str, emotional descriptor
          summary      — str, human-readable summary for LLM
    """
    if not _HAS_LIBROSA:
        return _mock_analysis(file_path)

    if not os.path.exists(file_path):
        return {"error": f"File not found: {file_path}"}

    # ── Load audio ──────────────────────────────────────────────────────────
    y, sr = librosa.load(file_path, sr=None, mono=True)
    duration = librosa.get_duration(y=y, sr=sr)

    # ── Tempo & beats ────────────────────────────────────────────────────────
    tempo, beat_frames = librosa.beat.beat_track(y=y, sr=sr, hop_length=hop_length)
    bpm = float(tempo)
    beat_times = librosa.frames_to_time(beat_frames, sr=sr, hop_length=hop_length).tolist()

    # ── RMS energy curve ─────────────────────────────────────────────────────
    rms = librosa.feature.rms(y=y, hop_length=hop_length)[0]
    rms_times = librosa.frames_to_time(range(len(rms)), sr=sr, hop_length=hop_length)
    rms_norm = (rms / (rms.max() + 1e-8)).tolist()
    # Downsample to ~1 point per second for the LLM
    step = max(1, len(rms_norm) // int(duration))
    energy_curve = [
        {"t": round(float(rms_times[i]), 1), "e": round(rms_norm[i], 2)}
        for i in range(0, len(rms_norm), step)
    ]

    # ── Structural segmentation ───────────────────────────────────────────────
    segments = _detect_segments(y, sr, hop_length, duration, rms_norm, bpm)

    # ── Vibe detection ────────────────────────────────────────────────────────
    vibe = _detect_vibe(y, sr, bpm, float(np.mean(rms_norm)))

    summary = (
        f"Music: {os.path.basename(file_path)} | "
        f"Duration: {duration:.1f}s | BPM: {bpm:.1f} | Vibe: {vibe} | "
        f"{len(segments)} segments: " +
        ", ".join(f"{s['label']}@{s['start']:.0f}s" for s in segments)
    )

    return {
        "bpm":          round(bpm, 1),
        "beat_times":   [round(t, 3) for t in beat_times[:32]],  # first 32 beats
        "duration":     round(duration, 1),
        "segments":     segments,
        "energy_curve": energy_curve,
        "vibe":         vibe,
        "summary":      summary,
    }


def _detect_segments(y, sr, hop_length, duration, rms_norm, bpm) -> List[Dict]:
    """
    Detect song structure: intro → buildup → drop → outro
    using spectral flux + energy thresholds.
    """
    try:
        import numpy as np
        # Onset strength as a proxy for "energy building"
        onset_env = librosa.onset.onset_strength(y=y, sr=sr, hop_length=hop_length)
        # Normalize
        onset_norm = onset_env / (onset_env.max() + 1e-8)
        times = librosa.frames_to_time(range(len(onset_norm)), sr=sr, hop_length=hop_length)

        # Simple sliding window energy: split into ~4s chunks
        chunk_sec = 4.0
        chunk_frames = int(chunk_sec * sr / hop_length)
        n_chunks = len(onset_norm) // chunk_frames

        if n_chunks < 2:
            return [{"start": 0.0, "end": duration, "label": "main",
                     "intensity": 0.7, "bpm_multiplier": 1.0}]

        chunk_energy = []
        for i in range(n_chunks):
            s, e = i * chunk_frames, (i + 1) * chunk_frames
            chunk_energy.append(float(np.mean(onset_norm[s:e])))

        # Find drop (max energy chunk)
        drop_chunk = int(np.argmax(chunk_energy))
        drop_t     = round(drop_chunk * chunk_sec, 1)

        # Build segments
        segs = []
        if drop_t > 8.0:
            # Intro: 0 → buildup
            buildup_t = max(0.0, drop_t - min(16.0, drop_t * 0.3))
            if buildup_t > 4.0:
                segs.append({"start": 0.0,       "end": buildup_t, "label": "intro",
                              "intensity": 0.3, "bpm_multiplier": 0.5})
            segs.append({"start": buildup_t, "end": drop_t,    "label": "buildup",
                          "intensity": 0.6, "bpm_multiplier": 1.0})
        else:
            segs.append({"start": 0.0, "end": drop_t, "label": "intro",
                          "intensity": 0.3, "bpm_multiplier": 0.5})

        # Drop → outro
        outro_t = min(duration, drop_t + max(16.0, (duration - drop_t) * 0.7))
        segs.append({"start": drop_t,  "end": outro_t,  "label": "drop",
                      "intensity": 1.0, "bpm_multiplier": 1.0})
        if outro_t < duration - 4.0:
            segs.append({"start": outro_t, "end": duration, "label": "outro",
                          "intensity": 0.4, "bpm_multiplier": 0.5})

        return segs

    except Exception:
        beat_period = 60.0 / bpm
        return [{"start": 0.0, "end": duration, "label": "main",
                  "intensity": 0.7, "bpm_multiplier": 1.0}]


def _detect_vibe(y, sr, bpm: float, avg_energy: float) -> str:
    """Classify the emotional vibe using spectral and rhythm features."""
    try:
        import numpy as np
        spectral_centroid = float(np.mean(librosa.feature.spectral_centroid(y=y, sr=sr)))
        brightness = spectral_centroid / (sr / 2)  # 0..1

        if bpm > 140 and avg_energy > 0.5:
            return "Aggressive EDM"
        elif bpm > 120 and brightness > 0.25:
            return "Energetic Dance"
        elif bpm > 100 and avg_energy > 0.4:
            return "Upbeat Pop"
        elif bpm > 80 and brightness < 0.15:
            return "Deep / Dark"
        elif bpm < 80:
            return "Atmospheric / Ambient"
        else:
            return "Mid-tempo Groove"
    except Exception:
        return "Unknown"


def _mock_analysis(file_path: str) -> Dict[str, Any]:
    """
    Returns synthetic music analysis when librosa is not installed.
    Simulates a typical EDM track at 128 BPM for demo/testing purposes.
    """
    fname = os.path.basename(file_path) if file_path else "demo_track.mp3"
    return {
        "bpm":      128.0,
        "beat_times": [round(i * 60 / 128, 3) for i in range(32)],
        "duration": 60.0,
        "segments": [
            {"start":  0.0, "end": 15.0, "label": "intro",   "intensity": 0.3, "bpm_multiplier": 0.5},
            {"start": 15.0, "end": 30.0, "label": "buildup", "intensity": 0.6, "bpm_multiplier": 1.0},
            {"start": 30.0, "end": 52.0, "label": "drop",    "intensity": 1.0, "bpm_multiplier": 1.0},
            {"start": 52.0, "end": 60.0, "label": "outro",   "intensity": 0.4, "bpm_multiplier": 0.5},
        ],
        "energy_curve": [
            {"t": t, "e": round(0.2 + 0.8 * min(1.0, t / 30.0) if t < 30 else 0.9 - 0.5 * (t - 52) / 8 if t > 52 else 0.9, 2)}
            for t in range(0, 61, 2)
        ],
        "vibe":    "Energetic EDM (simulated — install librosa for real analysis)",
        "summary": (
            f"[SIMULATED] {fname} | 60.0s | BPM: 128.0 | Vibe: Energetic EDM | "
            "4 segments: intro@0s, buildup@15s, drop@30s, outro@52s"
        ),
        "_simulated": True,
    }
