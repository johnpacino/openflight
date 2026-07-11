"""Matplotlib-free radar-DSP core for the IWR6843 elevation array.

Extracted from music_stage0_sim.py so RUNTIME code — the Pi-side L3-dump
pipeline (iwr6843_l3dump.py) and the stage-1 analysis — can import the
estimators + array model without dragging in matplotlib. music_stage0_sim
re-imports these names, so existing callers (stage1_dryrun, stage1_analyze)
are unaffected.

Array model: uniform linear array, lambda/2 pitch, angle theta measured from
boresight. steer(theta, n)[m] = exp(j*pi*sin(theta)*m).
"""

import numpy as np

# ----------------------------------------------------------------- constants
C = 299_792_458.0
F_C = 62.0e9                 # 60-64 GHz chirp center
LAM = C / F_C                # ~4.84 mm
D_EL = LAM / 2.0             # lambda/2 element pitch
RANGE_RES = 0.0375           # c/(2*4GHz) — sim default; real cfg passes its own
G = 9.81

GRID = np.radians(np.arange(-40.0, 40.001, 0.1))
_STEER_CACHE = {}


def steer(theta, n_el):
    """lambda/2 ULA steering vector for elevation ``theta`` (radians)."""
    return np.exp(1j * np.pi * np.sin(theta) * np.arange(n_el))


def _grid_steer(m):
    if m not in _STEER_CACHE:
        _STEER_CACHE[m] = np.exp(
            1j * np.pi * np.sin(GRID)[None, :] * np.arange(m)[:, None]
        )
    return _STEER_CACHE[m]


def est_interferometry(x):
    """2-element phase interferometry (K-LD7-class baseline), middle pair."""
    dphi = np.angle(x[2] * np.conj(x[1]))
    return float(np.arcsin(np.clip(dphi / np.pi, -1.0, 1.0)))


def est_bartlett(x):
    """Conventional beamforming on the full array (stock-demo-class)."""
    a = _grid_steer(len(x))
    p = np.abs(a.conj().T @ x) ** 2
    return float(GRID[int(np.argmax(p))])


def est_music_fbss(x, noise_var):
    """Forward-backward spatially-smoothed MUSIC; returns direct-path angle.

    Direct pick rule: the floor image is always BELOW the direct arrival,
    so with two resolved sources take the higher-angle peak.
    """
    n = len(x)
    m = n // 2 + 1                       # subarray size: 3->2, 4->3, 6->4, 8->5
    p = n - m + 1
    r = np.zeros((m, m), dtype=complex)
    for i in range(p):
        xs = x[i:i + m]
        r += np.outer(xs, xs.conj())
    r /= p
    j = np.eye(m)[::-1]
    r = 0.5 * (r + j @ r.conj() @ j)
    ev, vec = np.linalg.eigh(r)          # ascending
    k_src = int(np.clip(np.sum(ev > 6.0 * noise_var), 1, m - 1))
    k_src = min(k_src, 2)
    e_n = vec[:, : m - k_src]
    a = _grid_steer(m)
    denom = np.sum(np.abs(e_n.conj().T @ a) ** 2, axis=0)
    spec = 1.0 / np.maximum(denom, 1e-12)
    interior = np.where(
        (spec[1:-1] > spec[:-2]) & (spec[1:-1] > spec[2:])
    )[0] + 1
    if len(interior) == 0:
        return float(GRID[int(np.argmax(spec))])
    top = interior[np.argsort(spec[interior])[::-1][:k_src]]
    if len(top) == 1:
        return float(GRID[top[0]])
    ths = GRID[top]
    a_full = np.exp(1j * np.pi * np.sin(ths)[None, :] * np.arange(n)[:, None])
    amps, *_ = np.linalg.lstsq(a_full, x, rcond=None)
    return float(ths[int(np.argmax(np.abs(amps)))])
