"""Metrics for one run: valve timing from the nozzle pressure, coil electrical behaviour,
accumulator use, temperatures, and (with video) the frame timing and jet penetration.

Times are ms relative to 'go' unless a name says otherwise ('*_ms' delays are from fire or
release). Every metric is NaN / None when the data for it is missing, never guessed.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .camera import read_gray
from .data import Run

NAN = float("nan")


def _crossing(t, y, level, t_from, rising=True):
    """First time >= t_from where y crosses level (linear interpolation)."""
    i0 = int(np.searchsorted(t, t_from))
    yy = y[i0:]
    hit = np.nonzero(yy >= level if rising else yy <= level)[0]
    if not len(hit):
        return NAN
    k = i0 + hit[0]
    if k == 0:
        return float(t[0])
    y0, y1 = y[k - 1], y[k]
    f = 0.0 if y1 == y0 else (level - y0) / (y1 - y0)
    return float(t[k - 1] + f * (t[k] - t[k - 1]))


def _smooth(y, n):
    if n <= 1:
        return y
    k = np.ones(n) / n
    return np.convolve(y, k, mode="same")


def lfsr_bits(n: int, seed: int = 0x1FF) -> np.ndarray:
    """Signal-LED timecode, as rig_core.cpp: output bit 0, feedback bit0 ^ bit4 into bit 8."""
    s, out = seed, np.empty(n, np.uint8)
    for k in range(n):
        out[k] = s & 1
        fb = (s ^ (s >> 4)) & 1
        s = (s >> 1) | (fb << 8)
    return out


# ---------------------------------------------------------------- pressure and valve
def pressure_metrics(run: Run) -> dict:
    m = {}
    tf, tr = run.t_fire, run.t_release
    acc, noz = run.rig["accumulator_sensor"], run.rig["nozzle_sensor"]
    t = run.t
    if acc:
        pa = run.psi(acc)
        ref = tf if tf is not None else 0.0
        m["p_acc_pre"] = run.mean(pa, t, ref - 200, ref - 20)
        # just after the valve has closed: later the gas warms back up (and a fill may be open)
        post0 = (tr + 200) if tr is not None else t[-1] - 150
        m["p_acc_post"] = run.mean(pa, t, post0, post0 + 100)
        m["p_acc_drop"] = m["p_acc_pre"] - m["p_acc_post"]
    if noz and tf is not None and tr is not None:
        pn = _smooth(run.psi(noz), 3)
        base = run.mean(pn, t, tf - 200, tf - 20)
        win = (t >= tf) & (t <= tr + 200)
        peak = float(pn[win].max()) if win.any() else NAN
        open_ms = tr - tf
        plat = NAN
        if open_ms >= 60:
            mm = (t >= tf + 0.5 * open_ms) & (t <= tr)
            plat = float(np.median(pn[mm])) if mm.any() else NAN
        ref = plat if np.isfinite(plat) else peak
        m.update({"p_noz_base": base, "p_noz_peak": peak, "p_noz_plateau": plat})
        span = ref - base
        if np.isfinite(span) and span > 1.0:
            for f in (10, 50, 90):
                m[f"t_open_{f}_ms"] = _crossing(t, pn, base + f / 100 * span, tf) - tf
            for f in (90, 50, 10):
                m[f"t_close_{f}_ms"] = _crossing(t, pn, base + f / 100 * span, tr, rising=False) - tr
        else:
            m["note_pressure"] = "nozzle pressure did not rise by more than 1 psi"
    return m


def valve_metrics(run: Run, close90_ms: float | None = None) -> dict:
    """Coil behaviour from telemetry (1 kHz) and the raw Q2 current sense (5 kHz)."""
    m = {}
    tf, tr = run.t_fire, run.t_release
    if tf is None or not run.rig["valve"]:
        return m
    vcfg = run.meta.get("device", {}).get("info", {}).get("valve", {})
    vbus = float(run.rig["bus_voltage"])
    r_path = float(run.rig["valve"].get("r_path", 0.15))
    v_free = float(run.rig["valve"].get("v_free", 0.8))
    tt, i, d = run.tt, run.valve_i, run.valve_d
    st = run.valve_state
    peak_target = float(vcfg.get("peak", NAN))
    pk = (st == "PEAK")
    if pk.any():
        m["i_peak_max"] = float(i[pk].max())
        if np.isfinite(peak_target):
            m["t_peak_90_ms"] = _crossing(tt, i, 0.9 * peak_target, tf) - tf
    hold = (st == "HOLD") & (tt > (tt[pk][-1] + 50 if pk.any() else tf + 150))
    ramp = run.params.get("ramp", 0) or 0
    if ramp:
        hold &= tt < tf + run.params.get("ramp_at", 300)
    if hold.sum() > 20:
        ok = hold & (i > 0.2)
        r = (d[ok] * vbus - (1 - d[ok]) * v_free) / i[ok] - r_path
        m["r_coil_est"] = float(np.median(r)) if ok.any() else NAN
        m["i_hold"] = float(np.median(i[hold]))
        m["d_hold"] = float(np.median(d[hold]))
    # Rise: undo the 1 ms sense RC (i = s + tau ds/dt) to get the coil current, then the
    # inductance the rise implies, L_eff = (V - R i) / (di/dt). It reads L0 (armature open)
    # first, jumps while the armature moves (its back EMF adds to L di/dt), and settles at the
    # closed-armature value. Window: until the current reaches 90 % of what the duty can push.
    line = run.rig["valve"]["adc"]
    q2 = run.sense_amps(line)
    t = run.t
    r_tot = (m["r_coil_est"] if np.isfinite(m.get("r_coil_est", NAN)) else float(vcfg.get("r", 8.5))) + r_path
    m["armature_ms"] = NAN
    w = (t >= tf) & (t <= tf + 120)
    if w.sum() > 50 and np.isfinite(r_tot):
        ts = t[w] - tf
        dd = np.interp(t[w], tt, d)
        sf = _smooth(q2[w], 5)
        dt_ = np.gradient(ts)
        i_coil = (sf + 1.0 * np.gradient(sf) / dt_) / np.maximum(dd, 0.05)
        i_coil = _smooth(i_coil, 5)
        v_app = dd * vbus - (1 - dd) * v_free
        lim = 0.9 * np.max(v_app) / r_tot
        k_end = int(np.argmax(i_coil >= lim)) if (i_coil >= lim).any() else len(i_coil)
        k_end = min(k_end, int(np.argmax(dd < 0.9)) if (dd < 0.9).any() else len(dd))
        di = np.gradient(i_coil) / dt_ * 1000.0                      # A/s
        with np.errstate(divide="ignore", invalid="ignore"):
            leff = (v_app - r_tot * i_coil) / di
        k0 = int(np.searchsorted(ts, 1.5))
        k1 = int(np.searchsorted(ts, 3.0))
        if k_end > k1 + 10:
            l0 = float(np.median(leff[k0:k1]))
            m["l_coil_est"] = l0
            m["tau_rise_ms"] = l0 / r_tot * 1000.0
            seg = leff[k1:k_end]
            fin = np.isfinite(seg) & (seg > 0)
            half = seg[len(seg) // 2:][fin[len(seg) // 2:]]
            l1 = float(np.median(half)) if len(half) >= 5 else NAN
            thr_l = max(1.4 * l0, 1.25 * l1) if np.isfinite(l1) else 1.4 * l0
            high = np.nonzero((np.isfinite(seg) & (seg > thr_l)) | (np.isfinite(seg) & (seg < 0)))[0]
            if len(high) >= 3:
                end = high[0]
                for h_ in high[1:]:                      # first contiguous region, gaps <= 3 samples
                    if h_ - end > 4:
                        break
                    end = h_
                m["armature_ms"] = float(ts[k1 + high[0]])                # motion starts
                m["armature_end_ms"] = float(ts[k1 + end])
                m["armature_i_a"] = float(i_coil[k1 + high[0]])
                if np.isfinite(l1) and k1 + end < k1 + len(seg) // 2:
                    m["l_closed_est"] = l1

    # dropout (hold ramp): nozzle pressure starts falling while the hold current ramps down
    if ramp and run.rig["nozzle_sensor"]:
        # nozzle / accumulator ratio: the accumulator draining lowers both, the valve closing
        # only the nozzle
        pn = _smooth(run.psi(run.rig["nozzle_sensor"]), 5)
        acc = run.rig["accumulator_sensor"]
        pa = _smooth(run.psi(acc), 5) if acc else np.ones_like(pn)
        t_ramp = tf + run.params.get("ramp_at", 300)
        base = run.mean(pn, t, tf - 200, tf - 20)
        ratio = (pn - base) / np.maximum(pa, 1.0)
        ref = run.mean(ratio, t, t_ramp - 100, t_ramp)
        if np.isfinite(ref) and ref > 0.02:
            t_fall = _crossing(t, ratio, 0.85 * ref, t_ramp, rising=False)
            if np.isfinite(t_fall) and (tr is None or t_fall < tr):
                m["t_dropout_seen_ms"] = t_fall - tf
                m["i_at_pressure_fall"] = float(np.interp(t_fall, tt, i))
                if close90_ms is not None and np.isfinite(close90_ms):
                    m["i_dropout_est"] = float(np.interp(t_fall - close90_ms, tt, i))
            else:
                m["note_dropout"] = "pressure did not fall during the ramp: dropout below hold_min, or ramp too short"
    return m


def temperature_metrics(run: Run) -> dict:
    T = run.temperatures()
    m = {}
    for k in ("chip", "cj", "tc"):
        v = T[k]
        if np.isfinite(v).any():
            m[f"temp_{k}_start"] = float(v[np.isfinite(v)][0])
            m[f"temp_{k}_min"] = float(np.nanmin(v))
            m[f"temp_{k}_max"] = float(np.nanmax(v))
    return m


# ---------------------------------------------------------------- video timing
def expected_sync(run: Run, t_grid):
    """Signal-LED state (0/1) the firmware drove at times t_grid (ms rel. go); -1 outside."""
    e = run.ev("tc_start")
    if e is None:
        return None
    t0 = e["t"] - run.go
    cell = e["cell"]
    t_end = run.t_end if run.t_end is not None else run.tt[-1]
    n = int((t_end - t0) / cell) + 2
    bits = lfsr_bits(max(n, 1), e.get("seed", 0x1FF))
    k = np.floor((t_grid - t0) / cell).astype(int)
    out = np.full(len(t_grid), -1, int)
    ok = (k >= 0) & (t_grid < t_end)
    out[ok] = bits[np.clip(k[ok], 0, len(bits) - 1)]
    return out, t0, cell, t_end


def find_sync_roi(frames, jet_mask=None, block=8):
    """The block that changes most before the jet arrives: the signal LED."""
    n = min(len(frames), 200)
    f = frames[:n].astype(np.float32)
    s = f.std(axis=0)
    if jet_mask is not None:
        s = np.where(jet_mask, 0, s)
    h, w = s.shape
    hb, wb = h // block, w // block
    sb = s[:hb * block, :wb * block].reshape(hb, block, wb, block).mean(axis=(1, 3))
    y, x = np.unravel_index(int(np.argmax(sb)), sb.shape)
    return [x * block, y * block, (x + 1) * block, (y + 1) * block]


def sync_video(run: Run, frames, scale: float, fps_nominal: float, roi=None) -> dict:
    """Frame k is at t_k = a + b k (ms rel. go). Coarse (a, b) by correlating the thresholded
    LED brightness against the known timecode over all offsets (FFT) for b within +-3 % of the
    nominal period, then refined by least squares on every matched on/off edge."""
    if roi is None:
        roi_s = find_sync_roi(frames)
    else:
        roi_s = [int(round(v * scale)) for v in roi]
    x0, y0, x1, y1 = roi_s
    B = frames[:, y0:y1, x0:x1].reshape(len(frames), -1).mean(axis=1).astype(np.float64)
    lo, hi = np.percentile(B, 5), np.percentile(B, 95)
    if hi - lo < 10:
        return {"ok": False, "why": f"signal LED not found (brightness range {hi - lo:.1f} in roi {roi_s})"}
    bit = (B > 0.5 * (lo + hi)).astype(int)
    exp = expected_sync(run, np.array([0.0]))
    if exp is None:
        return {"ok": False, "why": "no tc_start event (timecode off?)"}
    _, t0, cell, t_end = exp
    res = 0.5                                   # ms grid
    span_lo, span_hi = t0 - 20000.0, t_end + 1000.0
    grid = np.arange(span_lo, span_hi, res)
    e = expected_sync(run, grid)[0]
    ev = np.where(e < 0, 0.0, 2.0 * e - 1.0)
    obs = 2.0 * bit - 1.0
    b0 = 1000.0 / fps_nominal
    best = (-np.inf, None, None)
    nfft = 1 << int(np.ceil(np.log2(len(grid) + len(frames) * b0 * 1.03 / res + 16)))
    Fe = np.fft.rfft(ev, nfft)
    for b in b0 * np.linspace(0.97, 1.03, 121):
        pos = np.round(np.arange(len(frames)) * b / res).astype(int)
        o = np.zeros(nfft)
        o[pos[pos < nfft]] = obs[pos < nfft]
        c = np.fft.irfft(Fe * np.conj(np.fft.rfft(o, nfft)), nfft)   # c[s] = sum ev[s + pos] obs
        s = int(np.argmax(c[:len(grid)]))
        if c[s] > best[0]:
            best = (c[s], b, span_lo + s * res)
    score, b, a = best
    # refine on edges: observed transition between frames k and k+1 <-> nearest expected edge
    edges_exp = t0 + cell * np.nonzero(np.diff(lfsr_bits(int((t_end - t0) / cell) + 2)))[0] + cell
    for _ in range(3):
        ks = np.nonzero(np.diff(bit))[0]
        tm = a + b * (ks + 0.5)
        j = np.clip(np.searchsorted(edges_exp, tm), 1, len(edges_exp) - 1)
        near = np.where(np.abs(edges_exp[j] - tm) < np.abs(edges_exp[j - 1] - tm), edges_exp[j], edges_exp[j - 1])
        ok = np.abs(near - tm) < 0.75 * b
        if ok.sum() < 6:
            break
        A = np.vstack([np.ones(ok.sum()), ks[ok] + 0.5]).T
        (a, b), *_ = np.linalg.lstsq(A, near[ok], rcond=None)
    resid = near[ok] - (a + b * (ks[ok] + 0.5)) if ok.sum() else np.array([])
    agree = float(np.mean(bit == np.where(expected_sync(run, a + b * np.arange(len(frames)))[0] == 1, 1, 0)))
    jumps = int(np.sum(np.abs(np.diff(resid)) > 0.9 * b)) if len(resid) > 1 else 0
    return {"ok": bool(ok.sum() >= 6 and agree > 0.9), "a_ms": float(a), "period_ms": float(b), "fps": 1000.0 / b,
            "edges": int(ok.sum()), "resid_rms_ms": float(np.sqrt(np.mean(resid ** 2))) if len(resid) else NAN,
            "bit_agreement": agree, "suspect_drops": jumps, "roi": [int(round(v / scale)) for v in roi_s],
            "n_frames": int(len(frames))}


# ---------------------------------------------------------------- jet
def _box(x, n=5):
    c = np.cumsum(np.cumsum(np.pad(x, ((1, 0), (1, 0))), 0), 1)
    r = n // 2
    H, W = x.shape
    ys0, ys1 = np.clip(np.arange(H) - r, 0, H), np.clip(np.arange(H) + r + 1, 0, H)
    xs0, xs1 = np.clip(np.arange(W) - r, 0, W), np.clip(np.arange(W) + r + 1, 0, W)
    s = c[ys1][:, xs1] - c[ys0][:, xs1] - c[ys1][:, xs0] + c[ys0][:, xs0]
    return s / ((ys1 - ys0)[:, None] * (xs1 - xs0)[None, :])


def jet_analysis(run: Run, frames, sync: dict, scale: float, vcfg: dict, sync_roi_full, n_contact: int = 12) -> dict:
    """Penetration vs time: each frame, normalised to its own mean brightness, against the
    median of the frames just before fire; pixels whose smoothed difference exceeds the
    background noise count as jet; penetration is the farthest column (along the jet axis)
    with more than 3 % of the jet region disturbed, without a gap over 10 % of the field.
    First pass: check it against the contact sheet before trusting it."""
    if not sync.get("ok") or run.t_fire is None:
        return {"ok": False}
    a, b = sync["a_ms"], sync["period_ms"]
    tk = a + b * np.arange(len(frames))
    tf, tr = run.t_fire, run.t_release
    bgm = np.nonzero((tk > tf - 150) & (tk < tf - 5))[0]
    if len(bgm) < 5:
        return {"ok": False, "why": "fewer than 5 frames before fire for the background"}
    h, w = frames.shape[1:]
    mask = np.ones((h, w), bool)
    if sync_roi_full:
        x0, y0, x1, y1 = [int(round(v * scale)) for v in sync_roi_full]
        mask[max(0, y0 - 4):y1 + 4, max(0, x0 - 4):x1 + 4] = False
    jet = np.zeros((h, w), bool)
    jr = vcfg.get("jet_roi")
    if jr:
        x0, y0, x1, y1 = [int(round(v * scale)) for v in jr]
        jet[y0:y1, x0:x1] = True
    else:
        jet[:] = True
    jet &= mask

    def norm(k):
        f = frames[k].astype(np.float32)
        return f / max(float(f[mask].mean()), 1.0)

    bg = np.median(np.stack([norm(k) for k in bgm[-30:]]), axis=0)
    bgd = np.stack([_box(np.abs(norm(k) - bg))[jet] for k in bgm[-30:]])
    med = float(np.median(bgd))
    thr = max(med + 6.0 * 1.4826 * float(np.median(np.abs(bgd - med))), 1e-3)

    direction = vcfg.get("direction", "+x")
    noz = vcfg.get("nozzle_px")
    nx, ny = (noz[0] * scale, noz[1] * scale) if noz else (0.0, h / 2.0)
    axis = 1 if "x" in direction else 0
    sign = 1 if direction.startswith("+") else -1
    L = w if axis == 1 else h
    origin = nx if axis == 1 else ny
    dist = (np.arange(L) - origin) * sign
    order = np.argsort(dist)
    jet_frac = jet.mean(axis=0 if axis == 1 else 1)
    t_last = (tr if tr is not None else tf) + 500
    contact_t = tf + np.linspace(0, (tr - tf if tr is not None else 300) + 200, n_contact)
    contact_k = sorted({int(np.argmin(np.abs(tk - t))) for t in contact_t})
    pen = np.full(len(frames), np.nan)
    contact = []
    for k in range(len(frames)):
        if tk[k] < tf - 20 or tk[k] > t_last:
            continue
        fn = norm(k)
        dmap = _box(np.abs(fn - bg))
        if k in contact_k:
            contact.append((float(tk[k] - tf), fn / np.maximum(bg, 1e-3)))
        dis = (dmap > thr) & jet
        prof = dis.mean(axis=0 if axis == 1 else 1) / np.maximum(jet_frac, 1e-6)
        on = (prof > 0.03) & (dist >= 0)
        last, gap = None, 0
        for dd, oo in zip(dist[order], on[order]):
            if dd < 0:
                continue
            if oo:
                last, gap = dd, 0
            else:
                gap += 1
                if last is not None and gap > 0.1 * L:
                    break
        pen[k] = float(last) if last is not None else 0.0
    mmpp = float(vcfg.get("mm_per_px", 1.0)) / scale
    pen_mm = pen * mmpp
    keep = np.isfinite(pen_mm)
    out = {"ok": True, "t_ms": (tk[keep] - tf).tolist(), "penetration_mm": pen_mm[keep].tolist(),
           "threshold": thr, "mm_per_px_analysis": mmpp}
    field = (L - 1 - origin if sign > 0 else origin) * mmpp
    out["field_mm"] = float(field)
    open_m = keep & (tk >= tf) & ((tk <= tr) if tr is not None else True)
    if open_m.any():
        out["penetration_max_mm"] = float(np.nanmax(pen_mm[open_m]))
        rise = open_m & (pen_mm > 0.15 * field) & (pen_mm < 0.75 * field)
        if rise.sum() >= 3:
            out["front_speed_m_s"] = float(np.polyfit(tk[rise], pen_mm[rise], 1)[0])   # mm/ms = m/s
        for tq in (20, 50, 100, 200):
            kk = int(np.argmin(np.abs(tk - (tf + tq))))
            if abs(tk[kk] - (tf + tq)) < b and keep[kk]:
                out[f"penetration_{tq}ms_mm"] = float(pen_mm[kk])
        reach = np.nonzero(open_m & (pen_mm >= 0.9 * field))[0]
        if len(reach):
            out["t_reach_90pct_field_ms"] = float(tk[reach[0]] - tf)
    out["_contact"] = contact
    return out


def analyze(run_dir: str | Path, close90_ms: float | None = None, video: bool = True, log=print) -> dict:
    run = Run(run_dir)
    res = {"run": run.dir.name, "status": run.meta["status"], "params": run.params, "meta": run.meta.get("meta", {})}
    if not run.has_samples:
        res["error"] = "no samples recorded"
        return res
    res["missing_ms"] = int((~run.valid).sum())
    res.update(pressure_metrics(run))
    res.update(valve_metrics(run, close90_ms))
    res.update(temperature_metrics(run))
    vfile = run.meta.get("video")
    res["video"] = {"ok": False}
    if video and vfile and (run.dir / vfile).exists():
        vcfg = run.rig["video"]
        cam = run.rig["camera"]
        scale = float(vcfg.get("analysis_scale", 0.5))
        try:
            frames, full = read_gray(run.dir / vfile, scale, cam.get("ffmpeg", "ffmpeg"))
            fps = float(cam.get("fps_real") or cam["fps"])
            sy = sync_video(run, frames, scale, fps, vcfg.get("sync_roi"))
            truth = run.dir / "video_truth.json"
            if truth.exists():
                tt = json.loads(truth.read_text())
                sy["truth_a_ms"] = tt["t_frame0_ms"] - run.go
                sy["truth_period_ms"] = tt["period_ms"]
            res["video"] = {"sync": sy, "full_size": list(full)}
            if sy.get("ok"):
                jet = jet_analysis(run, frames, sy, scale, vcfg, sy.get("roi"), int(vcfg.get("contact_frames", 12)))
                res["video"]["jet"] = jet
                res["video"]["ok"] = True
            else:
                log(f"  video sync failed: {sy.get('why', sy)}")
        except Exception as e:      # video problems never lose the pressure analysis
            res["video"] = {"ok": False, "error": f"{type(e).__name__}: {e}"}
            log(f"  video analysis failed: {e}")
    return res


def strip_private(res: dict) -> dict:
    """Drop arrays kept for plotting (keys starting with '_') before writing JSON."""
    if isinstance(res, dict):
        return {k: strip_private(v) for k, v in res.items() if not str(k).startswith("_")}
    if isinstance(res, list):
        return [strip_private(v) for v in res]
    if isinstance(res, float) and not np.isfinite(res):
        return None
    if isinstance(res, (np.floating,)):
        return float(res)
    if isinstance(res, (np.integer,)):
        return int(res)
    return res
