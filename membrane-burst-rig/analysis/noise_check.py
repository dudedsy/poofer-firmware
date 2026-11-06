#!/usr/bin/env python3
"""noise_check.py - steady-pressure noise and update-rate check on burst rig captures.

Take forced captures (UP button, or `t` in Live) with the rig vented or at a
steady pressure, one tag or run label per setup. Then:

    python analysis/noise_check.py "runs/*" --plots noise_plots

For each capture and channel it prints:
    rms, pk-pk      after removing the mean, in ADC counts (~33 counts/psi at 3.3 V)
    60/120/180 Hz   amplitude of each mains harmonic (least-squares fit, counts peak)
    peaks           strongest spectral lines above 200 Hz (Hz:amplitude)
    step period     estimate of the sensor's internal update rate from the
                    autocorrelation of the sample-to-sample differences (blank if none)

The step-period estimate only works if the steps stand out above the ADC noise.
Check it against the zoomed trace in the plots.
"""
import argparse
import glob
import json
import os

import numpy as np

MAINS = (60.0, 120.0, 180.0)


def mains_amplitudes(x, fs):
    t = np.arange(len(x)) / fs
    cols = [np.ones_like(t)]
    for f in MAINS:
        cols += [np.sin(2 * np.pi * f * t), np.cos(2 * np.pi * f * t)]
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    return [float(np.hypot(coef[1 + 2 * i], coef[2 + 2 * i])) for i in range(len(MAINS))]


def spectrum(x, fs):
    w = np.hanning(len(x))
    X = np.fft.rfft((x - x.mean()) * w)
    amp = 2 * np.abs(X) / w.sum()          # peak amplitude of a sine, in counts
    f = np.fft.rfftfreq(len(x), 1 / fs)
    return f, amp


def top_peaks(f, amp, fmin=200.0, n=6):
    m = f >= fmin
    fi, ai = f[m], amp[m]
    # local maxima only
    is_pk = np.r_[False, (ai[1:-1] > ai[:-2]) & (ai[1:-1] >= ai[2:]), False]
    idx = np.where(is_pk)[0]
    idx = idx[np.argsort(ai[idx])[::-1][:n]]
    return [(float(fi[i]), float(ai[i])) for i in sorted(idx, key=lambda i: -ai[i])]


def _step_ac(x, fs, k, max_lag_s):
    """Autocorrelation of a k-sample step detector's energy, and the first strong lag."""
    c = np.cumsum(np.r_[0.0, x.astype(float)])
    i = np.arange(k, len(x) - k + 1)
    s = (c[i + k] - c[i]) / k - (c[i] - c[i - k]) / k     # mean after - mean before
    # remove slow trends (hum, pressure drift) that bias s; a step's 2k-wide blip survives
    w = 8 * k + 1
    if len(s) > 2 * w:
        cs = np.cumsum(np.r_[0.0, s])
        ma = (cs[w:] - cs[:-w]) / w
        s = s[w // 2: w // 2 + len(ma)] - ma
    e = s ** 2
    e -= e.mean()
    n = len(e)
    if n < 8 or not e.any():
        return None, 0.0
    nfft = 1 << (2 * n - 1).bit_length()
    E = np.fft.rfft(e, nfft)
    ac = np.fft.irfft(E * np.conj(E), nfft)[:n]
    ac /= ac[0]
    lo, hi = 2 * k + 1, min(n // 2, int(max_lag_s * fs))
    if hi <= lo:
        return None, 0.0
    seg = ac[lo:hi]
    best = float(seg.max())
    is_pk = np.r_[False, (seg[1:-1] > seg[:-2]) & (seg[1:-1] >= seg[2:]), False]
    cand = np.where(is_pk & (seg >= 0.6 * best))[0]
    j = int(cand[0] if len(cand) else np.argmax(seg))
    lag = lo + j
    frac = 0.0
    if 0 < j < len(seg) - 1:                               # parabolic sub-sample refinement
        a, b, c2 = seg[j - 1], seg[j], seg[j + 1]
        den = a - 2 * b + c2
        frac = 0.5 * (a - c2) / den if den else 0.0
    return lag + frac, float(ac[lag])


def step_period(x, fs, max_lag_s=0.025, ks=(1, 4, 16, 64)):
    """Return (period_s, strength) of the sensor's output update, or (None, best r).

    A sensor that updates its output every P seconds puts a jump of random size and
    sign every P. A step detector (mean of k samples after minus k before) squared
    gives a spike train at P, which peaks in its autocorrelation at lag P. Several k
    are tried: larger k averages down ADC noise but cannot see periods under ~2k
    samples. The first lag within 60% of the best peak is taken, so a multiple of P
    is not reported instead of P.
    """
    best = (None, 0.0)
    for k in ks:
        lag, r = _step_ac(x, fs, k, max_lag_s)
        if lag is not None and r > best[1]:
            best = (lag, r)
    lag, r = best
    # with no periodic steps, r at the best lag is a few percent
    if lag is None or r < 0.15:
        return None, r
    return lag / fs, r


def analyse(npz_path, plot_dir=None):
    counts = np.load(npz_path)["counts"].astype(float)
    with open(npz_path[:-4] + ".json") as fh:
        meta = json.load(fh)
    fs = float(meta["fs_ch"])
    label = meta.get("label", os.path.basename(os.path.dirname(npz_path)))
    rows = []
    for ch in range(counts.shape[1]):
        x = counts[:, ch]
        xc = x - x.mean()
        m60, m120, m180 = mains_amplitudes(x, fs)
        f, amp = spectrum(x, fs)
        per, r = step_period(x, fs)
        rows.append(dict(file=npz_path, label=label, ch=ch, fs=fs, mean=float(x.mean()),
                         rms=float(xc.std()), pkpk=float(xc.max() - xc.min()),
                         m60=m60, m120=m120, m180=m180, peaks=top_peaks(f, amp),
                         step_s=per, step_r=r))
    if plot_dir:
        plot(counts, fs, npz_path, label, rows, plot_dir)
    return rows


def plot(counts, fs, npz_path, label, rows, plot_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(plot_dir, exist_ok=True)
    n_ch = counts.shape[1]
    t_ms = np.arange(len(counts)) / fs * 1e3
    fig, ax = plt.subplots(n_ch, 3, figsize=(15, 3.2 * n_ch), squeeze=False)
    for ch in range(n_ch):
        x = counts[:, ch]
        ax[ch, 0].plot(t_ms, x, lw=0.4)
        ax[ch, 0].set(title=f"{label} ch{ch}: full capture", xlabel="ms", ylabel="counts")
        z = t_ms < 10.0
        ax[ch, 1].step(t_ms[z], x[z], where="post", lw=0.8)
        ax[ch, 1].set(title="first 10 ms (look for steps)", xlabel="ms")
        f, amp = spectrum(x, fs)
        ax[ch, 2].loglog(f[1:], amp[1:], lw=0.5)
        for m in MAINS:
            ax[ch, 2].axvline(m, color="r", lw=0.4, alpha=0.5)
        ax[ch, 2].set(title="spectrum (red = 60/120/180 Hz)", xlabel="Hz", ylabel="counts peak")
    fig.tight_layout()
    name = os.path.basename(os.path.dirname(npz_path)) + "_" + os.path.basename(npz_path)[:-4] + ".png"
    fig.savefig(os.path.join(plot_dir, name), dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="run folders or globs, e.g. \"runs/*\"")
    ap.add_argument("--plots", help="folder for one PNG per capture")
    ap.add_argument("--csv", help="also write the table to this CSV")
    args = ap.parse_args()

    files = []
    for pat in args.runs:
        for d in sorted(glob.glob(pat)):
            files += sorted(glob.glob(os.path.join(d, "capture_*.npz"))) if os.path.isdir(d) else [d]
    if not files:
        raise SystemExit("no capture_*.npz found")

    rows = [r for p in files for r in analyse(p, args.plots)]
    hdr = f"{'label':24s} {'capture':12s} ch {'mean':>7s} {'rms':>6s} {'pkpk':>5s} " \
          f"{'60Hz':>6s} {'120Hz':>6s} {'180Hz':>6s}  {'step':>10s}  peaks >200 Hz (Hz:counts)"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        step = f"{1 / r['step_s']:7.0f} Hz" if r["step_s"] else f"{'-':>10s}"
        pk = " ".join(f"{f:.0f}:{a:.2f}" for f, a in r["peaks"][:4])
        cap = os.path.basename(r["file"])[:-4].replace("capture_", "#")
        print(f"{r['label'][:24]:24s} {cap:12s} {r['ch']:2d} {r['mean']:7.1f} {r['rms']:6.2f} {r['pkpk']:5.0f} "
              f"{r['m60']:6.2f} {r['m120']:6.2f} {r['m180']:6.2f}  {step}  {pk}")
    print("\ncounts are 12-bit ADC counts; ~33 counts per psi for a 0-100 psi sensor on 3.3 V.")
    if args.csv:
        import csv
        with open(args.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["file", "label", "ch", "fs", "mean", "rms", "pkpk", "a60", "a120", "a180",
                        "step_hz", "step_r", "peaks"])
            for r in rows:
                w.writerow([r["file"], r["label"], r["ch"], r["fs"], r["mean"], r["rms"], r["pkpk"],
                            r["m60"], r["m120"], r["m180"], 1 / r["step_s"] if r["step_s"] else "",
                            r["step_r"], ";".join(f"{f:.1f}:{a:.3f}" for f, a in r["peaks"])])


if __name__ == "__main__":
    main()
