"""HTML reports: one per run (plots + metrics), and a summary per experiment."""
from __future__ import annotations

import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .analysis import analyze, strip_private  # noqa: E402
from .data import Run  # noqa: E402

CSS = """body{font-family:system-ui,sans-serif;margin:24px;max-width:1200px;color:#222}
table{border-collapse:collapse;margin:8px 0 18px}td,th{border:1px solid #ccc;padding:3px 8px;font-size:13px;text-align:left}
th{background:#f3f3f3}h1{font-size:22px}h2{font-size:17px;margin-top:28px}img{max-width:100%;border:1px solid #ddd}
.bad{color:#b00}.note{color:#666;font-size:13px}"""

METRIC_LABELS = {
    "p_acc_pre": ("Accumulator before fire", "psi"), "p_acc_post": ("Accumulator after", "psi"),
    "p_acc_drop": ("Accumulator drop", "psi"), "p_noz_peak": ("Nozzle peak", "psi"),
    "p_noz_plateau": ("Nozzle plateau", "psi"), "t_open_10_ms": ("Open 10 % (from fire)", "ms"),
    "t_open_50_ms": ("Open 50 %", "ms"), "t_open_90_ms": ("Open 90 %", "ms"),
    "t_close_90_ms": ("Close to 90 % (from release)", "ms"), "t_close_50_ms": ("Close to 50 %", "ms"),
    "t_close_10_ms": ("Close to 10 %", "ms"), "i_peak_max": ("Peak current max", "A"),
    "t_peak_90_ms": ("Peak current 90 % at", "ms"), "i_hold": ("Hold current", "A"), "d_hold": ("Hold duty", ""),
    "r_coil_est": ("Coil R (hold, assumes bus_voltage)", "ohm"), "tau_rise_ms": ("Rise time constant", "ms"),
    "l_coil_est": ("Coil L, armature open (est.)", "H"), "l_closed_est": ("Coil L, armature closed (est.)", "H"),
    "armature_ms": ("Armature starts moving", "ms"), "armature_end_ms": ("Armature motion ends", "ms"),
    "armature_i_a": ("Coil current when it starts", "A"), "t_dropout_seen_ms": ("Nozzle pressure falls at", "ms"),
    "i_at_pressure_fall": ("Hold current when pressure falls", "A"), "i_dropout_est": ("Dropout current (est.)", "A"),
    "temp_tc_start": ("Thermocouple start", "°C"), "temp_tc_min": ("Thermocouple min", "°C"),
    "temp_cj_start": ("Cold junction", "°C"), "temp_chip_start": ("RP2040 chip", "°C"),
    "missing_ms": ("Missing samples", "ms"),
}
JET_LABELS = {"penetration_max_mm": ("Penetration max (open)", "mm"), "front_speed_m_s": ("Front speed", "m/s"),
              "penetration_20ms_mm": ("Penetration at 20 ms", "mm"), "penetration_50ms_mm": ("Penetration at 50 ms", "mm"),
              "penetration_100ms_mm": ("Penetration at 100 ms", "mm"), "penetration_200ms_mm": ("Penetration at 200 ms", "mm"),
              "t_reach_90pct_field_ms": ("Front reaches 90 % of field", "ms"), "field_mm": ("Field along the jet", "mm")}


def _fmt(v):
    if v is None:
        return "–"
    if isinstance(v, float):
        if not np.isfinite(v):
            return "–"
        a = abs(v)
        return f"{v:.4g}" if a < 1000 else f"{v:.0f}"
    return html.escape(str(v))


def _table(rows, head=None):
    out = ["<table>"]
    if head:
        out.append("<tr>" + "".join(f"<th>{html.escape(h)}</th>" for h in head) + "</tr>")
    for r in rows:
        out.append("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>")
    out.append("</table>")
    return "\n".join(out)


def _vlines(ax, run: Run):
    for t, c, lab in ((run.t_fire, "tab:red", "fire"), (run.t_release, "tab:blue", "release")):
        if t is not None:
            ax.axvline(t, color=c, lw=0.8, ls="--", label=lab)


def run_report(run_dir: str | Path, close90_ms=None, video=True, log=print) -> dict:
    run_dir = Path(run_dir)
    res = analyze(run_dir, close90_ms=close90_ms, video=video, log=log)
    (run_dir / "analysis.json").write_text(json.dumps(strip_private(res), indent=2, default=float))
    run = Run(run_dir)
    imgs = []
    if run.has_samples:
        t = run.t
        # pressures
        fig, ax = plt.subplots(figsize=(10, 3.6))
        for n in run.pressure_names():
            ax.plot(t, run.psi(n), lw=0.8, label=n)
        _vlines(ax, run)
        ax.set_xlabel("ms from go"); ax.set_ylabel("psi"); ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.3)
        ax.set_title("Pressure")
        fig.tight_layout(); fig.savefig(run_dir / "pressure.png", dpi=110); plt.close(fig)
        imgs.append(("Pressure", "pressure.png"))
        # zoom on the valve timing
        if run.t_fire is not None and run.rig["nozzle_sensor"]:
            fig, axs = plt.subplots(1, 2, figsize=(10, 3.2), sharey=True)
            pn = run.psi(run.rig["nozzle_sensor"])
            for ax, t0, lab in ((axs[0], run.t_fire, "fire"), (axs[1], run.t_release, "release")):
                if t0 is None:
                    continue
                m = (t >= t0 - 20) & (t <= t0 + 120)
                ax.plot(t[m] - t0, pn[m], lw=1)
                ax.axvline(0, color="k", lw=0.6, ls="--")
                ax.set_xlabel(f"ms from {lab}"); ax.grid(alpha=0.3)
            axs[0].set_ylabel(f"{run.rig['nozzle_sensor']} psi")
            fig.suptitle("Valve opening and closing at the nozzle sensor", fontsize=10)
            fig.tight_layout(); fig.savefig(run_dir / "valve_timing.png", dpi=110); plt.close(fig)
            imgs.append(("Valve timing", "valve_timing.png"))
        # coil
        if run.rig["valve"] and run.t_fire is not None:
            tf = run.t_fire
            tr = run.t_release if run.t_release is not None else tf + 300
            fig, axs = plt.subplots(1, 2, figsize=(10, 3.4))
            q2 = run.sense_amps(run.rig["valve"]["adc"])
            for ax, (a0, a1) in zip(axs, ((tf - 5, tf + min(150, tr - tf + 40)), (tf - 20, tr + 60))):
                m = (t >= a0) & (t <= a1)
                mt = (run.tt >= a0) & (run.tt <= a1)
                ax.plot(t[m] - tf, q2[m], lw=0.7, color="0.6", label="Q2 current (sense)")
                ax.plot(run.tt[mt] - tf, run.valve_i[mt], lw=1.1, color="tab:red", label="coil current est.")
                ax2 = ax.twinx()
                ax2.plot(run.tt[mt] - tf, run.valve_d[mt], lw=0.8, color="tab:green", label="duty")
                ax2.set_ylim(0, 1.05); ax2.set_ylabel("duty", color="tab:green")
                ax.set_xlabel("ms from fire"); ax.set_ylabel("A"); ax.grid(alpha=0.3)
            axs[0].legend(fontsize=7, loc="lower right")
            axs[0].set_title("Peak", fontsize=10); axs[1].set_title("Whole fire", fontsize=10)
            fig.tight_layout(); fig.savefig(run_dir / "coil.png", dpi=110); plt.close(fig)
            imgs.append(("Coil current and duty", "coil.png"))
        # LED and temperatures
        fig, axs = plt.subplots(1, 2, figsize=(10, 3.0))
        if run.rig["led"]:
            axs[0].plot(run.tt, run.led_i, lw=0.8, label="LED current est.")
            axs[0].plot(run.tt, run.led_d * 4, lw=0.6, label="duty x 4")
            axs[0].set_ylabel("A"); axs[0].legend(fontsize=7)
        axs[0].set_title("Shadowgraph LED", fontsize=10); axs[0].set_xlabel("ms from go"); axs[0].grid(alpha=0.3)
        T = run.temperatures()
        for k, lab in (("tc", f"thermocouple ({(run.rig['thermocouple'] or {}).get('label', '')})"), ("cj", "cold junction"),
                       ("chip", "RP2040")):
            if np.isfinite(T[k]).any():
                axs[1].plot(T["t"], T[k], lw=1, label=lab)
        _vlines(axs[1], run)
        axs[1].set_title("Temperatures (10 Hz)", fontsize=10); axs[1].set_xlabel("ms from go"); axs[1].set_ylabel("°C")
        axs[1].legend(fontsize=7); axs[1].grid(alpha=0.3)
        fig.tight_layout(); fig.savefig(run_dir / "led_temp.png", dpi=110); plt.close(fig)
        imgs.append(("LED and temperatures", "led_temp.png"))
    vid = res.get("video", {})
    jet = vid.get("jet", {}) if vid else {}
    if jet.get("ok"):
        fig, ax = plt.subplots(figsize=(10, 3.2))
        ax.plot(jet["t_ms"], jet["penetration_mm"], ".-", ms=3, lw=0.8)
        ax.axvline(0, color="tab:red", ls="--", lw=0.8)
        if run.t_release is not None and run.t_fire is not None:
            ax.axvline(run.t_release - run.t_fire, color="tab:blue", ls="--", lw=0.8)
        ax.axhline(jet["field_mm"], color="0.5", lw=0.6, ls=":")
        ax.set_xlabel("ms from fire"); ax.set_ylabel("mm from nozzle"); ax.grid(alpha=0.3)
        ax.set_title("Jet penetration (first pass: check against the contact sheet)", fontsize=10)
        fig.tight_layout(); fig.savefig(run_dir / "penetration.png", dpi=110); plt.close(fig)
        imgs.append(("Jet penetration", "penetration.png"))
        cs = jet.get("_contact", [])
        if cs:
            n = len(cs)
            cols = 4
            rows = (n + cols - 1) // cols
            fig, axs = plt.subplots(rows, cols, figsize=(12, 2.2 * rows))
            for ax in np.ravel(axs):
                ax.axis("off")
            for ax, (tq, img) in zip(np.ravel(axs), cs):
                ax.imshow(img, cmap="gray", vmin=0.7, vmax=1.3)
                ax.set_title(f"{tq:.0f} ms", fontsize=8)
            fig.suptitle("Frame / background (0.7 to 1.3), ms from fire", fontsize=10)
            fig.tight_layout(); fig.savefig(run_dir / "contact.png", dpi=100); plt.close(fig)
            imgs.append(("Contact sheet", "contact.png"))

    # html
    meta = run.meta
    head = f"<h1>{html.escape(run_dir.name)}</h1>"
    if meta["status"] != "ok":
        head += f"<p class='bad'>Status: {html.escape(meta['status'])} ({html.escape(meta.get('why', ''))})</p>"
    cond = meta.get("meta", {})
    rows = [(html.escape(str(k)), _fmt(v)) for k, v in cond.items()]
    rows += [(html.escape(k), _fmt(v)) for k, v in meta["params"].items()]
    parts = [head, f"<p class='note'>{html.escape(meta['created'])} · device {html.escape(str(meta['device']['port']))}"
             f" · fw {html.escape(str(meta['device']['info'].get('fw')))}</p>",
             "<h2>Setup and parameters</h2>", _table(rows, ["", ""])]
    mrows = [(html.escape(lab), _fmt(res.get(k)), u) for k, (lab, u) in METRIC_LABELS.items() if k in res]
    for k, (lab, u) in JET_LABELS.items():
        if k in jet:
            mrows.append((html.escape(lab), _fmt(jet[k]), u))
    parts += ["<h2>Metrics</h2>", _table(mrows, ["metric", "value", "unit"])]
    notes = [res[k] for k in res if k.startswith("note")]
    if notes:
        parts.append("<p class='note'>" + "<br>".join(html.escape(n) for n in notes) + "</p>")
    sy = vid.get("sync") if vid else None
    if sy:
        srows = [(k, _fmt(v)) for k, v in sy.items()]
        parts += ["<h2>Video timing (signal-LED timecode)</h2>", _table(srows, ["", ""])]
        if not sy.get("ok"):
            parts.append(f"<p class='bad'>Sync failed: {html.escape(str(sy.get('why', '')))}</p>")
    elif vid and vid.get("error"):
        parts.append(f"<p class='bad'>Video: {html.escape(vid['error'])}</p>")
    for title, f in imgs:
        parts += [f"<h2>{html.escape(title)}</h2>", f"<img src='{f}'>"]
    (run_dir / "report.html").write_text(f"<!doctype html><meta charset='utf-8'><title>{html.escape(run_dir.name)}</title>"
                                         f"<style>{CSS}</style>" + "\n".join(parts))
    return res


SUMMARY_KEYS = ["p_acc_pre", "p_acc_drop", "p_noz_plateau", "p_noz_peak", "t_open_50_ms", "t_open_90_ms",
                "t_close_50_ms", "t_close_10_ms", "i_hold", "r_coil_est", "armature_ms"]
SUMMARY_JET = ["penetration_max_mm", "front_speed_m_s", "penetration_100ms_mm"]


def summary_report(exp_dir: str | Path, results: list[dict], sweep_keys: list[str], cond_keys: list[str]) -> Path:
    exp_dir = Path(exp_dir)
    flat = []
    for r in results:
        row = {"run": r["run"], "status": r["status"]}
        row.update({k: r.get("meta", {}).get(k) for k in cond_keys})
        row.update({k: r.get("params", {}).get(k) for k in sweep_keys})
        row.update({k: r.get(k) for k in SUMMARY_KEYS})
        jet = (r.get("video") or {}).get("jet") or {}
        row.update({k: jet.get(k) for k in SUMMARY_JET})
        flat.append(row)
    cols = ["run", "status"] + cond_keys + sweep_keys + SUMMARY_KEYS + SUMMARY_JET
    with open(exp_dir / "summary.csv", "w") as f:
        f.write(",".join(cols) + "\n")
        for row in flat:
            f.write(",".join("" if row.get(c) is None else str(row.get(c)) for c in cols) + "\n")
    # group statistics: condition x sweep values
    groups = {}
    for row in flat:
        if row["status"] != "ok":
            continue
        key = tuple(row.get(k) for k in cond_keys + sweep_keys)
        groups.setdefault(key, []).append(row)
    stat_keys = [k for k in SUMMARY_KEYS + SUMMARY_JET if any(isinstance(r.get(k), (int, float)) for r in flat)]
    grows = []
    for key, rows in groups.items():
        cells = [_fmt(v) for v in key] + [str(len(rows))]
        for k in stat_keys:
            v = np.array([r[k] for r in rows if isinstance(r.get(k), (int, float)) and np.isfinite(r[k])], float)
            cells.append("–" if not len(v) else (f"{v.mean():.4g}" + (f" ± {v.std(ddof=1):.2g}" if len(v) > 1 else "")))
        grows.append(cells)
    head = cond_keys + sweep_keys + ["n"] + [METRIC_LABELS.get(k, JET_LABELS.get(k, (k, "")))[0] for k in stat_keys]
    imgs = []
    for sk in sweep_keys:
        vals = sorted({row.get(sk) for row in flat if row.get(sk) is not None})
        if len(vals) < 2:
            continue
        for mk in [k for k in ("t_open_50_ms", "t_close_10_ms", "p_noz_plateau", "penetration_max_mm", "front_speed_m_s")
                   if k in stat_keys]:
            fig, ax = plt.subplots(figsize=(6, 3.4))
            conds = sorted({tuple(row.get(c) for c in cond_keys) for row in flat})
            for ci, c in enumerate(conds):
                xs, ys = [], []
                for row in flat:
                    if tuple(row.get(k) for k in cond_keys) == c and row["status"] == "ok" and isinstance(row.get(mk), (int, float)):
                        xs.append(row[sk]); ys.append(row[mk])
                if xs:
                    jitter = (ci - (len(conds) - 1) / 2) * 0.01 * (max(vals) - min(vals) or 1)
                    ax.plot(np.array(xs, float) + jitter, ys, "o", ms=4, alpha=0.7, label=" / ".join(str(v) for v in c))
            lab, unit = METRIC_LABELS.get(mk, JET_LABELS.get(mk, (mk, "")))
            ax.set_xlabel(sk); ax.set_ylabel(f"{lab} ({unit})" if unit else lab); ax.grid(alpha=0.3)
            ax.legend(fontsize=7)
            fig.tight_layout()
            fn = f"summary_{mk}_vs_{sk}.png"
            fig.savefig(exp_dir / fn, dpi=100); plt.close(fig)
            imgs.append(fn)
    run_rows = [[f"<a href='{html.escape(r['run'])}/report.html'>{html.escape(r['run'])}</a>",
                 html.escape(r["status"])] + [_fmt(r.get(c)) for c in cols[2:]] for r in flat]
    parts = [f"<h1>{html.escape(exp_dir.name)}</h1>", "<h2>By condition</h2>", _table(grows, head),
             "<p class='note'>mean ± sample standard deviation over repeats with status ok</p>"]
    parts += [f"<img src='{f}'>" for f in imgs]
    parts += ["<h2>All runs</h2>", _table(run_rows, cols)]
    out = exp_dir / "summary.html"
    out.write_text(f"<!doctype html><meta charset='utf-8'><title>{html.escape(exp_dir.name)}</title><style>{CSS}</style>"
                   + "\n".join(parts))
    return out
