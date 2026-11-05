"""Experiments: a YAML file of hardware conditions x parameter sweeps x repeats, run one shot
at a time with operator prompts for hardware swaps and accumulator fills.

    name: nozzle-screening
    valve: uss2-34            # valves/<name>.json from `rigctl characterize`, optional
    defaults: {open_ms: 300, pressure_psi: 60, pressure_tol: 2}
    conditions:               # hardware set-ups; the operator is asked to swap between them
      - {nozzle: N1-straight, diffuser: none}
      - {nozzle: N2-taper, diffuser: honeycomb}
    sweep:                    # every combination, inside each condition
      open_ms: [100, 300]
      pressure_psi: [40, 60]
    repeats: 3
    order: grouped            # grouped (as listed) or shuffled (shuffled within each condition)
    seed: 1
"""
from __future__ import annotations

import datetime as dt
import itertools
import json
import random
import re
import sys
import time
from pathlib import Path

import yaml

from .config import Rig
from .link import Device, DeviceError
from .report import run_report, summary_report
from .shot import PARAM_DEFAULTS, ShotError, run_shot


class Quit(Exception):
    pass


def slug(s) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "-", str(s)).strip("-") or "x"


def load(path: str | Path) -> dict:
    e = yaml.safe_load(Path(path).read_text()) or {}
    e.setdefault("name", Path(path).stem)
    e.setdefault("defaults", {})
    e.setdefault("conditions", [{}])
    e.setdefault("sweep", {})
    e.setdefault("repeats", 1)
    e.setdefault("order", "grouped")
    bad = (set(e["defaults"]) | set(e["sweep"])) - set(PARAM_DEFAULTS)
    if bad:
        raise ValueError(f"{path}: unknown parameter(s) {sorted(bad)}; known: {sorted(PARAM_DEFAULTS)}")
    for k, v in e["sweep"].items():
        if not isinstance(v, list) or not v:
            raise ValueError(f"{path}: sweep.{k} must be a non-empty list")
    if e["order"] not in ("grouped", "shuffled"):
        raise ValueError(f"{path}: order must be grouped or shuffled")
    return e


def plan(e: dict) -> list[dict]:
    keys = list(e["sweep"])
    combos = list(itertools.product(*[e["sweep"][k] for k in keys])) or [()]
    rng = random.Random(e.get("seed", 1))
    runs, n = [], 0
    for ci, cond in enumerate(e["conditions"]):
        group = []
        for combo in combos:
            for r in range(1, int(e["repeats"]) + 1):
                params = dict(e["defaults"])
                params.update(dict(zip(keys, combo)))
                group.append({"condition": cond, "cond_index": ci, "params": params, "repeat": r, "sweep": dict(zip(keys, combo))})
        if e["order"] == "shuffled":
            rng.shuffle(group)
        for g in group:
            n += 1
            parts = [f"{n:03d}"] + [slug(v) for v in cond.values()] + [f"{k}{slug(v)}" for k, v in g["sweep"].items()] + [f"r{g['repeat']}"]
            g["name"] = "_".join(parts)
            runs.append(g)
    return runs


def ask(prompt: str, choices: str) -> str:
    while True:
        try:
            a = input(prompt).strip().lower()
        except EOFError:
            raise Quit()
        if a == "" and "" in choices.split(","):
            return ""
        if a in choices.split(","):
            return a
        print(f"  answer one of: {choices.replace(',', ' ').strip() or 'Enter'}")


def wait_pressure(dev: Device, rig: Rig, target, tol, auto: bool, out=print) -> float | None:
    """Live accumulator pressure until it sits within tol of target for 1 s. Returns psi."""
    name = rig["accumulator_sensor"]
    if target is None or not name:
        return None
    line = rig.sensor(name)["adc"]
    if auto and dev.info_json and dev.info_json.get("sim"):
        dev.command(f"sim fill={target}")
        p = rig.psi(name, dev.wait_status()["adc"][line])
        if p > target + tol:
            dev.command(f"sim p={target}")
    ok_since = None
    last_print = 0.0
    try:
        while True:
            st = dev.wait_status(timeout=2.0)
            p = float(rig.psi(name, st["adc"][line]))
            inside = abs(p - target) <= tol
            ok_since = (ok_since or time.time()) if inside else None
            if time.time() - last_print > 0.25:
                hint = "ok" if inside else ("fill" if p < target else "vent")
                sys.stdout.write(f"\r  {name} {p:6.1f} psi  target {target} ± {tol}  [{hint}]   ")
                sys.stdout.flush()
                last_print = time.time()
            if ok_since and time.time() - ok_since >= 1.0:
                sys.stdout.write("\n")
                return p
            time.sleep(0.05)
    except KeyboardInterrupt:
        sys.stdout.write("\n")
        a = ask("  pressure wait interrupted: Enter = fire anyway, s = skip run, q = quit: ", ",s,q")
        if a == "q":
            raise Quit()
        if a == "s":
            return float("nan")
        return p


def run_experiment(rig: Rig, dev: Device, exp_path: str | Path, name: str | None = None, resume: bool = False,
                   auto: bool = False, valve_file: dict | None = None, out=print) -> Path:
    e = load(exp_path)
    runs = plan(e)
    inst = name or f"{slug(e['name'])}_{dt.datetime.now():%Y%m%d-%H%M}"
    exp_dir = rig.data_dir / inst
    if exp_dir.exists() and not resume and any(exp_dir.iterdir()):
        raise SystemExit(f"{exp_dir} exists; use --resume to continue it, or another --name")
    exp_dir.mkdir(parents=True, exist_ok=True)
    (exp_dir / "experiment.yaml").write_text(Path(exp_path).read_text())
    (exp_dir / "plan.json").write_text(json.dumps(runs, indent=1))
    cond_keys = sorted({k for c in e["conditions"] for k in c})
    sweep_keys = list(e["sweep"])
    out(f"experiment {e['name']} -> {exp_dir}  ({len(runs)} runs, {len(e['conditions'])} condition(s))")
    results, prev_cond = [], None
    close90 = (valve_file or {}).get("t_close_90_ms")
    try:
        i = 0
        while i < len(runs):
            r = runs[i]
            run_dir = exp_dir / r["name"]
            done = run_dir / "run.json"
            if resume and done.exists() and json.loads(done.read_text()).get("status") == "ok":
                if (run_dir / "analysis.json").exists():
                    results.append(json.loads((run_dir / "analysis.json").read_text()))
                prev_cond = r["cond_index"]
                i += 1
                continue
            out(f"\n[{i + 1}/{len(runs)}] {r['name']}")
            if r["cond_index"] != prev_cond:
                desc = ", ".join(f"{k} = {v}" for k, v in r["condition"].items()) or "(no hardware labels)"
                out(f"  set up: {desc}")
                if not auto and ask("  Enter when set up, q = quit: ", ",q") == "q":
                    raise Quit()
                prev_cond = r["cond_index"]
            p = r["params"]
            psi = wait_pressure(dev, rig, p.get("pressure_psi"), p.get("pressure_tol", PARAM_DEFAULTS["pressure_tol"]), auto, out)
            if psi is not None and psi != psi:      # NaN: skipped
                i += 1
                continue
            if not auto:
                a = ask(f"  ready ({', '.join(f'{k}={v}' for k, v in r['sweep'].items()) or 'defaults'}): "
                        "Enter = FIRE, s = skip, q = quit: ", ",s,q")
                if a == "q":
                    raise Quit()
                if a == "s":
                    i += 1
                    continue
            meta = dict(r["condition"])
            meta.update({"experiment": e["name"], "instance": inst, "repeat": r["repeat"], "run_index": i + 1,
                         "pressure_at_fire": psi})
            try:
                run_shot(dev, rig, run_dir, p, meta, valve_file, log=out)
                res = run_report(run_dir, close90_ms=close90, log=out)
                results.append(res)
                out("  " + quick_line(res))
            except (ShotError, DeviceError) as ex:
                out(f"  shot failed: {ex}")
                if auto:
                    raise
                a = ask("  r = retry, s = skip, q = quit: ", "r,s,q")
                if a == "q":
                    raise Quit()
                if a == "r":
                    continue
                i += 1
                continue
            if not auto:
                a = ask("  Enter = next, r = redo this run, q = quit: ", ",r,q")
                if a == "q":
                    raise Quit()
                if a == "r":
                    if run_dir.exists():
                        run_dir.rename(run_dir.with_name(run_dir.name + f"_discarded{int(time.time())}"))
                    results.pop()
                    continue
            i += 1
    except Quit:
        out("stopped; continue later with --resume")
    finally:
        if results:
            s = summary_report(exp_dir, results, sweep_keys, cond_keys)
            out(f"summary: {s}")
    return exp_dir


def quick_line(res: dict) -> str:
    def f(k, fmt="{:.1f}"):
        v = res.get(k)
        return "–" if v is None or v != v else fmt.format(v)
    jet = ((res.get("video") or {}).get("jet") or {})
    sy = ((res.get("video") or {}).get("sync") or {})
    s = (f"acc {f('p_acc_pre')}→{f('p_acc_post')} psi, nozzle {f('p_noz_plateau')} psi, open50 {f('t_open_50_ms')} ms, "
         f"close10 {f('t_close_10_ms')} ms, hold {f('i_hold', '{:.2f}')} A")
    if sy:
        s += f", video {'synced' if sy.get('ok') else 'NOT synced'}"
    if jet.get("front_speed_m_s") is not None:
        s += f", front {jet['front_speed_m_s']:.1f} m/s"
    return s
