#!/usr/bin/env python3
"""Check lutpack window correctness on deterministic reconvergent networks.

Covers the serial path, the private-window parallel engine (LUTPACK_WORKERS),
the exact live-structure rejection self-test (LUTPACK_SELFTEST_FP) and the
driver-plus-worker scratch admission bound (LUTPACK_SCRATCH_MB).
"""
import argparse
import os
import pathlib
import random
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("abc", type=pathlib.Path)
parser.add_argument("--seeds", type=int, default=24)
args = parser.parse_args()
abc = args.abc.resolve()


def make_reconvergent(work, seed):
    rng = random.Random(seed)
    signals = [f"i{i}" for i in range(32)]
    lines = [".model reconvergent", ".inputs " + " ".join(signals)]
    for i in range(4096):
        # Distinct 4-input cones that stay above the parallel-engine size
        # gate after `if -K 4`, with shared and inverted parents.
        a, b, c, d = rng.sample(signals, 4)
        name = f"n{i}"
        lines += [f".names {a} {b} {c} {d} {name}", "1111 1", "0001 1", "0010 1", "0100 1"]
        signals.append(name)
    lines += [".outputs " + " ".join(signals[-16:]), ".end"]
    (work / "input.blif").write_text("\n".join(lines) + "\n")


def _write_lut_net(work, name, n_nodes, rng, sig_count=32, local=0):
    """Write a BLIF of n_nodes distinct 4-input functions with random minterm
    sets; with local>0 most cones come from the recent window so the mapped
    network stays large (the parallel-engine admission scales with it)."""
    sig = [f"i{i}" for i in range(sig_count)]
    lines = [f".model {name}", ".inputs " + " ".join(sig)]
    for i in range(n_nodes):
        if local and len(sig) > local:
            a, b = rng.sample(sig[-local:], 2)
            c, d = rng.sample(sig, 2)
        else:
            a, b, c, d = rng.sample(sig, 4)
        node = f"n{i}"
        lines.append(f".names {a} {b} {c} {d} {node}")
        for _ in range(3 if local else 4):
            m = rng.randrange(16)
            cube = "".join("1" if (m >> k) & 1 else "0" for k in range(4))
            lines.append(cube + " 1")
        sig.append(node)
    lines.append(".outputs " + " ".join(sig[-16:]))
    lines.append(".end")
    (work / f"{name}.blif").write_text("\n".join(lines) + "\n")


def run_abc(work, script, env=None, timeout=300):
    merged = dict(os.environ)
    merged.update(env or {})
    return subprocess.run([str(abc), "-c", script], cwd=work,
                          capture_output=True, text=True, timeout=timeout, env=merged)


with tempfile.TemporaryDirectory(prefix="abc-lutpack-") as directory:
    work = pathlib.Path(directory)
    for seed in range(args.seeds):
        make_reconvergent(work, seed)
        script = ("read input.blif; strash; if -K 4; mfs2; write_blif before.blif; "
                  "lutpack -S 1; write_blif after.blif; cec before.blif after.blif")
        run = run_abc(work, script, timeout=60)
        assert run.returncode == 0 and "Networks are equivalent" in run.stdout, (seed, run.stdout, run.stderr)
        assert (work / "after.blif").is_file(), seed
        print(f"PASS: lutpack reconvergent seed {seed}", flush=True)

    # Parallel engine and its invalidation repair on deterministic networks.
    for seed in range(min(4, args.seeds)):
        make_reconvergent(work, seed)
        for label, env, expect in [
            ("parallel", {"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1"},
             [r"\[lpkpar\] storage: .* workers=8 "]),
            ("selftest-fp", {"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1",
                             "LUTPACK_SELFTEST_FP": "1"},
             [r"drifted=[1-9]"]),
        ]:
            script = ("read input.blif; strash; if -K 4; mfs2; write_blif before.blif; "
                      "lutpack -S 1; write_blif after.blif; cec before.blif after.blif")
            run = run_abc(work, script, env=env, timeout=120)
            assert run.returncode == 0 and "Networks are equivalent" in run.stdout, (label, seed, run.stdout, run.stderr)
            for pattern in expect:
                assert re.search(pattern, run.stdout), (label, seed, pattern, run.stdout)
            print(f"PASS: lutpack {label} seed {seed}", flush=True)

    # Scratch admission: driver plus worker contexts stay inside the budget
    # (clamped case), and a budget below one context is not admitted at all
    # (the caller then runs the classic serial iteration).
    rng = random.Random(1234)
    _write_lut_net(work, "wide", 100000, rng)
    run = run_abc(work, "read wide.blif; strash; if -K 4; lutpack -S 1; write_blif wide-after.blif",
                  env={"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1",
                       "LUTPACK_SCRATCH_MB": "1"}, timeout=600)
    assert run.returncode == 0, (run.stdout[-2000:], run.stderr[-2000:])
    m = re.search(r"scratch (\d+) B/context, budget (\d+) B -> workers 8 reduced to (-?\d+)", run.stdout)
    assert m, run.stdout[-2000:]
    per_context, budget, workers = int(m.group(1)), int(m.group(2)), int(m.group(3))
    assert workers >= 1 and per_context * (workers + 1) <= budget, (per_context, budget, workers)
    m2 = re.search(r"scratch_bytes=(\d+)", run.stdout)
    assert m2 and int(m2.group(1)) == per_context * (workers + 1) and int(m2.group(1)) <= budget, run.stdout[-2000:]
    print(f"PASS: lutpack scratch admission clamp workers={workers} scratch_bytes={m2.group(1)} budget={budget}", flush=True)

    rng2 = random.Random(99)
    _write_lut_net(work, "huge", 400000, rng2, sig_count=64, local=200)
    run = run_abc(work, "read huge.blif; strash; if -K 4; lutpack -S 1; write_blif huge-serial.blif",
                  env={"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1",
                       "LUTPACK_SCRATCH_MB": "1"}, timeout=900)
    assert run.returncode == 0, (run.stdout[-2000:], run.stderr[-2000:])
    assert "no room for the driver plus one worker; parallel pass not admitted" in run.stdout, run.stdout[-2000:]
    print("PASS: lutpack below-one-context budget falls back to the serial iteration", flush=True)
