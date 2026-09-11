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
    for i in range(256):
        # Shared cones, inverted literals and XORs exercise window
        # ordering and truth-table ownership through distinct parents.
        a, b = rng.sample(signals, 2)
        name = f"n{i}"
        kind = rng.randrange(6)
        if kind < 2:
            lines += [f".names {a} {name}", "0 1" if kind else "1 1"]
        else:
            cubes = ["11 1", "00 1", "01 1\n10 1", "00 1\n11 1"][kind - 2]
            lines += [f".names {a} {b} {name}", cubes]
        signals.append(name)
    lines += [".outputs " + " ".join(signals[-16:]), ".end"]
    (work / "input.blif").write_text("\n".join(lines) + "\n")


def make_wide(work, n_nodes=100000):
    # Large object count with trivial per-node cones: exercises the scratch
    # admission arithmetic without heavy cut enumeration.
    rng = random.Random(1234)
    lines = [".model wide", ".inputs " + " ".join(f"i{i}" for i in range(32))]
    for i in range(n_nodes):
        a, b = rng.randrange(32), rng.randrange(32)
        lines += [f".names i{a} i{b} n{i}", "11 1"]
    lines += [".outputs " + " ".join(f"n{n_nodes - 1 - i}" for i in range(16)), ".end"]
    (work / "wide.blif").write_text("\n".join(lines) + "\n")


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
             [r"committed_par=[1-9]"]),
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

    # Scratch admission: driver plus worker contexts stay inside the budget,
    # and a budget below one context falls back to the serial iteration.
    make_wide(work)
    run = run_abc(work, "read wide.blif; strash; if -K 4; lutpack -S 1; write_blif wide-after.blif",
                  env={"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1",
                       "LUTPACK_SCRATCH_MB": "4"}, timeout=600)
    assert run.returncode == 0, (run.stdout[-2000:], run.stderr[-2000:])
    m = re.search(r"scratch (\d+) B/context, budget (\d+) B -> workers 8 reduced to (\d+)", run.stdout)
    assert m, run.stdout[-2000:]
    per_context, budget, workers = int(m.group(1)), int(m.group(2)), int(m.group(3))
    assert workers >= 1 and per_context * (workers + 1) <= budget, (per_context, budget, workers)
    m2 = re.search(r"scratch_bytes=(\d+)", run.stdout)
    assert m2 and int(m2.group(1)) == per_context * (workers + 1) and int(m2.group(1)) <= budget, run.stdout[-2000:]
    print(f"PASS: lutpack scratch admission clamp workers={workers} scratch_bytes={m2.group(1)} budget={budget}", flush=True)

    run = run_abc(work, "read wide.blif; strash; if -K 4; lutpack -S 1; write_blif wide-serial.blif",
                  env={"LUTPACK_WORKERS": "8", "LUTPACK_STATS": "1",
                       "LUTPACK_SCRATCH_MB": "1"}, timeout=600)
    assert run.returncode == 0, (run.stdout[-2000:], run.stderr[-2000:])
    assert "parallel pass not admitted" in run.stdout, run.stdout[-2000:]
    assert (work / "wide-after.blif").is_file() and (work / "wide-serial.blif").is_file()
    run = run_abc(work, "cec wide-after.blif wide-serial.blif", timeout=600)
    assert run.returncode == 0 and "Networks are equivalent" in run.stdout, (run.stdout[-2000:], run.stderr[-2000:])
    print("PASS: lutpack below-one-context budget falls back to the serial iteration", flush=True)
