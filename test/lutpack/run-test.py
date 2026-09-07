#!/usr/bin/env python3
"""Check lutpack window correctness on deterministic reconvergent networks."""
import argparse
import pathlib
import random
import subprocess
import tempfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("abc", type=pathlib.Path)
parser.add_argument("--seeds", type=int, default=24)
args = parser.parse_args()
abc = args.abc.resolve()

with tempfile.TemporaryDirectory(prefix="abc-lutpack-") as directory:
    work = pathlib.Path(directory)
    for seed in range(args.seeds):
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
        script = ("read input.blif; strash; if -K 4; mfs2; write_blif before.blif; "
                  "lutpack -S 1; write_blif after.blif; cec before.blif after.blif")
        run = subprocess.run([str(abc), "-c", script], cwd=work,
                             capture_output=True, text=True, timeout=60)
        assert run.returncode == 0 and "Networks are equivalent" in run.stdout, (seed, run.stdout, run.stderr)
        assert (work / "after.blif").is_file(), seed
        print(f"PASS: lutpack reconvergent seed {seed}", flush=True)
