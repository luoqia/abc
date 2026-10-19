#!/usr/bin/env python3
"""Exercise parallel if policies, serial fallback, and PIF integration.

Run: python3 test/if/run-test.py /absolute/path/to/abc --work-dir _TEST/if-policy
Requires a pthread-enabled build. Logs and mapped networks stay in work-dir.
"""

import argparse
import math
import os
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[2]
POLICIES = {
    "default": "",
    "relax_ratio": "-R 10",
    "absolute_target": "-D 100",
    "unmet_target": "-D 1",
    "area_only": "-a",
}
POLICY_RECORD = re.compile(
    r"IF_PAR_REQUIRED_POLICY source=(\S+) mode=(-?\d+) policy=(\S+) "
    r"actual_delay=(\S+) required_glo=(\S+) propagate=([01])"
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def normalized_blif(path):
    return "\n".join(line for line in path.read_text().splitlines()
                     if line.strip() and not line.startswith("#"))


def wide_adder(source, destination):
    """Replicate the fixture so PIF exercises its >=64-node child path."""
    inputs, outputs, gates = [], [], []
    for lane in range(8):
        for line in source.read_text().splitlines():
            tokens = line.split()
            if not tokens or tokens[0] in (".model", ".end"):
                continue
            if tokens[0] == ".inputs":
                inputs.extend(f"b{lane}_{name}" for name in tokens[1:])
            elif tokens[0] == ".outputs":
                outputs.extend(f"b{lane}_{name}" for name in tokens[1:])
            elif tokens[0] == ".names":
                gates.append(".names " + " ".join(f"b{lane}_{name}" for name in tokens[1:]))
            else:
                gates.append(line)
    destination.write_text("\n".join([
        ".model wide_add8", ".inputs " + " ".join(inputs),
        ".outputs " + " ".join(outputs), *gates, ".end", "",
    ]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abc", type=Path)
    parser.add_argument("--work-dir", type=Path, default=ROOT / "_TEST/if-policy")
    args = parser.parse_args()
    binary = args.abc.resolve()
    work = args.work_dir.resolve()
    work.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IF_PAR_ROUND_VALIDATE="1", IF_PAR_IMPROVE_VALIDATE="1",
               PIF_ABC_BIN=str(binary), PIF_ABC_RC=str(ROOT / "abc.rc"))
    # Keep policy coverage independent of ambient profiling/runtime toggles.
    for key in list(env):
        if key.startswith("IF_PAR_") and key not in (
                "IF_PAR_ROUND_VALIDATE", "IF_PAR_IMPROVE_VALIDATE"):
            del env[key]
    runs = 0

    def run(label, commands):
        nonlocal runs
        script = work / f"{label}.abc"
        script.write_text("\n".join(commands) + "\n")
        result = subprocess.run([str(binary), "-s", "-f", str(script)], cwd=work,
                                env=env, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=120)
        (work / f"{label}.log").write_text(result.stdout)
        require(result.returncode == 0, f"{label}: exit {result.returncode}; see log")
        require("IF_PAR_INVARIANT" not in result.stdout, f"{label}: invariant failure")
        require("Networks are equivalent" in result.stdout, f"{label}: CEC did not pass")
        require((work / f"{label}.blif").is_file(), f"{label}: no mapped output")
        runs += 1
        return result.stdout

    def mapped(label, source, flags, choice=False):
        commands = [f"read_blif {source}", "strash"]
        if choice:
            commands.append("dch")
        commands += [f"if {flags}", "print_stats", f"write_blif {label}.blif",
                     f"cec {source} {label}.blif"]
        return run(label, commands)

    def check_policy(label, log, policy, choice):
        require("IF_PAR_UNSUPPORTED" not in log, f"{label}: unexpected fallback")
        expected = "absolute_target" if policy == "unmet_target" else policy
        require(re.search(r"IF_PAR_SUPPORTED\b[^\n]*\bpolicy=" + expected + r"\b", log),
                f"{label}: parallel policy not accepted")
        if choice:
            require(re.search(r"IF_PAR_SUPPORTED\b[^\n]*\bchoices=[1-9]\d*", log),
                    f"{label}: fixture did not exercise choices")
        records = POLICY_RECORD.findall(log)
        require(records, f"{label}: required-time policy not executed")
        for _, _, actual_policy, actual, required, propagate in records:
            require(actual_policy == expected, f"{label}: policy changed during mapping")
            require(math.isfinite(float(actual)) and math.isfinite(float(required)),
                    f"{label}: invalid timing value")
            require(propagate == ("0" if policy == "area_only" else "1"),
                    f"{label}: unexpected required-time propagation")
        if policy == "relax_ratio":
            target = float(records[0][3]) * 1.1
            require(all(math.isclose(float(row[4]), target, abs_tol=1e-5) for row in records),
                    f"{label}: relaxed target was not stable")
        if policy == "absolute_target":
            require(all(float(row[4]) == 100.0 for row in records),
                    f"{label}: attainable absolute target was not respected")
        if policy == "unmet_target":
            require("Cannot meet the target required times" in log,
                    f"{label}: unattainable target was not reported")
        modes = set(re.findall(r"IF_PAR_ROUND\b[^\n]*\bmode=([012])", log))
        require(modes == {"0", "1", "2"}, f"{label}: incomplete mapping rounds")

    source = ROOT / "test/dch2/add8.blif"
    for choice in (False, True):
        for width in (4, 6):
            for policy, options in POLICIES.items():
                stem = f"add8_{'choice' if choice else 'nochoice'}_k{width}_{policy}"
                flags = f"-K {width} {'-r' if choice else ''} {options} -v"
                serial = mapped(stem + "_serial", source, flags, choice)
                require("IF_PAR_SUPPORTED" not in serial, f"{stem}: serial path changed")
                for suffix, threads in [("j1", 1), ("j4", 4), ("j4_repeat", 4)]:
                    label = stem + "_" + suffix
                    log = mapped(label, source, flags + f" -P -j {threads}", choice)
                    check_policy(label, log, policy, choice)
                require(normalized_blif(work / f"{stem}_j4.blif") ==
                        normalized_blif(work / f"{stem}_j4_repeat.blif"),
                        f"{stem}: repeated j4 mapping changed")
                print(f"PASS {stem}: serial/j1/j4 CEC, policy, repeat", flush=True)

    fallbacks = [
        (f"-K {width} {policy}", "policy_combination")
        for width in (4, 6)
        for policy in ("-R 10 -D 100", "-R 10 -a", "-D 100 -a", "-R 10 -D 100 -a")
    ] + [("-K 5", "unsupported_lut_width"), ("-K 6 -C 9", "nondefault_cuts")]
    for index, (flags, reason) in enumerate(fallbacks):
        stem = f"fallback_{index}"
        mapped(stem + "_serial", source, flags)
        log = mapped(stem, source, flags + " -P -j 4 -v")
        require(f"IF_PAR_UNSUPPORTED reason={reason}" in log, f"{stem}: wrong fallback")
        require("IF_PAR_SUPPORTED" not in log and "IF_PAR_ROUND " not in log,
                f"{stem}: unsupported configuration entered workers")
        require(normalized_blif(work / f"{stem}.blif") ==
                normalized_blif(work / f"{stem}_serial.blif"),
                f"{stem}: fallback output differs from serial")
        print(f"PASS {stem}: {reason}, serial-identical output and CEC", flush=True)

    pif_source = work / "pif-input.blif"
    wide_adder(source, pif_source)
    for policy in ("relax_ratio", "absolute_target", "area_only"):
        label = f"pif_{policy}"
        commands = [f"read_blif {pif_source}", "strash",
                    f'pif -N 2 -j 2 -s 100000 -e -d {work}/{label}_parts -t {work} '
                    f'-S "strash; if -K 6 -P -j 2 {POLICIES[policy]} -v"',
                    f"write_blif {label}.blif", f"cec {pif_source} {label}.blif"]
        log = run(label, commands)
        outcomes = re.search(r"Child Outcomes:\s+ok=(\d+) fallback=(\d+) failure=(\d+)", log)
        require(outcomes and int(outcomes[1]) > 0 and outcomes.groups()[1:] == ("0", "0"),
                f"{label}: PIF child failed or used fallback")
        require(re.search(r"\[PIF-CHILD\][^\n]*reason=ok", log),
                f"{label}: PIF did not execute a child process")
        print(f"PASS {label}: child processes, no fallback, merged CEC", flush=True)

    print(f"PASS {runs} mapping/PIF runs; artifacts: {work}", flush=True)


if __name__ == "__main__":
    main()
