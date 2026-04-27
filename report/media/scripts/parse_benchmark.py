"""Parse hybrid_astar benchmark log output into a summary CSV.

Usage:
    uv run runsim car --no-render -n 10 2>&1 | uv run python report/media/scripts/parse_benchmark.py car
    uv run runsim diff --no-render -n 10 2>&1 | uv run python report/media/scripts/parse_benchmark.py diff

Or pipe multiple bot types into separate invocations; the script appends to the CSV.
Pass --reset to overwrite instead of append.

Output: report/data/benchmark.csv
"""

import re
import sys
from pathlib import Path

OUT = Path(__file__).parents[2] / "data" / "benchmark.csv"

FOUND_RE  = re.compile(r"\[hybrid_astar\] found path:\s+(\d+) expansions,.*?,\s+([\d.]+)s")
FAIL_RE   = re.compile(r"\[hybrid_astar\] no path found after\s+(\d+) expansions,\s+([\d.]+)s")
RUN_RE    = re.compile(r"\[run \d+/\d+\] seed=(\d+)")

def parse(lines: list[str]) -> list[dict]:
    runs = []
    current_seed = None
    for line in lines:
        m = RUN_RE.search(line)
        if m:
            current_seed = int(m.group(1))
        m = FOUND_RE.search(line)
        if m:
            runs.append({"seed": current_seed, "success": True,
                         "expansions": int(m.group(1)), "time_s": float(m.group(2))})
            current_seed = None
        m = FAIL_RE.search(line)
        if m:
            runs.append({"seed": current_seed, "success": False,
                         "expansions": int(m.group(1)), "time_s": float(m.group(2))})
            current_seed = None
    return runs

def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    if not args:
        print("Usage: parse_benchmark.py <bot_type> [--reset]", file=sys.stderr)
        sys.exit(1)

    bot_type = args[0]
    reset    = "--reset" in flags

    runs = parse(sys.stdin.readlines())
    if not runs:
        print("No runs found in input.", file=sys.stderr)
        sys.exit(1)

    OUT.parent.mkdir(parents=True, exist_ok=True)

    write_header = reset or not OUT.exists()
    mode = "w" if reset or not OUT.exists() else "a"

    with open(OUT, mode) as f:
        if write_header:
            f.write("bot_type,seed,success,expansions,time_s\n")
        for r in runs:
            f.write(f"{bot_type},{r['seed']},{int(r['success'])},{r['expansions']},{r['time_s']:.2f}\n")

    success = [r for r in runs if r["success"]]
    fail    = [r for r in runs if not r["success"]]
    print(f"{bot_type}: {len(runs)} runs | {len(success)} succeeded | "
          f"avg time (success): {sum(r['time_s'] for r in success)/len(success):.2f}s | "
          f"avg expansions (success): {sum(r['expansions'] for r in success)//len(success)}"
          if success else f"{bot_type}: {len(runs)} runs | 0 succeeded")
    print(f"Written to {OUT}")

if __name__ == "__main__":
    main()
