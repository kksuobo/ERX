"""Merge episode CSVs, replacing any existing rows of the incoming planners."""

import argparse
import _bootstrap  # noqa: F401
import csv
import os

ROOT = _bootstrap.ROOT


def read(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        return list(r), r.fieldnames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--into", default=os.path.join(ROOT, "results", "episodes.csv"))
    ap.add_argument("--from", dest="src", required=True)
    args = ap.parse_args()

    rows, fields = read(args.into)
    add, afields = read(args.src)
    # Replace every (planner, scenario) pair present in the incoming file, so a
    # single-scenario slice merges without clobbering the other scenarios.
    incoming = {(r["planner"], r["scenario"]) for r in add}
    rows = [r for r in rows if (r["planner"], r["scenario"]) not in incoming]
    rows.extend(add)
    rows.sort(key=lambda r: (r["scenario"], r["planner"], float(r["seed"])))

    with open(args.into, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in fields})
    print(f"{args.into}: {len(rows)} rows after merging "
          f"{len(incoming)} (planner, scenario) slices")


if __name__ == "__main__":
    main()
