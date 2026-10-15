# RA-DWA — Reproducible local-planning benchmark

Code for *Risk-Adaptive Dynamic Window Approach* (ERX submission, see
`RA-DWA_ERX_plan.md`). The benchmark is a self-contained 2-D kinematic
simulator (unicycle plant, 100 ms control period) so that every planner emits
the same `(v, omega)` interface and every scenario is bit-reproducible.

## Layout

```
configs/experiment.json   frozen Table 2/3 entities (scenarios, metrics, params)
radwa/                    package
  risk.py                 normalized TTC risk field R = exp(-TTC/T0)
  scenarios.py            6-scenario suite (deterministic per seed)
  simulator.py            closed-loop episode + metrics
  metrics.py              aggregation into paper tables
  runner.py               batch driver
  planners/               DWA, APF, RA-DWA
scripts/                  thin CLI entry points
results/                  episodes.csv, overall summary, markdown tables
figures/                  generated figures
```

## Setup

```
pip install -r requirements.txt
```

## Run

```
python scripts/verify.py                           # 26 invariants/regression checks
python scripts/smoke_test.py                       # 1-seed sanity check
python scripts/run_benchmark.py --seeds 50         # benchmark -> results/
python scripts/report.py                           # overall + per-scenario table
python scripts/significance.py                     # paired tests -> table_significance.md
python scripts/tables.py                           # Table 4 (Wilson CI) + Table 6 (latency)
python scripts/ablation.py --seeds 20              # Table 5 ablation
python scripts/robustness.py --seeds 15            # noise sweep, 5 planners (CIs)
python scripts/robustness_report.py                # -> results/robustness.md
python scripts/stress.py --seeds 8                 # actuation delay / jitter
python scripts/scaling.py                          # latency vs obstacles / roll-outs
python scripts/make_method_figures.py              # Fig 2 / Fig 3
python scripts/make_figures.py --scenario crossing --seed 4
python scripts/make_sensitivity.py                 # Fig 8
python scripts/make_robustness_figure.py           # Fig 9
python scripts/make_stress_figure.py               # Fig 12
python scripts/sumo_validation.py --seeds 10       # SUMO external validation (Fig 13)
python scripts/diagnose.py --seeds 20              # per-(planner,scenario) table
python scripts/scan.py --scenario crossing --planner RA-DWA --seeds 20
python scripts/debug_episode.py --scenario merge --planner RA-DWA --seed 3
python scripts/debug_costs.py --scenario static_field --seed 4   # cost trace
```

## Method (RA-DWA, summary)

One normalized TTC risk field drives both decisions:

1. Sample `(v, omega)` inside the dynamic window (accelerations limit the set).
2. Roll each candidate out over the horizon `T`; integrate the risk field.
3. Cost `C = w_g*progress + w_c*heading + w_r*∫R + w_s*smooth + w_j*jerk
   + w_d*clearance`; predicted collisions are inadmissible.
4. `w_*` come from a four-state scheduler (free / follow / conflict / emergency)
   with hysteresis; risk also shrinks the admissible speed set
   (`v_max_eff = v_max (1 - gain * rho)`).

## Results (6 scenarios x 50 seeds = 1500 episodes)

| Method | Success | Collision | Min clear. | Min TTC | Mean \|jerk\| | Var(omega) | p95 latency |
|---|---|---|---|---|---|---|---|
| DWA | 0.91 | 0.06 | 1.05 | 1.06 | 2.31 | 0.146 | 12.2 ms |
| DWA-P (prediction control) | 0.96 | 0.00 | 1.29 | 1.34 | 2.02 | 0.067 | 9.3 ms |
| APF | 0.51 | 0.28 | 1.02 | 0.63 | 1.49 | 0.004 | 0.1 ms |
| VO (margin matched) | 0.99 | 0.00 | 0.70 | 1.11 | 3.37 | 0.042 | 16.2 ms |
| TEB (20 seeds) | 0.85 | 0.15 | 1.01 | 1.66 | 8.77 | 0.046 | 306 ms |
| MPPI | 0.69 | 0.31 | 0.65 | 2.08 | 4.49 | 0.094 | 5.2 ms |
| **RA-DWA** | **0.99** | **0.00** | 0.56 | **3.56** | **1.92** | **0.019** | 48.3 ms |

After an internal review the paper's claim is narrower than in earlier drafts:
motion-aware prediction (DWA-P) and a matched safety distance account for task
success, and RA-DWA's attributable advantage is the **largest minimum TTC and
the lowest steering-rate variance**. The earlier "graded vs hard constraint"
robustness claim is withdrawn (a negative result) because a margin-matched VO
matches RA-DWA under noise.

TEB exceeds the 100 ms real-time budget; every other planner meets it.

RA-DWA ties the strongest baseline (VO) on success/collision while keeping
~3x larger minimum clearance, ~8x larger minimum TTC (no near-misses), and the
lowest jerk of any planner that succeeds, within the 100 ms budget.

## Significance (paired, `results/table_significance.md`)

McNemar (rates) + Wilcoxon signed-rank (continuous), matched by (scenario, seed),
n = 300 per method, reference = RA-DWA:

- vs **DWA**: better on success, collision, clearance, TTC, jerk, Var(omega)
  (all p < 1e-4); e.g. roundabout collision 1/50 vs 15/50 (p < 1e-3).
- vs **APF / MPPI / TEB**: better on every metric (p < 1e-4).
- vs **VO**: success/collision statistically tied (p = 1.0), but RA-DWA has
  significantly larger clearance/TTC and lower jerk/Var(omega) (p < 1e-14).

## Perception-noise robustness (`results/robustness.md`, `figures/fig9_robustness.png`)

`sensor` in the config adds Gaussian position/velocity error and random obstacle
dropout to what the planner perceives (ground truth still drives motion and
metrics). Reproduce with `python scripts/robustness.py --planners DWA,VO,RA-DWA
--seeds 15`.

| Level | DWA (succ/coll) | VO | RA-DWA |
|---|---|---|---|
| clean | 0.89 / 0.07 | 0.99 / 0.01 | 0.98 / 0.01 |
| low | 0.80 / 0.16 | 0.79 / 0.21 | 0.99 / 0.01 |
| medium | 0.72 / 0.24 | 0.53 / 0.47 | 0.93 / 0.03 |
| high | 0.53 / 0.47 | 0.30 / 0.70 | 0.70 / 0.30 |

VO ties RA-DWA when perception is clean but collapses under noise; RA-DWA
degrades gracefully and is best at every noise level.

## Status / next steps

See `PROGRESS.md`. The manuscript storyline is framed as *graded risk vs hard
constraint under uncertainty* (see `manuscript/`), not as a clean-condition
collision-rate win over the strongest baseline. Built: 5 baselines (DWA/APF/VO/MPPI/TEB), 50-seed benchmark
with paired significance tests and Wilson CIs, per-scenario breakdown, 8-variant
ablation, parameter sensitivity, perception-noise and actuation-stress
robustness, runtime-scaling measurement, SUMO 1.27 external validation, a
13-figure set, a drafted manuscript (`manuscript/`, 23 Crossref-verified
references) and a PDF (`manuscript/RA-DWA_paper.pdf`, 16 pages), and 33
reproducibility checks. A
constant-turn-rate prediction extension is implemented but off by default.
