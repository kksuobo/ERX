# Development notes

Technical record of the benchmark and of the revisions that changed the paper's
claims. Result files under `results/` are authoritative.

## Frozen configuration

`configs/experiment.json` holds the benchmark definition: ego limits, the six
scenarios, planner hyper-parameters, metrics and seeds. It is fingerprinted
(SHA-256) and every table records the fingerprint used, so a reported number can
be traced to the exact settings.

## Benchmark

A self-contained two-dimensional kinematic simulator (`radwa/`) advances a
unicycle plant at 100 ms per cycle and applies the actuator envelope
independently of the planner, so all planners face identical dynamics. Six
scenarios are generated deterministically from an identifier and an integer seed:
static field, multi-agent crossing, dense merging, oncoming narrow road,
intersection, and roundabout. `scripts/verify.py` runs 33 dependency-free checks
(geometry, TTC and risk invariants, statistical helpers, configuration-fingerprint
stability, cross-process determinism of scenario generation, and a headline
performance band).

## Planners

Six baselines plus the proposed method. DWA (fixed weights, static obstacles),
DWA-P (DWA with motion-aware object prediction, the prediction control), APF, VO
(velocity obstacles, safety margin matched to the proposed method), MPPI-lite and
TEB. RA-DWA integrates a bounded normalized time-to-collision risk field along
each roll-out, propagates obstacles by their measured motion, and schedules its
weights over four contexts with hysteresis.

## Results

Main comparison (6 scenarios x 50 seeds; TEB 20 seeds), `results/table4_ci.md`:

| method | success | collision | min clear. | min TTC | mean abs jerk | Var(omega) |
|---|---|---|---|---|---|---|
| DWA | 0.91 | 0.06 | 1.05 | 1.06 | 2.31 | 0.146 |
| DWA-P | 0.96 | 0.00 | 1.29 | 1.34 | 2.02 | 0.067 |
| APF | 0.51 | 0.28 | 1.02 | 0.63 | 1.49 | 0.004 |
| VO | 0.99 | 0.00 | 0.70 | 1.11 | 3.37 | 0.042 |
| TEB | 0.85 | 0.15 | 1.01 | 1.66 | 8.77 | 0.046 |
| MPPI | 0.69 | 0.31 | 0.65 | 2.08 | 4.49 | 0.094 |
| RA-DWA | 0.99 | 0.00 | 0.56 | 3.56 | 1.92 | 0.019 |

## Ablation

`results/table5_ablation.md`, 8 variants x 6 scenarios x 20 seeds with paired
tests. Removing motion-aware prediction is the costliest change (success 0.98 to
0.89). Removing the risk field leaves success almost unchanged but reduces the
minimum time-to-collision from 3.52 s to 1.40 s. The explicit speed-set
modulation is neutral for safety.

## Robustness

Perception noise (`results/robustness.md`), actuation delay and command jitter
(`results/stress.csv`), runtime scaling (`results/scaling.csv`) and an external
validation on SUMO-generated traffic (`results/sumo_validation.md`).

## Revision history that changed the claims

1. The scenario suite was made denser after the first results looked too easy for
   competent planners.
2. A reproducibility defect was fixed: scenario seeding used Python's salted
   string hash, so scenarios differed between processes; it now uses a stable
   checksum.
3. The risk term was changed from a hyperbolic to a bounded exponential form with
   an explicit cutoff, which removed a deadlock in front of static obstacles.
4. Motion-aware prediction was added and proved to be the largest single factor
   in task success.
5. A prediction-matched control (DWA-P) and a margin-matched velocity-obstacle
   control were added. Both match the proposed method on success and collisions,
   including under perception noise, so the earlier claim of a structural
   robustness advantage for a graded decision rule over a hard feasibility test
   was withdrawn and is reported as a negative result. The attributable advantage
   of the method is the larger time-to-collision margin and the lower steering-rate
   variance.
6. All tables and figures were regenerated after each change; the paper is 20
   pages with the manuscript at 12 pt.
