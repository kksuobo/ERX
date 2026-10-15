# Robustness to perception noise (6 scenarios x 15 seeds = 90 episodes per cell)

Noise adds Gaussian position/velocity error and random obstacle dropout to what the planner perceives; ground truth drives motion and metrics. Reproduce with `python scripts\robustness.py --seeds 15` (TEB is excluded: its per-cycle cost precludes a real-time robustness comparison).

| Level | Method | Success (95% CI) | Collision (95% CI) | Min clear. | Min TTC | Mean \|jerk\| |
|---|---|---|---|---|---|---|
| clean | DWA | 0.89 [0.81, 0.94] | 0.07 [0.03, 0.14] | 1.07 | 1.01 | 2.34 |
| clean | DWA-P | 0.94 [0.88, 0.98] | 0.00 [0.00, 0.04] | 1.29 | 1.32 | 1.99 |
| clean | APF | 0.50 [0.40, 0.60] | 0.28 [0.20, 0.38] | 1.00 | 0.59 | 1.48 |
| clean | VO | 1.00 [0.96, 1.00] | 0.00 [0.00, 0.04] | 0.71 | 1.06 | 3.38 |
| clean | MPPI | 0.68 [0.58, 0.77] | 0.30 [0.22, 0.40] | 0.61 | 1.67 | 4.48 |
| clean | RA-DWA | 0.98 [0.92, 0.99] | 0.01 [0.00, 0.06] | 0.54 | 3.44 | 1.95 |
| low | DWA | 0.80 [0.71, 0.87] | 0.16 [0.09, 0.24] | 0.92 | 0.85 | 4.21 |
| low | DWA-P | 0.97 [0.91, 0.99] | 0.00 [0.00, 0.04] | 1.29 | 1.29 | 4.71 |
| low | APF | 0.50 [0.40, 0.60] | 0.47 [0.37, 0.57] | 0.97 | 0.36 | 9.84 |
| low | VO | 1.00 [0.96, 1.00] | 0.00 [0.00, 0.04] | 0.69 | 1.05 | 6.40 |
| low | MPPI | 0.71 [0.61, 0.79] | 0.29 [0.21, 0.39] | 0.58 | 1.60 | 4.98 |
| low | RA-DWA | 0.99 [0.94, 1.00] | 0.01 [0.00, 0.06] | 0.72 | 2.76 | 2.67 |
| medium | DWA | 0.72 [0.62, 0.80] | 0.24 [0.17, 0.34] | 0.73 | 0.68 | 5.85 |
| medium | DWA-P | 0.98 [0.92, 0.99] | 0.00 [0.00, 0.04] | 1.18 | 1.13 | 7.25 |
| medium | APF | 0.50 [0.40, 0.60] | 0.50 [0.40, 0.60] | 0.91 | 0.32 | 11.03 |
| medium | VO | 0.92 [0.85, 0.96] | 0.07 [0.03, 0.14] | 0.61 | 0.71 | 8.75 |
| medium | MPPI | 0.62 [0.52, 0.72] | 0.38 [0.28, 0.48] | 0.47 | 1.40 | 5.40 |
| medium | RA-DWA | 0.92 [0.85, 0.96] | 0.04 [0.02, 0.11] | 1.11 | 1.65 | 3.39 |
| high | DWA | 0.53 [0.43, 0.63] | 0.47 [0.37, 0.57] | 0.42 | 0.56 | 8.27 |
| high | DWA-P | 0.68 [0.58, 0.77] | 0.32 [0.23, 0.42] | 0.54 | 0.57 | 9.51 |
| high | APF | 0.50 [0.40, 0.60] | 0.50 [0.40, 0.60] | 0.77 | 0.24 | 12.18 |
| high | VO | 0.48 [0.38, 0.58] | 0.52 [0.42, 0.62] | 0.22 | 0.25 | 10.30 |
| high | MPPI | 0.58 [0.47, 0.67] | 0.42 [0.33, 0.53] | 0.39 | 1.16 | 5.51 |
| high | RA-DWA | 0.70 [0.60, 0.78] | 0.30 [0.22, 0.40] | 0.87 | 0.80 | 4.93 |

Key finding: DWA-P (prediction, no risk field) is the most robust planner at medium noise, and DWA-P and RA-DWA are the best at high noise. VO's apparent collapse under noise is largely a safety-margin effect: with its margin matched to RA-DWA's it holds 0.92 at medium and 0.68 at high noise (results/robustness_vo_m07.csv, results/robustness_vo_m12.csv). RA-DWA's residual advantage under noise is the larger minimum TTC, not task success.
