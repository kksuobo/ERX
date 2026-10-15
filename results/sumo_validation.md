# External validation on SUMO-generated traffic

SUMO 1.27 [22] corridor (3-lane main road with oncoming and merging traffic), 20 seeds. The ego plant and planners are unchanged; SUMO traffic is replayed as moving obstacles (open-loop with respect to the ego). Goal tolerance 2.0 m and a 60 s episode limit (larger than the main benchmark, the corridor being longer); obstacles are modelled as 1.5 m discs. DWA-P is DWA with motion-aware obstacle prediction, the control that separates prediction from the decision rule.

| Method | n | Success (95% CI) | Collision (95% CI) | Min clear. | Min TTC | Mean \|jerk\| | succ p vs RA-DWA |
|---|---|---|---|---|---|---|---|
| DWA | 20 | 0.25 [0.11, 0.47] | 0.75 [0.53, 0.89] | 0.02 | 0.40 | 0.16 | 1.22e-04 *** |
| DWA-P | 20 | 0.95 [0.76, 0.99] | 0.05 [0.01, 0.24] | 1.31 | 1.15 | 0.45 | 1.00e+00 n.s. |
| VO | 20 | 0.60 [0.39, 0.78] | 0.40 [0.22, 0.61] | 0.40 | 0.60 | 0.96 | 1.56e-02 * |
| RA-DWA | 20 | 0.95 [0.76, 0.99] | 0.05 [0.01, 0.24] | 0.64 | 2.87 | 0.56 | ref |
| MPPI | 20 | 0.35 [0.18, 0.57] | 0.65 [0.43, 0.82] | 0.36 | 0.56 | 5.91 | 4.88e-04 *** |
