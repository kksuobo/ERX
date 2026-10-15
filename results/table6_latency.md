# Table 6 -- real-time cost per control cycle (100 ms budget)

| Method | mean lat. [ms] | p95 lat. [ms] | max lat. [ms] | roll-outs/cycle | trained model |
|---|---|---|---|---|---|
| DWA | 3.57 | 12.22 | 16.01 | 153 | no |
| DWA-P | 3.99 | 9.31 | 13.54 | 153 | no |
| APF | 0.01 | 0.06 | 0.80 | 1 | no |
| VO | 8.92 | 16.22 | 28.64 | 231 | no |
| TEB | 151.42 | 306.13 | 434.84 | 3 x L-BFGS | no |
| MPPI | 1.60 | 5.15 | 8.98 | 64 | no |
| RA-DWA | 19.51 | 48.31 | 59.23 | 153 | no |
