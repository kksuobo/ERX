# Cover letter

**To:** The Editorial Board, *Engineering Research Express* (IOP Publishing)

**Manuscript title:** Risk-Adaptive Dynamic Window Approach: a normalized
time-to-collision risk field sets the safety margin of real-time obstacle avoidance

**Authors:** Suihua Lin and Yuelan Huang (corresponding author)

**Article type:** Research Paper

---

Dear Editors,

Please consider the enclosed manuscript, *Risk-Adaptive Dynamic Window Approach: a
normalized time-to-collision risk field sets the safety margin of real-time
obstacle avoidance*, for publication as a Research Paper in *Engineering Research
Express*.

**What the paper does.** We study what determines the safety margin that a fast
reactive planner can hold, and we propose RA-DWA, a training-free dynamic-window
planner that replaces the fixed obstacle penalty with a bounded normalized
time-to-collision risk field, integrated along every candidate roll-out, with
motion-aware prediction and a closed-form four-state scheduler.

**Why it fits ERX.** The work is a reproducible engineering study of a real-time
control algorithm. All claims are supported by a frozen, fingerprinted benchmark
with six scenarios, five baselines plus a prediction control, paired significance
tests, Wilson intervals, perception-noise and actuation-stress sweeps, a runtime
scaling measurement, and an independent SUMO-generated-traffic validation. The
full code and configuration are available, and `scripts/verify.py` provides 33
dependency-free reproducibility checks. The study is written to the journal's
emphasis on scientific rigour rather than perceived novelty.

**A note on framing.** The paper reports a decomposition that is unusual for this
literature and that we believe is the most useful part of the study. Two controls
show that much of the advantage of a risk-based planner over classical DWA comes
from motion-aware prediction and from a safety distance that reflects the
planner's own risk model, and that the earlier expectation of a structural
robustness advantage for a graded decision rule over a hard feasibility test does
not survive a margin-matched control. We report this explicitly as a negative
result and identify the narrower, attributable advantage instead: a substantially
larger time-to-collision margin and smoother steering at equal task success. We
hope the editors agree that a well-identified, honestly reported decomposition
suits the journal's scope.

**Statements.** The manuscript is original, is not under consideration elsewhere,
and has not been published previously in the peer-reviewed literature. There are
no competing interests. No human or animal subjects were involved, so ethics
approval was not required. Funding and author contributions are given in the
Acknowledgements. The manuscript is prepared for the single-anonymous model, with
author names and institutes at the start of the PDF and figures and tables
embedded in the text.

We would be glad to suggest reviewers if the editorial office wishes.

Yours sincerely,

Yuelan Huang
School of Mathematics and Computer Science
Guangxi Minzu Normal University
No. 23 Fuzi Road, Jiangzhou District, Chongzuo 532200, Guangxi, China
Email: zhuyi19840719@163.com
