"""Shared statistical tests for the paper tables (single source of truth).

Used by ``scripts/significance.py`` (Table 4) and ``scripts/ablation.py``
(Table 5) so the same paired tests back every reported p-value.
"""

import math

import numpy as np
from scipy import stats


def wilson(k, n, z=1.96):
    """95% Wilson score interval for a binomial rate; returns (lo, hi)."""
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1.0 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def mcnemar(a, b):
    """Exact McNemar test on paired binary outcomes.

    Returns (p_value, n01, n10) where n01 counts (a=1, b=0) and n10 counts
    (a=0, b=1) discordant pairs.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n01 = int(np.sum((a == 1) & (b == 0)))
    n10 = int(np.sum((a == 0) & (b == 1)))
    n = n01 + n10
    p = 1.0 if n == 0 else float(stats.binomtest(n01, n, 0.5).pvalue)
    return p, n01, n10


def wilcoxon(a, b):
    """Wilcoxon signed-rank test over finite paired values.

    Returns (p_value, n_pairs_used).
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    mask = np.isfinite(a) & np.isfinite(b)
    a, b = a[mask], b[mask]
    if len(a) < 5 or np.allclose(a, b):
        return 1.0, len(a)
    try:
        return float(stats.wilcoxon(a, b).pvalue), len(a)
    except ValueError:
        return 1.0, len(a)


def star(p):
    """Significance marker for a p-value."""
    return "***" if p < 1e-3 else "**" if p < 0.01 else "*" if p < 0.05 else "n.s."
