"""Local planners: baselines and RA-DWA."""

from .apf import APF
from .dwa import DWA, DWAP
from .mppi import MPPI
from .radwa import RADWA
from .teb import TEB
from .vo import VO

PLANNERS = {"DWA": DWA, "DWA-P": DWAP, "APF": APF, "VO": VO, "MPPI": MPPI,
            "TEB": TEB, "RA-DWA": RADWA}
