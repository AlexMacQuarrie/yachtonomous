# Module for plotting tools
from enum import Enum


class PlotCtrl(Enum):
    ALWAYS  = 0,
    NEVER   = 1,
    ON_PASS = 2,
    ON_FAIL = 3