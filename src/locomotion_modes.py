#!/usr/bin/env python
import enum

class LocomotionMode(enum.Enum):
    ACKERMANN = 1
    POINT_TURN = 2
    CRABBING = 3