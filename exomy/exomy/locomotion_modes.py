#!/usr/bin/env python

import enum


class LocomotionMode(enum.Enum):
    FAKE_ACKERMANN = 0
    ACKERMANN = 1
    POINT_TURN = 2
    CRABBING = 3
