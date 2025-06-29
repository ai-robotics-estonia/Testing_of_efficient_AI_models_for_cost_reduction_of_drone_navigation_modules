# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague
# Reference: https://github.com/ctu-vras/compass/blob/master/compass_msgs/msg/Azimuth.msg

Header header

# The represented azimuth angle (angular difference between a given direction and north/east) in unit corresponding to
# the 'unit' field.
float64 azimuth

# Variance of the measurement (i.e. standard deviation squared) in units corresponding to squared 'unit' field.
float64 variance

# Unit of measurement:
#  RAD: radians, default
#  DEG: degrees (not suggested by REP-103, but are more human-friendly)
uint8 UNIT_RAD=0
uint8 UNIT_DEG=1
uint8 unit

# Orientation type:
#  ENU: default, 0 degrees on east, increases CCW, follows to REP-103
#  NED: 0 degrees on north, increases CW (consistent with the touristic magnetic compasses)
uint8 ORIENTATION_ENU=0
uint8 ORIENTATION_NED=1
uint8 orientation

# Which north reference is used:
#  MAGNETIC: magnetic north, changes in time
#  GEOGRAPHIC: geographic north, stable in time, also called "true azimuth"
#  UTM: UTM grid azimuth, based on GEOGRAPHIC, but accounts for UTM grid convergence; valid only in specific UTM zone
uint8 REFERENCE_MAGNETIC=0
uint8 REFERENCE_GEOGRAPHIC=1
uint8 REFERENCE_UTM=2
uint8 reference