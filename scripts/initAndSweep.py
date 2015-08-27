#!/usr/bin/env python

import roslib
from pr2_safety import Init
from pr2_safety import PR2Sweep
roslib.load_manifest('pr2_safety')
Init.main()
PR2Sweep.main()