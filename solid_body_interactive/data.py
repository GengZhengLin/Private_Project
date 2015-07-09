#!/usr/bin/env python

import numpy as np
import pylab

data = []
with open('data', 'r') as f:
  for line in f.readlines():
    parts = [float(part) for part in line.split()]
    data.append(tuple(parts))

x = [xy[0] for xy in data]
y = [xy[1] for xy in data]
pylab.figure()
pylab.scatter(x, y)
pylab.show()
