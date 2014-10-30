#!/usr/bin/python
import sys
from subprocess import *

Popen(['java', '-jar', 'ihmcSimulator.jar']+list(sys.argv))
