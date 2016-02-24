#! /bin/bash

java simulator.framework.Elevator -b 200 -fs 5 -monitor GradingMonitor -pf $1 2>&1| tee "$1.out"
