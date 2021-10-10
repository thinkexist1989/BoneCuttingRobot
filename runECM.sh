#!/bin/sh
cd /opt/ECMworkspace_64
#nice -n -20 ./y2 -f eni.xml -i8254x 6 1 -b 1000 -v 3 -perf -t 0 -sp 6000 &
nice -n -20 ./y2 -f eni.xml -i8254x 6 1 -b 1000 -v 3 -perf -t 0 -sp 6000 > output.txt
