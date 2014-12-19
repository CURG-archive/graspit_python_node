#!/bin/bash
GRASPIT_SSH_URL=$1
DISPLAY=$2
ssh $GRASPIT_SSH_URL -t "export DISPLAY=$DISPLAY;bash -ic graspit"
