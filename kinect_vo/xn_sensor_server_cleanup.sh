#!/usr/bin/env bash

trap 'echo "Killing XnSensorServer..."; DONE=1' 2

while true; do
  [[ -n $DONE ]] && break
  sleep 2
done

killall XnSensorServer
