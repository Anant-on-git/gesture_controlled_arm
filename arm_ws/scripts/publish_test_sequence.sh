#!/usr/bin/env bash

set -euo pipefail

MODE="${1:-ik}"
SLEEP_SECONDS="${SLEEP_SECONDS:-0.6}"

publish_vector() {
  local topic="$1"
  local x="$2"
  local y="$3"
  local z="$4"

  ros2 topic pub --once "$topic" geometry_msgs/msg/Vector3Stamped \
    "{header: {frame_id: 'map'}, vector: {x: ${x}, y: ${y}, z: ${z}}}" >/dev/null
}

sleep_step() {
  sleep "${SLEEP_SECONDS}"
}

run_ik_sequence() {
  echo "Publishing direct IK displacement sequence to /controller/displacement"
  publish_vector /controller/displacement 0.00 0.00 0.08
  sleep_step
  publish_vector /controller/displacement 0.08 0.00 0.00
  sleep_step
  publish_vector /controller/displacement 0.00 0.08 0.00
  sleep_step
  publish_vector /controller/displacement -0.05 -0.08 -0.04
  sleep_step
  publish_vector /controller/displacement 0.00 0.00 -0.04
  echo "IK sequence complete"
}

run_controller_sequence() {
  echo "Publishing synthetic IMU acceleration sequence to /imu/accel"
  publish_vector /imu/accel 0.00 0.00 9.81
  sleep_step
  publish_vector /imu/accel 0.00 0.00 9.81
  sleep_step
  publish_vector /imu/accel 2.80 0.00 9.81
  sleep_step
  publish_vector /imu/accel 2.80 0.00 9.81
  sleep_step
  publish_vector /imu/accel 0.00 0.00 9.81
  sleep_step
  publish_vector /imu/accel 0.00 -2.60 9.81
  sleep_step
  publish_vector /imu/accel 0.00 -2.60 9.81
  sleep_step
  publish_vector /imu/accel 0.00 0.00 9.81
  sleep_step
  publish_vector /imu/accel 0.00 0.00 12.80
  sleep_step
  publish_vector /imu/accel 0.00 0.00 12.80
  sleep_step
  publish_vector /imu/accel 0.00 0.00 9.81
  echo "Controller sequence complete"
}

case "${MODE}" in
  ik)
    run_ik_sequence
    ;;
  controller)
    run_controller_sequence
    ;;
  both)
    run_ik_sequence
    sleep_step
    run_controller_sequence
    ;;
  *)
    echo "Usage: $0 [ik|controller|both]" >&2
    exit 1
    ;;
esac
