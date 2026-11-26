#!/usr/bin/env bash
set -eo pipefail

tools_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
export DASE_ROBOT_MODE=encoder
exec "$tools_dir/run_robot.sh" "$@"
