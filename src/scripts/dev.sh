#!/usr/bin/env bash
# Run from any working directory. Build outputs never belong in the source tree.
set -eo pipefail

crx_source_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
crx_workspace_dir="$(dirname -- "$crx_source_dir")"
crx_output_dir="${CRX_OUTPUT_DIR:-$crx_workspace_dir}"
crx_ros_setup="/opt/ros/humble/setup.bash"
crx_packages=(crx_description crx_control crx_moveit_config crx_bringup crx_manipulation)

usage() {
    cat <<'HELP'
Usage: bash src/scripts/dev.sh <build|test|check|sim|demo|pick> [launch arguments...]
  build   Build the simulation packages with tests enabled.
  test    Run registered tests and print their results (build first).
  check   Build and test.
  sim     Launch Gazebo only.
  demo    Launch Gazebo + MoveIt + RViz.
  pick    Launch the fixed cube A-to-B grasp demonstration.
CRX_OUTPUT_DIR selects an absolute directory for build/install/log outputs.
Example: bash src/scripts/dev.sh demo headless:=true rviz:=false
HELP
}

crx_command="${1:-help}"
case "$crx_command" in
    help|-h|--help) usage; exit 0 ;;
    build|test|check|sim|demo|pick) shift ;;
    *) usage >&2; exit 2 ;;
esac
if [[ "$crx_output_dir" != /* ]]; then
    echo "CRX_OUTPUT_DIR must be an absolute path." >&2
    exit 2
fi
if [[ ! -f "$crx_ros_setup" ]]; then
    echo "ROS 2 Humble is required: $crx_ros_setup" >&2
    exit 1
fi
source "$crx_ros_setup"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="${ROS_LOG_DIR:-$crx_output_dir/log/ros}"

build() {
    colcon --log-base "$crx_output_dir/log" build \
        --base-paths "$crx_source_dir" \
        --build-base "$crx_output_dir/build" --install-base "$crx_output_dir/install" \
        --symlink-install --packages-select "${crx_packages[@]}" \
        --cmake-args -DBUILD_TESTING=ON
}

load_overlay() {
    if [[ ! -f "$crx_output_dir/install/local_setup.bash" ]]; then
        echo "Build first: bash $crx_source_dir/scripts/dev.sh build" >&2
        exit 1
    fi
    source "$crx_output_dir/install/local_setup.bash"
}

test_project() {
    load_overlay
    local crx_test_status=0
    colcon --log-base "$crx_output_dir/log" test \
        --base-paths "$crx_source_dir" --build-base "$crx_output_dir/build" \
        --install-base "$crx_output_dir/install" --packages-select "${crx_packages[@]}" \
        --return-code-on-test-failure || crx_test_status=$?
    colcon test-result --test-result-base "$crx_output_dir/build" --verbose || crx_test_status=$?
    return "$crx_test_status"
}

case "$crx_command" in
    build|test|check)
        if [[ $# -ne 0 ]]; then usage >&2; exit 2; fi
        case "$crx_command" in
            build) build ;;
            test) test_project ;;
            check) build; test_project ;;
        esac
        ;;
    sim|demo|pick)
        load_overlay
        if [[ "$crx_command" == sim ]]; then
            exec ros2 launch crx_bringup gazebo_simulation.launch.py "$@"
        elif [[ "$crx_command" == pick ]]; then
            exec ros2 launch crx_bringup cube_pick_place.launch.py "$@"
        else
            exec ros2 launch crx_bringup gazebo_moveit.launch.py "$@"
        fi
        ;;
esac
