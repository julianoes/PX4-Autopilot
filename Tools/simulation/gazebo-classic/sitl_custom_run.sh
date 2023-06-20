#!/bin/bash
# Run two custom models within one sdf model.

function cleanup() {
	pkill -x px4
	pkill gzclient
	pkill gzserver
}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/../../.."

build_path=${src_path}/build/px4_sitl_default

export PX4_SIM_MODEL=gazebo-classic_custom

function spawn_model() {
	n=$1
	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	echo "starting instance $n"

	pushd "$working_dir" &>/dev/null
	$build_path/bin/px4 -i $n -d "$build_path/etc" &
	popd &>/dev/null
}

source ${src_path}/Tools/simulation/gazebo-classic/setup_gazebo.bash ${src_path} ${src_path}/build/px4_sitl_default

echo "Starting gazebo"
gzserver ${src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world --verbose &
sleep 3

gz model --verbose --spawn-file=Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/custom/custom.sdf --model-name=custom -x 0 -y 0 -z 0.83

spawn_model 1
spawn_model 2

trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient
