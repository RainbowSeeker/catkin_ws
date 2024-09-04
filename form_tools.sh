#! /usr/bin/bash

workdir=$(pwd)

help() {
    echo "Usage: $0 [options] [arguments]"
    echo "Options:"
    echo "  -h, --help: Display this help message."
    echo "  -s, --sync: Sync the install directory in all machines."
    echo "  -e, --editor: Edit the params.yaml file."
    echo "  -p, --prelaunch: PreLaunch the formation on all machines."
    echo "  -l, --launch: Launch the formation on all machines."
    echo "  -k, --kill: Kill the formation on all machines."
}

sync() {
    IFS=$'\n'
    for line in $(cat machines.txt)
    do
        machine=$(echo $line | awk '{print $1}')
        password=$(echo $line | awk '{print $2}')
        echo "Sync the install directory to machine [$machine]..."
        sshpass -p "$password" rsync -avz --delete -e ssh $workdir/install/ $machine:$workdir/install/
    done
    IFS=
}

editor() {
    vim $workdir/src/formation/config/params.yaml
    colcon build --packages-select formation
}

prelaunch() {
    IFS=$'\n'
    base_run_cmd="ros2 launch formation mc_single_prelaunch.py"
    for line in $(cat machines.txt)
    do
        machine=$(echo $line | awk '{print $1}')
        password=$(echo $line | awk '{print $2}')
        amc_id=$(echo $machine | awk -F '.' '{print $NF}' | tail -c 2)
        run_cmd="$base_run_cmd amc_id:=$amc_id"
        echo "PreLaunch formation on machine [$machine]..."
        sshpass -p "$password" ssh $machine "source /opt/ros/humble/setup.bash; source $workdir/install/setup.bash; $run_cmd"
    done
    IFS=
}

launch() {
    IFS=$'\n'
    base_run_cmd="ros2 launch formation mc_single_launch.py"
    for line in $(cat machines.txt)
    do
        machine=$(echo $line | awk '{print $1}')
        password=$(echo $line | awk '{print $2}')
        amc_id=$(echo $machine | awk -F '.' '{print $NF}' | tail -c 2)
        run_cmd="$base_run_cmd amc_id:=$amc_id"
        echo "Launch formation on machine [$machine]..."
        sshpass -p "$password" ssh $machine "source /opt/ros/humble/setup.bash; source $workdir/install/setup.bash; $run_cmd"
    done
    IFS=
}

kill() {
    IFS=$'\n'
    for line in $(cat machines.txt)
    do
        machine=$(echo $line | awk '{print $1}')
        password=$(echo $line | awk '{print $2}')
        echo "Kill formation on machine [$machine]..."
        sshpass -p "$password" ssh $machine "pkill -f ros2"
    done
    IFS=
}

while [ "$1" != "" ]; do
    case $1 in
        -h | --help )           help
                                exit
                                ;;
        -s | --sync )           sync
                                exit
                                ;;
        -e | --editor )         editor
                                exit
                                ;;
        -p | --prelaunch )      prelaunch
                                exit
                                ;;
        -l | --launch )         launch
                                exit
                                ;;
        -k | --kill )           kill
                                exit
                                ;;
        * )                     help
                                exit 1
    esac
    shift
done