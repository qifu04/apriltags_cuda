#! /bin/bash

# functions I need for AprilTags that could be used by the manager or the other script in bin

# get proc pid (0 notfound/error, else is the pid)
function getProcPID() {
    # NOTE: this returns the OLDEST PID
    paths=$(ps -aux | grep $1 | grep -v 'grep' | grep -v 'libAprilTags.sh pid') #last section removes command syntax for this command
    # the quotes preserve linebreaks
    # rewriting the for loop, since its done in a subshell and causes issues when written normally
    while read line; do
        pid=$(echo $line | awk '{ printf $2}')
        re='^[0-9]+$'
        if [[ $pid =~ $re ]]; then
            echo $pid
            return
        fi
    done <<< "$(echo "$paths")"
    echo 0
}

if [[ $1 == "pid" ]]; then
    getProcPID $2
elif [[ $1 == "camIDs" ]]; then
    ret=/apps/bin/scanner
    if [[ "${ret,,}" == *"err"* ]]; then
    	echo "there was an error..."
    	return 1
    elif ! [[ "${ret:0:1}" == "[" ]] || ! [[ "${ret:-1:1}" == "]" ]]; then
        echo "what kind of array is this????"
        return 1
    fi
    # all good
    echo "${ret:1:-1}"
    return 0
elif [[ $1 == "getCamLoc" ]]; then
    camlocfile="/apps/AprilTags/data/cameralocations"
    camid=$2
    if ! [[ -f camlocfile ]]; then
        echo "cam locations file not found."
        return 1 # maybe stop using the same things
    fi
    # must have the file, assume that the file is correct
    set -e # just in case
    locfile=$(tail -n +1 camlocfile)
    for line in locfile; do
        if [[ "$2" == "${line[0]}" ]]; then
            echo line[1]
            return 0
        fi
    done
    echo "id not found"
    return 1 # again, same code
fi
