#! /bin/bash

if [ $# -ne 1 ]; then
    echo "only run with one argument"
    exit 1
fi

if [ ! -f '/apps/AprilTags/Backend/ws_server' ]; then
    echo "missing backend"
    exit 2
fi

function killIfRunning() {
    PID=`libAprilTags.sh pid $1`
    if [ $PID -ne 0 ]; then
        kill -2 $PID
        return 1
    fi
    return 0
}

# set clocks to max frequency (jetson only)
jetson_clocks || true

#open source for args
source /apps/AprilTags/args

# make sure that the bin dir is actually loaded
PATH="$PATH:/apps/bin"

if [[ $1 == "start" ]]; then
    # check for lock
    if [ -f "/apps/AprilTags/servicerunning" ]; then
        echo "service is already running"
        echo "HINT: if you think the service is not running, then run 'AprilTags.sh --validate lockfile'"
        # now check if it is allowed to remove the lockfile..
        if [[ $autormlockfile == "true" ]]; then
        	# check for if the services are fine
        	status=$(/apps/bin/AprilTags.sh -V lockfile)
        	lockfilestatus=$(echo $status | awk -F ";" '{print $1}' | awk -F "=" '{print $2}')
        	if [[ $lockfilestatus == "false" ]]; then
        		# the lockfile is gone, and nothing is wrong
        		echo "removed old lockfile"
        	else
        		echo "the service is still running... exiting"
        		echo "Specific status: ${lockfilestatus}"
        		exit 4
        	fi
        else
        	# either unable to remove the lock or it is running already
        	exit 4
        fi
    fi

    # to fix some odd pathing things
    cd /apps/AprilTags

    touch /apps/AprilTags/servicerunning
    
    cams=`libAprilTags.sh camIDs`
    retcode=$?
    
    # cd again just to be safe (that lib WILL cd)
    cd /apps/AprilTags
    
    if [[ $recode -ne 0 ]]; then
    	echo "No cams found or something. idk, but no cams, so die"
    	exit 5
    fi
    # iterate
    for cam in $cams; do
        # set items off of the output cam
        IFS=: read -r camID camIndex <<< "$s"
        
        # set args
        args=$backend # this actually is magical dwdw about it
        # https://stackoverflow.com/questions/5112663/bash-variable-reevaluation
        
        # abs path BECAUSE of the proc getting commands
        /apps/AprilTags/Backend/ws_server $args &
        echo $! >> /apps/AprilTags/servicerunning # add every pid to a new line
    done
    
    exit

elif [[ $1 == "stop" ]]; then
    if [[ -f /apps/AprilTags/servicerunning ]]; then
    	# read the file, kill if a pid is found inside
    	while read p; do
    	    if [[ "$p" =~ ^-?[0-9]+$ ]]; then # regex to find int
    	    	# assume its a PID that belongs to ws_server if running
    	    	if ps -p $p > /dev/null
    	    	then
    	    	    kill -2 $p
    	        fi
    	    fi
    	done < /apps/AprilTags/servicerunning
    fi
    # and add the rest of it
    #killIfRunning '/apps/AprilTags/Backend/ws_server'
    while killIfRunning '/apps/AprilTags/Backend/ws_server'; do
    	continue # just run the initial condition, a backup if the service orphaned something
    done
    
    rm /apps/AprilTags/servicerunning
else
    echo "input ${1} not understood"
    exit 3
fi
