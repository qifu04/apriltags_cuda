#! /bin/bash

function help() {
    echo "#### AprilTags Helper ####"
    echo "-h to print this menu"
    echo "-v to print verbose"
    echo "--validate or -V {option} to validate something"
    echo "--update or -u to update the running frontend/backend from the cuda root"
    echo "ex. -V lockfile"
    echo
    echo "if you try to run two different commands in one command, one might exit before the other"
    echo "if that happens, just seperate the commands, mk?"
    exit 0
}

if [ $# -eq 0 ]; then
    # print help
    help
fi

function ensureRoot() {
    if [ $EUID -ne 0 ]; then
        echo "The script you are trying to run requires root access, and this does not have it"
        echo "please rerun this script with root!"
        exit 4
    fi
}

# print if verbose is on
function printV() {
    if [[ $verbose == "t" ]]; then
        echo $1
    fi
}

# arg parser
while [[ $# -gt 0 ]]; do
  case $1 in
    -v)
      verbose="t"
      shift # past argument
      ;;
    -h)
      help
      ;;
    -V|--validate)
      if [ ! -z ${valitem+x} ]; then
        echo "an item has already been set to be validated, this script can not handle two"
        exit 2
      elif [ ! -n $2 ]; then
        echo "no argument specified after validate arg"
        exit 3
      fi
      valitem="$2"
      shift # past argument
      shift # past value
      ;;
    -u|--update)
      #simply sets a thing to say it should update lol
      ensureRoot
      update="t"
      shift # past arguement
      ;;
    *|-*|--*)
      echo "Unknown option $1"
      exit 1
      ;;
  esac
done

# validation checks
if [ ! -z ${valitem+x} ]; then
    printV "validating arg $valitem..."
    if [[ $valitem == "lockfile" ]]; then
        # check for the existance of the lock
        PID=$(libAprilTags.sh pid "/apps/AprilTags/Backend/ws_server")
        printV "BackendPID is $backPID. (zero is not running)"
        if [ -f /apps/AprilTags/servicerunning ]; then
            # lockfile is there
            printV "lockfile present, checking if backend is being run"
            if [ $PID -ne 0 ]; then
                # echo "lockfile=true;running=true;"
                printV "The service appears to be running and stable."
            else
                rm /apps/AprilTags/servicerunning
                if [ $? -ne 0 ]; then
                    # echo "lockfile=rm;running=false;"
                    printV "Lockfile found with the service not running, it could not be deleted, feel free to delete it."
                else
                    # echo "lockfile=false;runnning=false;"
                    printV "Lockfile found with no services running, it is now deleted."
                fi
            fi
        else
            # lockfile missing, check if anything is runing
            printV "lockfile is not there"
            if [ $PID -eq 0 ]; then
                # echo "lockfile=false;running=false;"
                printV "The AprilTags service is offline and the lockfile is not there."
            else
                printV "The lockfile is missing and at least one of the services is running."
                touch /apps/AprilTags/servicerunning
                if [ $? -ne 0 ]; then
                    # echo "lockfile=add;running=true;"
                    printV "The lockfile couldn't be created. please make it, or stop the processes!"
                    printV "Go to the ApriltagsManager.sh script and run it with 'stop'"
                else
                    # echo "lockfile=true;running=true;"
                    printV "Service is online, the lockfile is now in place."
                fi
            fi
        fi
    else
        printV "Unknown option. Please specify the correct option"
        exit 11
    fi
# update checks
elif [ $update == "t" ]; then
    # validate that we are inside of the project root
    # note, below will run inside of the directory that the command is being called in
    if [ ! -f build/ws_server ]; then
        printV "This is not the root of the repo, exiting.."
        printV "HINT: This can be an issue using sudo -i, meaning you need to use the script absolute path without the -i in sudo"
        exit 21
    fi

    # disable the service to make sure stuff is ok
    systemctl stop AprilTagsPipeline.service
    # just as a check, make sure that the lockfile is also gone
    rm /apps/AprilTags/servicerunning

    # make sure that the dirs exist. why did I not do this before
    if [ ! -d /apps/AprilTags/Backend ]; then
    	mkdir /apps/AprilTags/Backend
    fi
    
    printV "copying backend..."
    cp -R build/* /apps/AprilTags/Backend/
    
    # remove the existing data dir if it exists then replace it
    if [ -d /apps/AprilTags/data ]; then
    	rm -rf /apps/AprilTags/data
    fi
    if [ -d data ]; then
    	cp -R data/ /apps/AprilTags/data/
    fi
    
    printV "The files were sucessfully coppied"

    # restart the service
    systemctl start AprilTagsPipeline.service

# end of the args and stuffs
fi
