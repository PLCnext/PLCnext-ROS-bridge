#!/bin/sh
# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "$(date): \"${last_command}\" command filed with exit code $?."' EXIT

#set -x # for debugging

### BEGIN INIT INFO
# Provides:           @@@APPNAME@@@
# Required-Start:     $syslog $remote_fs
# Required-Stop:      $syslog $remote_fs
# Default-Start:      2 3 4 5
# Default-Stop:       0 1 6
### END INIT INFO

USER=root
CONTAINER_ENGINE=podman
COMPOSE_ENGINE=podman-compose
APP_NAME="@@@APPNAME@@@"
APP_HOME="/opt/plcnext/appshome"
APP_ID="@@@APPIDENTIFIER@@@"

##________Usefull environment variables________##

export APP_UNIQUE_NAME="@@@APPNAME@@@_@@@APPIDENTIFIER@@@" # unique AppName to use
export APP_PATH="/opt/plcnext/apps/${APP_ID}" # mountpoint for ro app file
export APP_TMP_PATH="/var/tmp/appsdata/${APP_ID}" # app temporary data storage
export APP_DATA_PATH="${APP_HOME}/data/${APP_ID}" # app persistent data storage
export APP_LOG="${APP_DATA_PATH}/${APP_NAME}.log" # logfile
export ROS_IP=$(ip -o addr show | awk '{print $4}' | cut -d / -f 1 | head -5 | tail +5)

#add path to app-binaries
export PATH=$PATH:$APP_HOME/bin

start () 
{
  if [ ! -e "$APP_LOG"  ]
  then
    touch $APP_LOG
  fi
  echo "$(date): Executing start()" >> $APP_LOG

  # When start() is executed then 
  # either 
  #   the image is not loaded and the container not started.
  #   Then load the docker image from the app directory and run container with 
  #   restart option to start it again after each boot of the controller.
  # or 
  #   the container is already loaded into the container cache. 
  #   It will be started by the container engine automatically.
 
  echo "$(date) Executing start()" >> $APP_LOG

 # Check app, install, run
  if [ -e "$APP_DATA_PATH/dockerapp_install" ]; then
    # app is already installed and container exists
    # just start it
    echo "$NAME is already installed. podman will start it automatically" >> $APP_LOG

    chmod -R 777 $APP_DATA_PATH
    cd $APP_DATA_PATH
    
    podman-compose down
    podman-compose up -d

  else
    echo "$NAME is not installed --> load images and install" >> $APP_LOG
    echo "Copy content" >> $APP_LOG
    cp -r $APP_PATH "$APP_HOME/data"
  
    chmod -R 777 $APP_DATA_PATH 

    echo "Load images" >> $APP_LOG
    podman load -i $APP_DATA_PATH/images/plcnext-ros-bridge.tar >> $APP_LOG 2>&1
    if [ ! $? -eq 0 ]
    then
      stop
      echo "$(date): Wasn't able to load  $APP_NAME." >> $APP_LOG
      exit 201
    fi

  # Remove tmp tar file from container load
    rm /media/rfs/rw/data/system/containers/docker-tar* >> $APP_LOG 2>&1
    if [ ! $? -eq 0 ]
    then
      echo "$(date): Wasn't able to remove with command: '$0'." >> $APP_LOG
    fi

    echo "Start compose" >> $APP_LOG
    cd $APP_DATA_PATH
    podman-compose up -d >> $APP_LOG 2>&1
    if [ ! $? -eq 0 ]
    then 
      stop 
      echo "$(date): Wasn't able to start podman compose." >> $APP_LOG
      exit 201
    fi
    
    # set app is installed
    touch $APP_DATA_PATH/dockerapp_install
    echo "Installation finished" >> $APP_LOG
  fi

  echo "$(date) start() finished" >> $APP_LOG
  
  # write Podman events to syslog
  logger -f /run/libpod/events/events.log
  rm -f /run/libpod/events/events.log
}

stop ()
{
  echo "$(date) Executing stop()" >> $APP_LOG
  # stop() is called when App is stopped by the AppManager e.g. via WBM
  # in this case the container needs to be stopped and removed from
  # the container cache. The goal is to keep the controller clean.
  # stop() is also called when the system will shutdown.
  # In this case the container should not be removed.
  # The container should just be started when the controller starts up again.
  if [ -e "$APP_DATA_PATH/dockerapp_install"  ]; then
    # Distinguish whether the controller is in shutdown phase
    # if not shutdown then the app is explicitly stopped -> remove container and image
    currentRunlevel=$(runlevel | cut -d ' ' -f2)
    echo current runlevel=$currentRunlevel >> $APP_LOG

    if [ "$currentRunlevel" -ne "6" ]; then
      echo "Stop $NAME" >> $APP_LOG
      cd $APP_DATA_PATH
      echo "Remove compose and all used images."
      podman-compose down 
      podman image rm --force §§IMAGE_ID§§
      rm $APP_DATA_PATH/dockerapp_install
    fi
  else
    echo "Stop $NAME: container not installed" >> $APP_LOG
    # the container is not installed.
    # nothing to be done
  fi
  echo "$(date) stop() finished" >> $APP_LOG

  # write Podman events to syslog
  logger -f /run/libpod/events/events.log
  rm -f /run/libpod/events/events.log
}

case "$1" in
  start)
    start
    ;;
  stop)
    stop
    ;;
  restart)
    stop
    start
    ;;
esac