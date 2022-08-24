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

##________APP configuration________##

# specify image archives and their accociated IDs in an array
# IMAGES[<image_ID>]=<image_archive>
declare -A IMAGES
IMAGES[§§IMAGE_ID§§]=plcnext-hello-world.tar.gz
# add all volumes to make them accessible to the container users and PLCnext admin (IDs 1002:1002)
# Space separated list e.g. VOLUMES=("${APP_DATA_PATH}/test1" "${APP_DATA_PATH}/test2")
declare -a VOLUMES=( "${APP_DATA_PATH}/www" )

##________Do not change the code below!________##

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
 
  # copy user configuration file to the app persistent storage if needed
  if [ ! -e "${APP_DATA_PATH}/user.env"  ]
  then
    echo "$(date): Creating  ${APP_DATA_PATH}/user.env" >> $APP_LOG
    cp -p ${APP_PATH}/user.env ${APP_DATA_PATH}
  fi

  for VOLUME in "${VOLUMES[@]}"
  do
    if [ ! -d $VOLUME ]
    then
      echo "$(date): Creating $VOLUME" >> $APP_LOG
      mkdir $VOLUME
      if [ ! $? -eq 0 ]
      then
        stop
        echo "$(date): Creating $VOLUME failed" >> $APP_LOG
        exit 101
      fi
      echo "$(date): CHOWN admin:plcnext for $VOLUME" >> $APP_LOG
      chown 1002:1002 $VOLUME
      if [ ! $? -eq 0 ]
      then
        stop
        echo "$(date): CHMOD for $VOLUME failed" >> $APP_LOG
        exit 102
      fi
    fi
  done

  # check for image, otherwise load image
  for IMAGE_NAME in "${!IMAGES[@]}"
  do
    echo "$(date): Checking image $IMAGE_NAME" >> $APP_LOG
    if ! $CONTAINER_ENGINE image exists $IMAGE_NAME
    then
      echo "$(date): $IMAGE_NAME is not available in cache and will now be loaded" >> $APP_LOG
      $CONTAINER_ENGINE load -i ${APP_PATH}/images/${IMAGES[$IMAGE_NAME]} >> $APP_LOG 2>&1
      if [ ! $? -eq 0 ]
      then
        stop
        echo "$(date): Wasn't able to load  $IMAGE_NAME from ${IMAGES[$IMAGE_NAME]}" >> $APP_LOG
        exit 201
      fi
    fi
  done

  if $CONTAINER_ENGINE pod exists pod_${APP_UNIQUE_NAME}
    then
      echo "$(date): Restarting $APP_NAME" >> $APP_LOG
      # Compose START
      $COMPOSE_ENGINE -f ${APP_PATH}/app-compose.yml --env-file ${APP_DATA_PATH}/user.env -p ${APP_UNIQUE_NAME} up -d --force-recreate >> $APP_LOG 2>&1
    else 
      # Compose UP
      echo "$(date): Starting $APP_NAME" >> $APP_LOG
      $COMPOSE_ENGINE -f ${APP_PATH}/app-compose.yml --env-file ${APP_DATA_PATH}/user.env -p ${APP_UNIQUE_NAME} up -d >> $APP_LOG 2>&1
  fi
  echo "$(date): start() finished" >> $APP_LOG

  # write Podman events to syslog
  logger -f /run/libpod/events/events.log
  rm -f /run/libpod/events/events.log
}

stop ()
{
  echo "$(date): Executing stop()" >> $APP_LOG
  # stop() is called when App is stopped by the AppManager e.g. via WBM
  # in this case the container needs to be stopped and removed from 
  # the container cache. The goal is to keep the controller clean.
  # stop() is also called when the system will shutdown.
  # In this case the container should not be removed. 
  # The container should just be started when the controller starts up again.

    # Distinguish whether the controller is in shutdown phase
    # if not shutdown then the app is explicitly stopped -> remove container and image
  currentRunlevel=$(runlevel | cut -d ' ' -f2)
  echo "$(date): current runlevel=${currentRunlevel}" >> $APP_LOG

  
  if [ "$currentRunlevel" -ne "6" ]
  then
  # User pressed Stop
    echo "$(date): Stoping pod_${APP_UNIQUE_NAME} " >> $APP_LOG
    $COMPOSE_ENGINE -f ${APP_PATH}/app-compose.yml --env-file ${APP_DATA_PATH}/user.env -p ${APP_UNIQUE_NAME} down >> $APP_LOG 2>&1
    echo "$(date): Remove network ${APP_UNIQUE_NAME}_default" >> $APP_LOG
    $CONTAINER_ENGINE network rm ${APP_UNIQUE_NAME}_default >> $APP_LOG 2>&1
    echo "$(date): Remove image(s)" >> $APP_LOG
    for IMAGE_NAME in "${!IMAGES[@]}"
    do
      if $CONTAINER_ENGINE image exists $IMAGE_NAME
      then
        $CONTAINER_ENGINE rmi $IMAGE_NAME >> $APP_LOG 2>&1
        if [ ! $? -eq 0 ]
        then
          echo "$(date): Wasn't able to delete $IMAGE_NAME, $IMAGE_NAME still in use!"  >> $APP_LOG
        else
          echo "$(date): $IMAGE_NAME deleted" >> $APP_LOG
        fi
      else
        echo "$(date): Wasn't able to delete $IMAGE_NAME, $IMAGE_NAME not available!"  >> $APP_LOG
      fi
    done
  else
  # System shutdown
    $COMPOSE_ENGINE -f ${APP_PATH}/app-compose.yml --env-file ${APP_DATA_PATH}/user.env -p ${APP_UNIQUE_NAME} down >> $APP_LOG 2>&1
  fi
  echo "$(date): stop() finished" >> $APP_LOG
  
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
