#!/bin/bash

DCF=docker-compose.yml
XSERVER=""

export ROS_IP=127.0.0.1
export ROBOT_TYPE="stage"
export DEMO_DIR=`pwd | gawk '{ print gensub(/\/bin/, "", 1) }'`

if [ "$1" == "dev" ]; then
  DCF=docker-compose-dev.yml
  echo "Starting in dev mode"
  sleep 3
fi
# if [ "$1" == "vnc" ]; then
  # DCF=docker-compose-vnc.yml
  # XSERVER=xserver
# fi


# if [ "$1" == "dev" ]; then
  # # run docker services
  # docker-compose -f $DCF up -d $XSERVER stage navigation speech vision \
    # stagepersondetection actions pnp
# else
  # # pull docker services
  # docker-compose -f $DCF pull $XSERVER stage navigation speech vision  \
    # stagepersondetection actions pnp persondetection

  # # run docker services
  # docker-compose -f $DCF up -d $XSERVER stage navigation speech vision \
    # stagepersondetection actions pnp persondetection
# fi

# By using the parameter "--rmi all" all the images that comes from the services are being deleted and pulled again (latest version)
if [ "$2" == "rmi" ]; then
  docker-compose down --rmi all

elif [ "$2" == "local" ]; then
	docker-compose down --rmi local

else
# pull docker services
  docker-compose -f $DCF pull $XSERVER stage navigation speech vision  \
     pnp persondetection facedetection fer actions
fi

  # run docker services
  docker-compose -f $DCF up -d $XSERVER stage navigation speech vision \
     pnp persondetection facedetection fer actions


sleep 10

docker ps

sleep 3

# Stage with map
echo 'DISB1;marrtino' | netcat -w 1 localhost 9235
sleep 5

# Navigation
echo '@loc' | netcat -w 1 localhost 9238
sleep 3
echo '@movebasegbn' | netcat -w 1 localhost 9238
sleep 3

# Speech

echo '@audio' | netcat -w 1 localhost 9239
sleep 3

# Vision  (use marrtino as robot in stage)

echo '@takephoto' | netcat -w 1 localhost 9237
sleep 3

echo '@usbcam' | netcat -w 1 localhost 9237
sleep 3


# check
docker exec -it stage bash -ci "rosnode list"


