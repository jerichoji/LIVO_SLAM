docker run -it \
  --rm \
  --gpus all \
  --volume/tmp/.X11-unix:/tmp/.X11-unix \
  --network=host \
  --shm-size=32g \
  -v /home/a/LIVO_SLAM/src/:/catkin_ws/src/ \
  --env="DISPLAY=$DISPLAY"\
  --name fast_livo2 \
  fast_livo2:latest
  