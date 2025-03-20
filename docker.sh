docker run -it \
  --rm \
  --gpus all \
  --shm-size=32g \
  -v ~/LIVO_SLAM/src/:/catkin_ws/src/ \
  # -v /home/data/Dataset/fast_livo2:/catkin_ws/dataset/ \
  --env="DISPLAY=$DISPLAY"\
  --name fast_livo2 \
  fast_livo2:latest
  