docker run -it \
  --gpus '"device=0,1"' \
  --shm-size=128g \
  -v /home/liji/catkin_ws/src/:/catkin_ws/src/ \
  -v /home/data/Dataset/fast_livo2:/catkin_ws/dataset/ \
  --env="DISPLAY=$DISPLAY"\
  --name fast_livo2 \
  fast_livo2:v2
#   ros:noetic-perception-focal
  