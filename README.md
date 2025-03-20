
```bash
git clone https://gitee.com/jerichoji/LIVO_SLAM.git
docker pull crpi-6bf8f4ebks7rgb7z.cn-beijing.personal.cr.aliyuncs.com/jerichoji/fast_livo2:latest
# run
cd LIVO_SLAM
bash docker.sh
cd /catkin_ws/ && catkin_make
roslaunch fast_livo mapping_avia.launch
rosrun rslidar_to_livox rslidar_to_livox_node
```
