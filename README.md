# welding-robot
 
需要git clone 默认package

cd src

git clone https://github.com/IntelRealSense/realsense-ros.git   

cd realsense-ros/  然后delete realsense2_description文件夹

git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`


cd src


git clone https://github.com/pal-robotics/ddynamic_reconfigure.git


git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git                 


