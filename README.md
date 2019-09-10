# welding-robot

cd src

git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..

git clone https://github.com/pal-robotics/ddynamic_reconfigure.git


git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver.git                 


