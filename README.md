## Prerequist:
1. `sudo apt-get install ros-kinetic-serial`
2. librealsense2:
```  
// follow instruction:  
// https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
// Or:
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE  
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u  
sudo apt-get update.  
sudo apt-get install librealsense2-dev
//  Wait Until Finished
```

## Build

```
git clone
cd jrc_ws
catkin_make
```
