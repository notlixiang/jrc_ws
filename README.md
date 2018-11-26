## Prerequist:
1. Ubutnu 1604
2. ROS Kinetic Kame

3. librealsense2
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
4. others   
    ```  
    chmod +x install_dep_for_all.sh   
    ./install_dep_for_all.sh  
    ```  

## Change Log  
- 2018.11.06       Motion Planing部分需要单独catkin build， 与主工程分开， jrc_ws内部架构不变 （孙明静）
- 2018.11.07 19:44 成功汇入： 运动规划模块:`motion_planning_ws`（孙明静）， 末端执行器模块:`ur_ee_server`（李想）， AGV模块:`agv_server`（陈斌斌）

## Developer
agv_server: 黄先群，陈斌斌， 苗浩原；  
hangeye_calib;  杨理欣；   
jrc_main:  杨理欣，马灼明；   
jrc_srvs:  所有的.srv 和 .msg， 大家一起维护，按照规范来；   
pose_server: 马灼明，张昊若，倪培远；   
reco_server: 马灼明，姜文俊；   
universal_robot + ur_server : 孙明镜， 李想， 杨刚刚；   
ur_ee_server: 李想；   
 
## Build

```
git clone
cd jrc_ws
catkin_make
```
