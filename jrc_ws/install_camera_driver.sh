echo "####################  FOR Camera Driver (realsense2) #########################"
echo "####################  耗时较长 #########################"
echo " "
echo " "
echo " "
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE  
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u  
sudo apt-get update.  
sudo apt-get install librealsense2-dev
sudo apt-get install ros-kinetic-cv-bridge
echo " "
echo " "
echo " "
