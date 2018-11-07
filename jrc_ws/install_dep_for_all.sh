#//
#//                       _oo0oo_
#//                      o8888888o
#//                      88" . "88
#//                      (| -_- |)
#//                      0\  =  /0
#//                    ___/`---'\___
#//                  .' \\|     |// '.
#//                 / \\|||  :  |||// \
#//                / _||||| -:- |||||- \
#//               |   | \\\  -  /// |   |
#//               | \_|  ''\---/''  |_/ |
#//               \  .-\__  '-'  ___/-. /
#//             ___'. .'  /--.--\  `. .'___
#//          ."" '<  `.___\_<|>_/___.' >' "".
#//         | | :  `- \`.;`\ _ /`;.`/ - ` : | |
#//         \  \ `_.   \_ __\ /__ _/   .-` /  /
#//     =====`-.____`.___ \_____/___.-`___.-'=====
#//                       `=---='
#//
#//
#//     ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#//
#//               佛祖保佑         永无BUG
#//


echo "####################  FOR End Effector Server #########################"
echo " "
echo " "
echo " "
sudo apt-get install ros-kinetic-serial
echo " "
echo " "
echo " "


echo "####################  FOR AGV Server #########################"
echo " "
echo " "
echo " "

sudo -H apt-get install -y libnetpbm10-dev
sudo -H apt-get install -y ros-kinetic-bfl
sudo -H apt-get install -y libsdl1.2-dev
sudo -H apt-get install -y libsdl-image1.2-dev
sudo -H apt-get install -y ros-kinetic-move-base-msgs
sudo apt-get install ros-kinetic-laser-scan-matcher
sudo apt-get install ros-kinetic-laser-proc

echo " "
echo " "
echo " "








