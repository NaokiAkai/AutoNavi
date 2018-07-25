# Teaching-Playback Navigation Without Consistent Map

## How to use **nav_core** using the simulation.

How to build the part maps and register the target path.

Please open a terminal and run following commands to launch the simulator  
`$ source [AutoNavi]/ros/devel/setup.bash`  
`$ roslaunch robot_sim robot_sim.launch use_ground_truth_tf:=true moving_objects_num:=0`  
Please replace [AutoNavi] for adjusting your environment. You can control the simulated robot using the arrow keys and stop it by the space key.

Please then open a new terminal and run following commands  
`$ source [AutoNavi]/ros/devel/setup.bash`  
`$ roslaunch nav_core path_register.launch data_dir:=$(TARGET_DIRECTORY)`  
You need to prepare an empty directory and specify it as **$(TARGET_DIRECTORY)**.

During the map building operation, you can change the map by the following command  
`rosparam set /nav_core_path_register/change_map true`  
The built map and trajectory where the robot passed are recorded in the specified directory and the map building operation re-starts with an empty map. 

How to use the teaching-playback navigation using the simulation.

nav_coreの利用の仕方．


経路登録の仕方．
以下のコマンドでpath_registerを起動．
roslaunch nav_core path_register.launch data_dir:=$(TARGET_DIRECTORY)

経路登録（地図構築）中に以下のコマンドで地図を切り替える．
rosparam set /nav_core_path_register/change_map true

$(TARGET DIRECTORY)にデータが保存される．一番最後の地図も上のコマンドで保存することを忘れないように．





自律走行の仕方．
以下のコマンドを実行．
roslaunch nav_core auto_navi.launch data_dir:=$(TARGET_DIRECTORY)

以下のコマンドで停止を解除．path_registerで登録した経路を追従し始める．
rosparam set /path_follower/stop false
