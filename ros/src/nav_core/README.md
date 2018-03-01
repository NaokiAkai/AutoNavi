nav_coreの利用の仕方．


経路登録の仕方．
以下のコマンドでpath_registerを起動．
roslaunch nav_core path_register.launch data_dir:=$(TARGET DIRECTORY)

経路登録（地図構築）中に以下のコマンドで地図を切り替える．
rosparam set /nav_core_path_register/change_map true

$(TARGET DIRECTORY)にデータが保存される．一番最後の地図も上のコマンドで保存することを忘れないように．





自律走行の仕方．
以下のコマンドを実行．
roslaunch nav_core auto_navi.launch data_dir:=$(TARGET DIRECTORY)

以下のコマンドで停止を解除．path_registerで登録した経路を追従し始める．
rosparam set /path_follower/stop false
