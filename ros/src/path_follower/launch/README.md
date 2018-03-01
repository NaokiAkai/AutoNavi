path_follower.launch内の変数（パラメータ）

name: path_file, default: $(find path_follower)/../../maps/nic_garage/path.txt
使用する経路のファイル名を指定．

name: use_simple_obstacle_avoidance, default: true
障害物回避を使用するかどうかを決定するフラグ．

name: map_frame, default: /map
ワールド座標のフレーム名．

name: path_topic_name, default: /target_path
使用する経路のトピック名．

name: service_path_topic_name, default: /static_target_path
path_serverがサービスコールとして配信する経路のトピック名．

name: base_link_frame, default: /base_link
経路追従のために参照するフレーム．

name: input_max_vel_topic_name, default: max_vel
最大速度を外部のノードから設定できるように，path_followerが購読するトピック名．単位はm/second．

name: output_twist_topic_name, default: /twist_cmd
経路追従のために配信する速度指令のトピック名．型式はgeometry_msgs/TwistStamped．

name: look_ahead_dist, default: 1.0
経路追従を行う際に参照する点の距離．単位はm．この値が大きいと滑らかに経路を追従できるが，カーブの内側を走行するようになるので，バランスを見て決定すること．

name: max_vel, default: 1.0
経路追従のための最大速度．単位はm/second．

name: kv, default: 0.7
カーブのときなどに減速するためのパラメータ．

name: cmd_publish_hz, default: 30.0
経路追従をするための速度指令を配信する周波数．単位はHz．

name: stop, default: true
経路追従中に停止するかどうかを決定するフラグ．これをfalseにすると経路追従が始まる．

name: use_nav_core_server, default: false
nav_core_serverを使用するかどうかを決定するフラグ．

name: input_scan_topic_name, default: /scan
障害物回避に使用するレーザスキャンのトピック名．

name: local_map_size_x, default: 10.0
障害物回避に使用するローカル地図のx方向のサイズ．単位はm．

name: local_map_size_y, default: 10.0
障害物回避に使用するローカル地図のy方向のサイズ．単位はm．

name: local_map_pixel_size, default: 0.05
障害物回避に使用するローカル地図のピクセルのサイズ．単位はm．

name: robot_width, default: 1.0
障害物回避を行う際に考慮するロボットの横幅．単位はm．

name: predict_time, default: 5.0
障害物回避を行う際に現在の速度指令から将来の進路を予測する秒数．単位はsecond．

name: step_time, default: 0.1
将来の進路を予測する際の時間ステップ．単位はsecond．この値が大きすぎると衝突を検知できなくなる．

name: kv_avoid, default: 0.8
障害物回避を行う際の減速に関する値．

name: w_range, default: 0.4
障害物回避を行う際に考慮する角速度の変更範囲．単位はrad/second．

name: use_emergency_stop, default: true
指定領域に障害物が入った際に緊急停止をするかどうか決定するフラグ．

name: stop_x_size, default: 0.5
緊急停止する領域のx方向（ロボット横方向）に関するサイズ．単位はm．

name: stop_y_size, default: 0.3
緊急停止する領域のy方向（ロボット縦方向）に関するサイズ．単位はm．

name: stop_time, default: 3.0
緊急停止を継続する時間．単位はsecond．

name: back_time, default: 5.0
緊急停止が指定時間継続された場合に後退する時間．単位はsecond．

name: back_speed, default: 0.2
後退するときに速度．単位はm/second．
