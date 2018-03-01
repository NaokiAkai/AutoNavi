**Mobile Robot Localization Considering Class of Sensor Observations**

How to test the localization program using the simulation regarding the paper submitted to IROS 2018.

You need to run two launch scripts. Please first open a terminal and run following commands.
    source [AutoNavi]/ros/devel/setup.bash
    roslaunch robot_sim robot_sim.launch

Please then open a new terminal and run following commands.
    source [AutoNavi]/ros/devel/setup.bash
    roslaunch robot_sim robot_sim.launch
    roslaunch localizer amcl.launch use_dspd:=true

Finally, to visualize the localization process, please run a following command in a new terminal.
    rviz -d [AutoNavi]/ros/rviz.rviz 

You can control the simulated robot using the allow keys and stop it using the space key. Note that a small black window should be active when controlling the simulated robot.





amcl.launch内の変数（パラメータ）

name: map_yaml_file, default: $(find robot_sim)/../../maps/nic_garage/ogm.yaml"
使用する占有格子地図に対応するyamlファイルを指定．

name: map_frame, default: /map
ワールド座標の名前．

name: laser_frame, default: /laser
レーザセンサフレームの名前．

name: base_link_frame, default: /base_link
ロボットの車軸中心フレームの名前．

name: map_topic_name, default: /amcl_map"
amclが使用するマップのトピック名．

name: input_odom_topic_name, default: /odom
amclに入力されるオドメトリのトピック名．

name: input_scan_topic_name, default: /scan
amclに入力されるレーザスキャンのトピック名．

name: min_particle_num, default: 200
amclが使用する最低のパーティクル数．

name: max_particle_num, default: 500
amclが使用する最大のパーティクル数．

name: resample_threshold, default: 0.5
エフェクティブサンプルサイズ（Effective Sample Size）に対するリサンプリングの閾値．0より大きく1より小さい値を必ず指定．

name: scan_step, default: 10
パーティクルの尤度計算に使用するレーザスキャンをスキップする数．1の場合はすべてのスキャンが使用される．観測の独立性を確保した方が位置推定のロバスト性が向上するため，1は推奨しない．しかしながら，大きな値を選択すると使用するスキャンの数が少なくなり，そもそも位置推定の精度が下がる．このバランスを考慮して選択すること．

name: max_dist_to_obstacle, default: 0.5
尤度場を用いて位置推定する場合に考慮する障害物までの最大の距離．値を小さくしすぎることは推奨しないが，大きな値を使用することは問題にならない．しかしながら，大きな値を使用すると初期化の計算に時間がかかるので気をつけること．

name: alpha_slow, default: 0.0001
amclによる位置推定の失敗を検知するためのパラメータ（オススメの値はよくわからない...）．

name: alpha_fast, default: 0.1
amclによる位置推定の失敗を検知するためのパラメータ（オススメの値はよくわからない...）．

name: update_dist, default: 0.2
リサンプリングをするための移動距離の閾値．単位はm．

name: update_yaw, default: 2.0
リサンプリングをするための角度変化量の閾値．単位はdegree．

name: update_time, default: 5.0
リサンプリングをするための時間経過の閾値．単位はsecond．

name: odom_noise_dist_dist, default: 1.5
オドメトリの移動距離に対して，パーティクルの移動距離に加えるノイズの係数．

name: odom_noise_head_dist, default: 0.9
オドメトリの角度移動量に対して，パーティクルの移動距離に加えるノイズの係数．

name: odom_noise_dist_head, default: 0.9
オドメトリの移動距離に対して，パーティクルの角度移動量に加えるノイズの係数．

name: odom_noise_head_head, default: 2.5
オドメトリの角度移動量に対して，パーティクルの角度移動量に加えるノイズの係数．

name: start_x, default: 0.0
amclによる推定値x方向の初期値．単位はm．

name: start_y, default: 0.0
amclによる推定値y方向の初期値．単位はm．

name: start_yaw, default: 0.0
amclによる推定値yaw方向の初期値．単位はdegree．

name: initial_cov_xx, default: 0.2
初期のパーティクル群のx方向の分布に関する分散．単位はm^2．

name: initial_cov_yy, default: 0.2
初期のパーティクル群のy方向の分布に関する分散．単位はm^2．

name: initial_cov_yawyaw, default: 1.0
初期のパーティクル群のyaw方向の分布に関する分散．単位はdegree^2．

name: pose_publish_hz, default: 20.0
amclによる位置推定および位置推定結果を出力する周波数．単位はHz．

name: use_kld_sampling, default: true
カルバックライブラーダイバージェンス（KLD）に基づくリサンプリングを行うかどうか（パーティクル数が動的に変化するかどうか）を決定するフラグ．パーティクル数は，min_particle_numを下回ることはなく，max_particle_numを上回ることはない．

name: use_test_range_measurement, default: true
動的障害物（地図に存在しない障害物）の検出を行うかどうかを決定するフラグ．この動的障害物の検出は，パーティクルの尤度計算に使用するスキャン点にのみに対して行われ，動的障害物と判断されたスキャン点は尤度計算に使用されなくなる．すべてのスキャンに対して動的障害物かどうかの検出は行っていないため，「動的障害物としてパブリッシュされるトピック（/dynamic_scan_points）＝レーザスキャンに含まれるすべての動的障害物」として検出されているわけではないため注意すること．

name: dynamic_scan_point_threshold, default: 0.9
動的障害物を検出するための閾値．0以上1以下の値を使用すること．

name: use_nav_core_server, default: false
nav_coreを用いて地図切替えを含む自動走行を行うかどうかを決定するフラグ．詳細は後ほど追記予定．
