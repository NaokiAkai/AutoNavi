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


	<arg name="base_link_frame" default="/base_link" />
	<arg name="input_max_vel_topic_name" default="/max_vel" />
	<arg name="output_twist_topic_name" default="/twist_cmd" />
	<arg name="look_ahead_dist" default="1.0" />
	<arg name="max_vel" default="1.0" />
	<arg name="kv" default="0.7" />
	<arg name="cmd_publish_hz" default="30.0" />
	<arg name="stop" default="true" />
	<arg name="use_nav_core_server" default="false" />
	<arg name="input_scan_topic_name" default="/scan" />
	<arg name="local_map_size_x" default="10.0" />
	<arg name="local_map_size_y" default="10.0" />
	<arg name="local_map_pixel_size" default="0.05" />
	<arg name="robot_width" default="1.0" />
	<arg name="predict_time" default="5.0" />
	<arg name="step_time" default="0.1" />
	<arg name="kv_avoid" default="0.8" />
	<arg name="w_range" default="0.4" />
	<arg name="use_emergency_stop" default="true" />
	<arg name="stop_x_size" default="0.5" />
	<arg name="stop_y_size" default="0.3" />
	<arg name="stop_time" default="3.0" />
	<arg name="back_time" default="5.0" />
	<arg name="back_speed" default="0.2" />

