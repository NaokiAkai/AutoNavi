#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <robot_sim/ScanObjectID.h>
#include <localizer/amcl.h>

AMCL* amcl;
bool use_nav_core_server = false;
std::vector<robot_sim::ScanObjectID> scan_object_ids;

// 実験終わったら消してOK
void for_iros(sensor_msgs::LaserScan scan)
{
	double norm_const = 1.0 / sqrt(amcl->z_hit_denom * M_PI);
	double max;
	pose_t pose = amcl->robot_pose;
	FILE* fp;
	double range = 0.8, resolution = 0.08;
	bool use_bayes = false; // for prior distribution regarding class, p(c)
	double zz_max = amcl->z_hit * amcl->norm_const_hit + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
	double z_uni = 1.0 / scan.range_max / amcl->map.info.resolution;
	// レーザスキャンと地図をプロット
	{
		amcl->save_map_as_txt_file("/tmp/amcl_map.txt");
		fp = fopen("/tmp/laser.txt", "w");
		double c = cos(pose.yaw);
		double s = sin(pose.yaw);
		double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + pose.x;
		double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + pose.y;
		for (int i = 0; i < scan.ranges.size(); i++)
		{
			double angle = scan.angle_min + scan.angle_increment * (double)i;
			double yaw = angle + pose.yaw;
			double r = scan.ranges[i];
			double x = r * cos(yaw) + xo;
			double y = r * sin(yaw) + yo;
			fprintf(fp, "%lf %lf\n", x, y);
		}
		fclose(fp);
	}
	// 各手法で尤度を計算
	for (int i = 0; i < 6; i++)
	{
		if (i == 0) // すべてのスキャンを使って尤度計算
		{
			amcl->use_test_range_measurement = false;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "w");
		}
		else if (i == 1) // 提案法
		{
			amcl->use_test_range_measurement = false;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "w");
			use_bayes = false;
		}
		else if (i == 2) // パーティクルを用いた動的なスキャン点の棄却（確率ロボティクスに書いてある）
		{
			amcl->use_test_range_measurement = true;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "w");
		}
		else if (i == 3) // ビームモデル
		{
			amcl->use_test_range_measurement = false;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "w");
		}
		else if (i == 4) // ビームモデル＋パーティクルを用いた動的なスキャン点の棄却
		{
			amcl->use_test_range_measurement = true;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "w");
		}
		else if (i == 5) // 提案法（ベイズ推定あり）
		{
			amcl->use_test_range_measurement = false;
			amcl->check_scan_points_validity(scan);
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "w");
			use_bayes = true;
		}
		for (double dx = -range; dx <= range; dx += resolution)
		{
			for (double dy = -range; dy <= range; dy += resolution)
			{
				double w = 0.0;
				double c = cos(pose.yaw);
				double s = sin(pose.yaw);
				double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + pose.x + dx;
				double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + pose.y + dy;
				double ssp_prob = 0.0, dsp_prob = 0.0;
				for (int j = 0; j < scan.ranges.size(); j++)
				{
					if (!amcl->is_valid_scan_points[j])
					{
						double zz = (amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult);
						if (use_bayes)
						{
							ssp_prob = zz * 0.5;
//							dsp_prob = (1.0 - zz) * 0.5;
							dsp_prob = z_uni * (zz_max - zz) * 0.5;
							double sum = ssp_prob + dsp_prob;
							ssp_prob /= sum;
							dsp_prob /= sum;
						}
						else
						{
							ssp_prob = (amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult);
//							dsp_prob = 1.0 - ssp_prob;
							dsp_prob = z_uni * (zz_max - ssp_prob);
						}
						amcl->dsp_probs[i][j] = dsp_prob;
						if (j % amcl->scan_step == 0)
						{
							double pz;
							if (i == 0 || i == 2 || i == 3 || i == 4)
							{
								pz = amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
								if (i == 3 || i == 4)
									pz += amcl->z_short * (1.0 / (1.0 - exp(-amcl->lambda_short * scan.range_max))) * amcl->lambda_short * exp(-amcl->lambda_short * scan.range_max);
							}
							else
							{
								double pzs = zz * ssp_prob;
//								double pzd = (1.0 - zz) * dsp_prob;
								double pzd = z_uni * (zz_max - zz) * dsp_prob;
								pz = pzs + pzd;
							}
							if (pz > 1.0)
								pz = 1.0;
							w += log(pz);
						}
						continue;
					}
					double angle = scan.angle_min + scan.angle_increment * (double)j;
					double yaw = angle + pose.yaw;
					double r = scan.ranges[j];
					double x = r * cos(yaw) + xo;
					double y = r * sin(yaw) + yo;
					double pz = 0.0;
					int u, v;
					amcl->xy2uv(x, y, &u, &v);
					double range_ = -1.0;
					if (i == 3 || i == 4)
					{
						x = xo;
						y = yo;
						double dx = amcl->map.info.resolution * cos(yaw);
						double dy = amcl->map.info.resolution * sin(yaw);
						for (double rr = 0.0; rr <= scan.range_max; rr += amcl->map.info.resolution)
						{
							amcl->xy2uv(x, y, &u, &v);
							if (0 <= u && u < amcl->map.info.width && 0 <= v && v < amcl->map.info.height)
							{
								int node = v * amcl->map.info.width + u;
								if (amcl->map.data[node] == 100)
								{
									range_ = rr;
									break;
								}
							}
							else
							{
								break;
							}
							x += dx;
							y += dy;
						}
						x = r * cos(yaw) + xo;
						y = r * sin(yaw) + yo;
						amcl->xy2uv(x, y, &u, &v);
					}
					if (range_ < 0.0)
						range_ = scan.range_max;
					double pzs = 0.0, pzd = 0.0; // s -> static, d -> dynamic
					ssp_prob = 0.0, dsp_prob = 0.0; // ssp -> static scan point, dsp -> dynamic scan point
					if (0 <= u && u < amcl->map.info.width && 0 <= v && v < amcl->map.info.height)
					{
//						double z = amcl->dist_map[u][v];
						double z = amcl->dist_map.at<float>(v, u);
						double zz = amcl->z_hit * amcl->norm_const_hit * exp(-(z * z) / amcl->z_hit_denom) + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
						if (i == 3 || i == 4)
						{
							zz = (1.0 - amcl->z_max - amcl->z_rand - amcl->z_short) * amcl->norm_const_hit * exp(-(z * z) / amcl->z_hit_denom) + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
							if (r <= range_)
								zz += amcl->z_short * (1.0 / (1.0 - exp(-amcl->lambda_short * range_))) * amcl->lambda_short * exp(-amcl->lambda_short * r);
						}
//						if (zz > 1.0)
//							zz = 1.0;
						if (use_bayes)
						{
							ssp_prob = zz * 0.5;
//							dsp_prob = (1.0 - zz) * 0.5;
							dsp_prob = z_uni * (zz_max - zz) * 0.5;
							double sum = ssp_prob + dsp_prob;
							ssp_prob /= sum;
							dsp_prob /= sum;
						}
						else
						{
							ssp_prob = zz;
							dsp_prob = 1.0 - zz;
						}
						pzs += zz * ssp_prob;
//						pzd += (1.0 - zz) * dsp_prob;
						pzd += z_uni * (zz_max - zz) * dsp_prob;
						if (i == 0 || i == 2 || i == 3 || i == 4)
							pz = zz;
						else
							pz = pzs + pzd;
					}
					else
					{
						double zz = amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
						if (i == 3 || i == 4)
						{
							if (r <= range_)
								zz += amcl->z_short * (1.0 / (1.0 - exp(-amcl->lambda_short * range_))) * amcl->lambda_short * exp(-amcl->lambda_short * r);
						}
						if (use_bayes)
						{
							ssp_prob = zz * 0.5;
//							dsp_prob = (1.0 - zz) * 0.5;
							dsp_prob = z_uni * (zz_max - zz) * 0.5;
							double sum = ssp_prob + dsp_prob;
							ssp_prob /= sum;
							dsp_prob /= sum;
						}
						else
						{
							ssp_prob = zz;
							dsp_prob = 1.0 - zz;
						}
						pzs += zz * ssp_prob;
//						pzd += (1.0 - zz) * dsp_prob;
						pzd += z_uni * (zz_max - zz) * dsp_prob;
						if (i == 0 || i == 2 || i == 3 || i == 4)
							pz = ssp_prob;
						else
							pz = pzs + pzd;
					}
					amcl->dsp_probs[i][j] = dsp_prob;
					if (j % amcl->scan_step == 0)
					{
						if (pz > 1.0)
							pzs = 1.0;
						w += log(pz);
					}
				}
				double weight = exp(w);
				fprintf(fp, "%lf %lf %lf %.20lf\n", dx, dy, w, weight);
			}
			fprintf(fp, "\n");
		}
		fclose(fp);
	}
	// 正規化
	for (int i = 0; i < 6; i++)
	{
		if (i == 0) // すべてのスキャンを使って尤度計算
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "r");
		else if (i == 1) // 提案法
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "r");
		else if (i == 2) // パーティクルを用いた動的なスキャン点の棄却
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "r");
		else if (i == 3) // ビームモデル
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "r");
		else if (i == 4) // ビームモデル＋パーティクルを用いた動的なスキャン点の棄却
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "r");
		else if (i == 5) // 提案法（ベイズ推定あり）
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "r");
		double x, y, w, weight, sum = 0.0, max = -1.0;
		while (fscanf(fp, "%lf %lf %lf %lf", &x, &y, &w, &weight) != EOF)
		{
			sum += exp(w);
			if (max < exp(w))
				max = exp(w);
		}
		fclose(fp);
		FILE* fp1;
		if (i == 0)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points_norm.txt", "w");
		}
		else if (i == 1)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_norm.txt", "w");
		}
		else if (i == 2)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges_norm.txt", "w");
		}
		else if (i == 3)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_norm.txt", "w");
		}
		else if (i == 4)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test_norm.txt", "w");
		}
		else if (i == 5)
		{
			fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "r");
			fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes_norm.txt", "w");
		}
		while (fscanf(fp, "%lf %lf %lf %lf", &x, &y, &w, &weight) != EOF)
		{
//			weight = exp(w) / sum;
			weight = exp(w) / max;
			fprintf(fp1, "%lf %lf %lf %.15lf\n", x, y, w, weight);
			if (y == range)
				fprintf(fp1, "\n");
		}
		fclose(fp);
		fclose(fp1);
	}
}

void wait_for_new_map(void)
{
	bool finish_navigation = false;
	amcl->is_map_data = amcl->is_initial_pose = false;
	amcl->nh.setParam("/nav_params/reach_at_goal", false);
	amcl->nh.setParam("/nav_params/request_new_map", true);
	ros::Rate loop_rate(10.0);
	while (ros::ok())
	{
		ros::spinOnce();
		amcl->broadcast_tf();
		amcl->publish_pose();
		amcl->publish_particles();
		if (amcl->is_map_data && amcl->is_initial_pose)
		{
			amcl->nh.setParam("/nav_params/is_new_map_data", true);
			break;
		}
		amcl->nh.getParam("/nav_params/finish_navigation", finish_navigation);
		if (finish_navigation)
		{
			printf("finish navigation from amcl_dspd_node\n");
			exit(0);
		}
		loop_rate.sleep();
	}
}

void object_ids_callback(const robot_sim::ScanObjectID::ConstPtr& msg)
{
	scan_object_ids.insert(scan_object_ids.begin(), *msg);
	if (scan_object_ids.size() >= 10)
		scan_object_ids.resize(10);
}

void evaluate_dspd_accuracy(sensor_msgs::LaserScan scan)
{
	amcl->use_test_range_measurement = true;
	amcl->check_scan_points_validity(scan);
	int cc = 0, fc = 0, cc_test = 0, fc_test = 0;
	double accuracy = -1.0, accuracy_test = -1.0;
	for (int i = 0; scan_object_ids.size(); i++)
	{
		if (i >= 10)
			return;
		if (scan_object_ids[i].header.stamp == scan.header.stamp)
		{
			for (int j = 0; j < scan.ranges.size(); j++)
			{
				if (scan_object_ids[i].ids[j].data > 0 && amcl->dsp_probs[amcl->max_particle_likelihood_num][j] >= amcl->dynamic_scan_point_threshold)
					cc++;
				else if (scan_object_ids[i].ids[j].data == 0 && amcl->dsp_probs[amcl->max_particle_likelihood_num][j] < amcl->dynamic_scan_point_threshold)
					cc++;
				else if (scan_object_ids[i].ids[j].data > 0 && amcl->dsp_probs[amcl->max_particle_likelihood_num][j] < amcl->dynamic_scan_point_threshold)
					fc++;
				else if (scan_object_ids[i].ids[j].data == 0 && amcl->dsp_probs[amcl->max_particle_likelihood_num][j] >= amcl->dynamic_scan_point_threshold)
					fc++;
				if (j % amcl->scan_step == 0)
				{
					if (scan_object_ids[i].ids[j].data > 0 && amcl->is_valid_scan_points[j] == false)
						cc_test++;
					else if (scan_object_ids[i].ids[j].data == 0 && amcl->is_valid_scan_points[j] == true)
						cc_test++;
					else if (scan_object_ids[i].ids[j].data > 0 && amcl->is_valid_scan_points[j] == true)
						fc_test++;
					else if (scan_object_ids[i].ids[j].data == 0 && amcl->is_valid_scan_points[j] == false)
						fc_test++;
				}
			}
			accuracy = (double)cc / (double)(cc + fc);
			accuracy_test = (double)cc_test / (double)(cc_test + fc_test);
			printf("accuracy = %lf, accuracy_test = %lf\n", accuracy, accuracy_test);
			break;
		}
	}
	if (accuracy < 0.0 || accuracy >= 1.0)
		return;

	static FILE* fp1, *fp2, *fp3, *fp4;
	if (fp1 == NULL)
	{
		fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/static_scan_points.txt", "w");
//		fp1 = fopen("/tmp/static_scan_points.txt", "w");
	}
	if (fp2 == NULL)
	{
		fp2 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/static_scan_points_particle.txt", "w");
//		fp2 = fopen("/tmp/static_scan_points_particle.txt", "w");
	}
	if (fp3 == NULL)
	{
		fp3 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/dspd_accuracy.txt", "w");
//		fp3 = fopen("/tmp/dspd_accuracy.txt", "w");
	}
	if (fp4 == NULL)
	{
		fp4 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/estimated_trajectory.txt", "w");
//		fp4 = fopen("/tmp/estimated_trajectory.txt", "w");
	}
	double c = cos(amcl->robot_pose.yaw);
	double s = sin(amcl->robot_pose.yaw);
	double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + amcl->robot_pose.x;
	double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + amcl->robot_pose.y;
	for (int i = 0; i < scan.ranges.size(); i++)
	{
		double angle = scan.angle_min + scan.angle_increment * (double)i;
		double yaw = angle + amcl->robot_pose.yaw;
		double r = scan.ranges[i];
		double x = r * cos(yaw) + xo;
		double y = r * sin(yaw) + yo;
		if (amcl->dsp_probs[amcl->max_particle_likelihood_num][i] < amcl->dynamic_scan_point_threshold)
		{
			fprintf(fp1, "%lf %lf\n", x, y);
		}
		if (i % amcl->scan_step == 0)
		{
			if (scan.range_min <= r && r <= scan.range_max && amcl->is_valid_scan_points[i] == true)
				fprintf(fp2, "%lf %lf\n", x, y);
		}
	}
	fprintf(fp3, "%lf %lf %lf\n", scan.header.stamp.toSec(), accuracy, accuracy_test);
	fprintf(fp4, "%lf %lf %lf\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw);
	amcl->use_test_range_measurement = false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "amcl");
	// amcl
	amcl = new AMCL;
	amcl->nh.param("/amcl/use_nav_core_server", use_nav_core_server, use_nav_core_server);
	ros::Subscriber object_ids_sub = amcl->nh.subscribe("/scan_object_ids", 10, object_ids_callback);
	if (amcl->use_dspd)
		amcl->use_test_range_measurement = false;
	// start main loop
	ros::Rate loop_rate(amcl->pose_publish_hz);
	double prev_time = 0.0;
	while (ros::ok())
	{
		ros::spinOnce();
		// perform localization
		amcl->update_particle_pose_by_odom();
		if (!amcl->is_map_data || !amcl->is_scan_data || !amcl->is_tf_initialized)
		{
			ROS_ERROR("initialization is not finished yet");
			loop_rate.sleep();
			continue;
		}
		sensor_msgs::LaserScan scan = amcl->curr_scan;
		double curr_time = scan.header.stamp.toSec();;
		amcl->check_scan_points_validity(scan);
		if (amcl->use_dspd)
		{
			amcl->evaluate_particles_with_dspd(scan);
		}
		else
		{
			if (amcl->use_beam_model)
				amcl->evaluate_particles_using_beam_model(scan);
			else
				amcl->evaluate_particles_using_likelihood_field_model(scan);
		}
		amcl->compute_total_weight_and_effective_sample_size();
		amcl->estimate_robot_pose();
		amcl->compute_random_particle_rate();
		double d_time = curr_time - prev_time;
		if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
		{
//			amcl->publish_likelihood_distribution_map_vis(0.5, 0.05, scan);
			amcl->resample_particles();
			prev_time = curr_time;
			amcl->delta_dist = amcl->delta_yaw = 0.0;
			printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw * 180.0 / M_PI);
			printf("particle_num = %d\n", amcl->particle_num);
			printf("effective_sample_size = %lf, total_weight = %lf\n", amcl->effective_sample_size, amcl->total_weight);
			printf("random_particle_rate = %lf\n", amcl->random_particle_rate);
//			evaluate_dspd_accuracy(scan);
			printf("\n");
		}
		// publish localization result messages
		amcl->broadcast_tf();
		amcl->publish_pose();
		amcl->publish_particles();
		// for iros
		bool do_record = false;
		amcl->nh.param("/amcl/do_record", do_record, do_record);
		if (do_record)
		{
			for_iros(scan);
			amcl->nh.setParam("/amcl/do_record", false);
			amcl->use_test_range_measurement = false;
		}
		// for cmmunication with nav_core_server
		if (use_nav_core_server)
		{
			bool reach_at_goal;
			amcl->nh.getParam("/nav_params/reach_at_goal", reach_at_goal);
			if (reach_at_goal)
				wait_for_new_map();
		}
		loop_rate.sleep();
	}
	return 0;
}
