// Copyright © 2018 Naoki Akai. All rights reserved.

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
#include <localizer/ndt.h>
#include <moep.hpp>

AMCL* amcl;
NDT* ndt;
bool use_nav_core_server = false;
std::vector<robot_sim::ScanObjectID> scan_object_ids;

// 実験終わったら消してOK
void for_iros(sensor_msgs::LaserScan scan)
{
    double norm_const = 1.0 / sqrt(amcl->z_hit_denom * M_PI);
    double max;
//    pose_t pose = amcl->robot_pose;
    pose_t pose;
    pose.x = pose.y = pose.yaw = 0.0;
    FILE* fp;
    double range = 0.5, resolution = 0.1;
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
    for (int i = 0; i < 8; i++)
    {
        if (i == 0) // すべてのスキャンを使って尤度計算
        {
            amcl->use_test_range_measurement = false;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "w");
            fp = fopen("/tmp/all_points.txt", "w");
        }
        else if (i == 1) // 提案法
        {
            amcl->use_test_range_measurement = false;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "w");
            fp = fopen("/tmp/use_dspd.txt", "w");
            use_bayes = false;
        }
        else if (i == 2) // パーティクルを用いた動的なスキャン点の棄却（確率ロボティクスに書いてある）
        {
            amcl->use_test_range_measurement = true;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "w");
            fp = fopen("/tmp/test_ranges.txt", "w");
        }
        else if (i == 3) // ビームモデル
        {
            amcl->use_test_range_measurement = false;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "w");
            fp = fopen("/tmp/beam_model.txt", "w");
        }
        else if (i == 4) // ビームモデル＋パーティクルを用いた動的なスキャン点の棄却
        {
            amcl->use_test_range_measurement = true;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "w");
            fp = fopen("/tmp/beam_model_with_test.txt", "w");
        }
        else if (i == 5) // 提案法（ベイズ推定あり）
        {
            amcl->use_test_range_measurement = false;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "w");
            fp = fopen("/tmp/use_dspd_bayes.txt", "w");
            use_bayes = true;
        }
        else if (i == 6) // NDT テストなし
        {
            amcl->use_test_range_measurement = false;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt.txt", "w");
            fp = fopen("/tmp/ndt.txt", "w");
        }
        else if (i == 7) // NDT テストあり
        {
            amcl->use_test_range_measurement = true;
            amcl->check_scan_points_validity(scan);
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt_with_test.txt", "w");
            fp = fopen("/tmp/ndt_with_test.txt", "w");
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
//                            dsp_prob = (1.0 - zz) * 0.5;
                            dsp_prob = z_uni * (zz_max - zz) * 0.5;
                            double sum = ssp_prob + dsp_prob;
                            ssp_prob /= sum;
                            dsp_prob /= sum;
                        }
                        else
                        {
                            ssp_prob = (amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult);
//                            dsp_prob = 1.0 - ssp_prob;
                            dsp_prob = z_uni * (zz_max - ssp_prob);
                        }
                        amcl->dsp_probs[i][j] = dsp_prob;
                        if (j % amcl->scan_step == 0)
                        {
                            double pz;
                            if (i == 0 || i == 2 || i == 3 || i == 4 || i == 6 || i == 7)
                            {
                                pz = amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                                if (i == 3 || i == 4)
                                    pz += amcl->z_short * (1.0 / (1.0 - exp(-amcl->lambda_short * scan.range_max))) * amcl->lambda_short * exp(-amcl->lambda_short * scan.range_max);
                            }
                            else
                            {
                                double pzs = zz * ssp_prob;
//                                double pzd = (1.0 - zz) * dsp_prob;
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
//                        double z = amcl->dist_map[u][v];
                        double z = amcl->dist_map.at<float>(v, u);
                        double zz = amcl->z_hit * amcl->norm_const_hit * exp(-(z * z) / amcl->z_hit_denom) + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                        if (i == 6 || i == 7)
                        {
                            z = ndt->compute_probability((float)x, (float)y);
                            if (z < 0.0)
                                z = 0.0;
                            if (z > 1.0)
                                z = 1.0;
                            zz = amcl->z_hit * z + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                        }
                        if (i == 3 || i == 4)
                        {
                            zz = (1.0 - amcl->z_max - amcl->z_rand - amcl->z_short) * amcl->norm_const_hit * exp(-(z * z) / amcl->z_hit_denom) + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                            if (r <= range_)
                                zz += amcl->z_short * (1.0 / (1.0 - exp(-amcl->lambda_short * range_))) * amcl->lambda_short * exp(-amcl->lambda_short * r);
                        }
                        if (use_bayes)
                        {
                            ssp_prob = zz * 0.5;
//                            dsp_prob = (1.0 - zz) * 0.5;
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
//                        pzd += (1.0 - zz) * dsp_prob;
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
//                            dsp_prob = (1.0 - zz) * 0.5;
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
//                        pzd += (1.0 - zz) * dsp_prob;
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
    for (int i = 0; i < 8; i++)
    {
        if (i == 0) // すべてのスキャンを使って尤度計算
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "r");
            fp = fopen("/tmp/all_points.txt", "r");
        else if (i == 1) // 提案法
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "r");
            fp = fopen("/tmp/use_dspd.txt", "r");
        else if (i == 2) // パーティクルを用いた動的なスキャン点の棄却
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "r");
            fp = fopen("/tmp/test_ranges.txt", "r");
        else if (i == 3) // ビームモデル
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "r");
            fp = fopen("/tmp/beam_model.txt", "r");
        else if (i == 4) // ビームモデル＋パーティクルを用いた動的なスキャン点の棄却
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "r");
            fp = fopen("/tmp/beam_model_with_test.txt", "r");
        else if (i == 5) // 提案法（ベイズ推定あり）
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "r");
            fp = fopen("/tmp/use_dspd_bayes.txt", "r");
        else if (i == 6) // NDT テストなし
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt.txt", "r");
            fp = fopen("/tmp/ndt.txt", "r");
        else if (i == 7) // NDT テストあり
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt_with_test.txt", "r");
            fp = fopen("/tmp/ndt_with_test.txt", "r");
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
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/all_points_norm.txt", "w");
            fp = fopen("/tmp/all_points.txt", "r");
            fp1 = fopen("/tmp/all_points_norm.txt", "w");
        }
        else if (i == 1)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_norm.txt", "w");
            fp = fopen("/tmp/use_dspd.txt", "r");
            fp1 = fopen("/tmp/use_dspd_norm.txt", "w");
        }
        else if (i == 2)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/test_ranges_norm.txt", "w");
            fp = fopen("/tmp/test_ranges.txt", "r");
            fp1 = fopen("/tmp/test_ranges_norm.txt", "w");
        }
        else if (i == 3)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_norm.txt", "w");
            fp = fopen("/tmp/beam_model.txt", "r");
            fp1 = fopen("/tmp/beam_model_norm.txt", "w");
        }
        else if (i == 4)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/beam_model_with_test_norm.txt", "w");
            fp = fopen("/tmp/beam_model_with_test.txt", "r");
            fp1 = fopen("/tmp/beam_model_with_test_norm.txt", "w");
        }
        else if (i == 5)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/use_dspd_bayes_norm.txt", "w");
            fp = fopen("/tmp/use_dspd_bayes.txt", "r");
            fp1 = fopen("/tmp/use_dspd_bayes_norm.txt", "w");
        }
        else if (i == 6)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt_norm.txt", "w");
            fp = fopen("/tmp/ndt.txt", "r");
            fp1 = fopen("/tmp/ndt_norm.txt", "w");
        }
        else if (i == 7)
        {
//            fp = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt_with_test.txt", "r");
//            fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/likelihood_distributions/case3/ndt_with_test_norm.txt", "w");
            fp = fopen("/tmp/ndt_with_test.txt", "r");
            fp1 = fopen("/tmp/ndt_with_test_norm.txt", "w");
        }
        while (fscanf(fp, "%lf %lf %lf %lf", &x, &y, &w, &weight) != EOF)
        {
//            weight = exp(w) / sum;
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
//        fp1 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/static_scan_points.txt", "w");
        fp1 = fopen("/tmp/static_scan_points.txt", "w");
    }
    if (fp2 == NULL)
    {
//        fp2 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/static_scan_points_particle.txt", "w");
        fp2 = fopen("/tmp/static_scan_points_particle.txt", "w");
    }
    if (fp3 == NULL)
    {
//        fp3 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/dspd_accuracy.txt", "w");
        fp3 = fopen("/tmp/dspd_accuracy.txt", "w");
    }
    if (fp4 == NULL)
    {
//        fp4 = fopen("/home/akai/Dropbox/papers/conference/16_iros2018/data/mapping/case2/estimated_trajectory.txt", "w");
        fp4 = fopen("/tmp/estimated_trajectory.txt", "w");
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

void for_itsc(sensor_msgs::LaserScan scan)
{
    int K = 7;
    int step = 10;
    int N = (int)scan.ranges.size();
    int MAX_ITER = 20;
    int MIN_DELTA = 1.0;
    MOEP moep(K, N);
    moep.set_max_iter(MAX_ITER);
    moep.set_min_delta(MIN_DELTA);
    moep.set_use_PMoEP(true);
    moep.set_PMoEP_param(1.0);
    moep.set_print_info(false);
    moep.set_update_indicator_variables(true);
    moep.set_shape_parameter(0, 0.5);
    moep.set_shape_parameter(1, 1.0);
    moep.set_shape_parameter(2, 1.5);
    moep.set_shape_parameter(3, 2.0);
    moep.set_shape_parameter(4, 5.0);
    moep.set_shape_parameter(5, 10.0);
    moep.set_shape_parameter(6, 20.0);
    std::vector<double> errors((int)scan.ranges.size());
    FILE* fp = fopen("/tmp/likelihood.txt", "w");
    FILE* fp1 = fopen("/tmp/moeps.txt", "w");
    pose_t pose0;
    pose0.x = pose0.y = pose0.yaw = 0.0;
    double sum = 0.0, max = -1.0;
    double moep_min = 10.0, moep_max = -10.0;
    bool write_moep_min, write_moep_max;
    std::vector<double> moep_mins, moep_maxs;
    int tnum = 0, total_iter_num = 0, max_iter_num = -1, min_iter_num = MAX_ITER + 1;
    double total_time = 0.0, max_time = -1.0, min_time = 100000000.0;
    std::vector<int> iters;
    std::vector<double> times;
    iters.clear();
    times.clear();
    for (double xx = -1.0; xx <= 1.0; xx += 0.1)
    {
        for (double yy = -1.0; yy <= 1.0; yy += 0.1)
        {
            clock_t start = clock();
            double c = cos(pose0.yaw);
            double s = sin(pose0.yaw);
            double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + pose0.x + xx;
            double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + pose0.y + yy;
            double yawo = pose0.yaw + amcl->base_link2laser.yaw;
            for (int j = 0; j < (int)scan.ranges.size(); j++)
            {
                if (!amcl->is_valid_scan_points[j])
                {
//                    errors[j] = scan.range_max;
                    errors[j] = 0.0;
                    continue;
                }
                double angle = scan.angle_min + scan.angle_increment * (double)j;
                double yaw = angle + yawo;
                double r = scan.ranges[j];
                double x = r * cos(yaw) + xo;
                double y = r * sin(yaw) + yo;
                int u, v;
                amcl->xy2uv(x, y, &u, &v);
                if (0 <= u && u < amcl->map.info.width && 0 <= v && v < amcl->map.info.height)
                    errors[j] = (double)amcl->dist_map.at<float>(v, u);
                else
//                    errors[j] = scan.range_max;
                    errors[j] = 0.0;
            }
            moep.reset_initial_parameters();
            moep.modeling_residual_errors(errors);
            double w = 0.0;
            for (int j = 0; j < (int)scan.ranges.size(); j += amcl->scan_step)
            {
//                double pz = moep.compute_likelihood(errors[j]);
                double pz = moep.compute_likelihood(errors[j]) * amcl->z_hit + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                if (pz > 1.0)
                    pz = 1.0;
                w += log(pz);
            }
            double weight = exp(w);
            clock_t end = clock();
            double etime = (double)(end - start) / CLOCKS_PER_SEC;
            int iter_num = moep.get_iter_num();
            tnum++;
            total_iter_num += iter_num;
            total_time += etime;
            if (max_iter_num < iter_num)
                max_iter_num = iter_num;
            if (min_iter_num > iter_num)
                min_iter_num = iter_num;
            if (max_time < etime)
                max_time = etime;
            if (min_time > etime)
                min_time = etime;
            iters.push_back(iter_num);
            times.push_back(etime);
            sum += weight;
            if (max < weight)
                max = weight;
            fprintf(fp, "%lf %lf %lf %lf\n", xx, yy, w, weight);
            write_moep_min = write_moep_max = false;
            if (moep_min > weight)
            {
                moep_mins.clear();
                moep_min = weight;
                write_moep_min = true;
            }
            if (moep_max < weight)
            {
                moep_maxs.clear();
                moep_max = weight;
                write_moep_max = true;
            }
            for (double e = 0.01; e <= 5.0; e += 0.1)
            {
//                double pz = moep.compute_likelihood(e);
                double pz = moep.compute_likelihood(e) * amcl->z_hit + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                fprintf(fp1, "%lf %lf\n", e, pz);
                if (write_moep_min)
                    moep_mins.push_back(pz);
                if (write_moep_max)
                    moep_maxs.push_back(pz);
            }
            fprintf(fp1, "\n");
//            num++;
//            printf("xx = %lf, yy = %lf, num = %d\n", xx, yy, num);
        }
        fprintf(fp, "\n");
    }
    double ave_iter_num = (double)total_iter_num / (double)tnum;
    double ave_time = total_time / (double)tnum;
    printf("ave_iter_num = %lf, min_iter_num = %d, max_iter_num = %d\n", ave_iter_num, min_iter_num, max_iter_num);
    printf("ave_time = %lf, min_time = %lf, max_time = %lf\n", ave_time, min_time, max_time);
//    const auto ave = std::accumulate(std::begin(iters), std::end(iters), 0.0) / std::size(iters);
//    const auto var = std::inner_product(std::begin(iters), std::end(iters), std::begin(iters), 0.0) / std::size(iters) - ave * ave;
    auto iters_ave = std::accumulate(std::begin(iters), std::end(iters), 0.0) / iters.size();
    auto iters_var = std::inner_product(std::begin(iters), std::end(iters), std::begin(iters), 0.0) / iters.size() - iters_ave * iters_ave;
    std::cout << "iters_標準偏差：" << sqrt(iters_var) << std::endl;
    auto times_ave = std::accumulate(std::begin(times), std::end(times), 0.0) / times.size();
    auto times_var = std::inner_product(std::begin(times), std::end(times), std::begin(times), 0.0) / times.size() - times_ave * times_ave;
    std::cout << "times_標準偏差：" << sqrt(times_var) << std::endl;
    fclose(fp);
    fclose(fp1);
    fp = fopen("/tmp/moep_max.txt", "w");
    fp1 = fopen("/tmp/moep_min.txt", "w");
    for (int i = 0; i < moep_mins.size(); i++)
    {
        fprintf(fp, "%lf %lf\n", (double)i * 0.1 + 0.01, moep_maxs[i]);
        fprintf(fp1, "%lf %lf\n", (double)i * 0.1 + 0.01, moep_mins[i]);
    }
    fclose(fp);
    fclose(fp1);

    fp = fopen("/tmp/likelihood.txt", "r");
    fp1 = fopen("/tmp/norm_likelihood.txt", "w");
    double xx, yy, w, weight;
    int num = 0, idx = 0;
    while (fscanf(fp, "%lf %lf %lf %lf", &xx, &yy, &w, &weight) != EOF)
    {
//        fprintf(fp1, "%lf %lf %lf %lf %d %lf\n", xx, yy, w, exp(w) / sum, iters[idx], times[idx]);
        fprintf(fp1, "%lf %lf %lf %lf %d %lf\n", xx, yy, w, exp(w) / max, iters[idx], times[idx]);
        idx++;
        num++;
        if (num == 21)
        {
            fprintf(fp1, "\n");
            num = 0;
        }
    }
    fclose(fp1);
}

void evaluate_particles_using_ndt_map(sensor_msgs::LaserScan scan)
{
    double max;
    for (int i = 0; i < amcl->particle_num; i++)
    {
        double w = 0.0;
        double c = cos(amcl->particles[i].pose.yaw);
        double s = sin(amcl->particles[i].pose.yaw);
        double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + amcl->particles[i].pose.x;
        double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + amcl->particles[i].pose.y;
        for (int j = 0; j < scan.ranges.size(); j += amcl->scan_step)
        {
            if (!amcl->is_valid_scan_points[j])
            {
                double pz = amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
                w += log(pz);
                continue;
            }
            double angle = scan.angle_min + scan.angle_increment * (double)j;
            double yaw = angle + amcl->particles[i].pose.yaw;
            double r = scan.ranges[j];
            float x = (float)(r * cos(yaw) + xo);
            float y = (float)(r * sin(yaw) + yo);
            double pz = 0.0;
            double z = ndt->compute_probability(x, y);
            if (z < 0.0)
                z = 0.0;
            if (z > 1.0)
                z = 1.0;
            pz += amcl->z_hit * z + amcl->z_max * amcl->max_dist_prob + amcl->z_rand * amcl->z_rand_mult;
            if (pz > 1.0)
                pz = 1.0;
            w += log(pz);
        }
        double weight = exp(w);
        amcl->particles[i].w *= weight;
        if (i == 0)
        {
            amcl->max_particle_likelihood_num = i;
            max = amcl->particles[i].w;
        }
        else
        {
            if (max < amcl->particles[i].w)
            {
                amcl->max_particle_likelihood_num = i;
                max = amcl->particles[i].w;
            }
        }
    }
}

void evaluate_particles_using_moep(sensor_msgs::LaserScan scan)
{
    int K = 3;
    int step = amcl->scan_step;
    int N = (int)scan.ranges.size();
    int MAX_ITER = 10;
    int MIN_DELTA = 1.0;
    MOEP moep(K, N);
    moep.set_max_iter(MAX_ITER);
    moep.set_min_delta(MIN_DELTA);
    moep.set_use_PMoEP(true);
    moep.set_PMoEP_param(1.0);
    moep.set_print_info(false);
    moep.set_update_indicator_variables(true);
    moep.set_shape_parameter(0, 0.5);
    moep.set_shape_parameter(1, 1.0);
    moep.set_shape_parameter(2, 1.5);
/*
    moep.set_shape_parameter(3, 2.0);
//    moep.set_shape_parameter(4, 10.0);
    moep.set_shape_parameter(4, 0.2);
    moep.set_shape_parameter(5, 10.0);
    moep.set_shape_parameter(6, 5.0);
 */
    std::vector<double> errors((int)scan.ranges.size());
    double max;
    for (int i = 0; i < amcl->particle_num; i++)
    {
        double c = cos(amcl->particles[i].pose.yaw);
        double s = sin(amcl->particles[i].pose.yaw);
        double xo = amcl->base_link2laser.x * c - amcl->base_link2laser.y * s + amcl->particles[i].pose.x;
        double yo = amcl->base_link2laser.x * s + amcl->base_link2laser.y * c + amcl->particles[i].pose.y;
        for (int j = 0; j < scan.ranges.size(); j += step)
        {
            if (!amcl->is_valid_scan_points[j])
            {
                errors[j] = 0.0;
                continue;
            }
            double angle = scan.angle_min + scan.angle_increment * (double)j;
            double yaw = angle + amcl->particles[i].pose.yaw;
            double r = scan.ranges[j];
            double x = r * cos(yaw) + xo;
            double y = r * sin(yaw) + yo;
            int u, v;
            amcl->xy2uv(x, y, &u, &v);
            if (0 <= u && u < amcl->map.info.width && 0 <= v && v < amcl->map.info.height)
                errors[j] = (double)amcl->dist_map.at<float>(v, u);
            else
                errors[j] = 0.0;
        }
        moep.reset_initial_parameters();
        moep.modeling_residual_errors(errors);
        double w = 0.0;
        for (int j = 0; j < (int)scan.ranges.size(); j += amcl->scan_step)
        {
            double pz = moep.compute_likelihood(errors[j]);
            if (pz > 1.0)
                pz = 1.0;
            w += log(pz);
        }
        double weight = exp(w);
        amcl->particles[i].w *= weight;
        if (i == 0)
        {
            amcl->max_particle_likelihood_num = i;
            max = amcl->particles[i].w;
        }
        else
        {
            if (max < amcl->particles[i].w)
            {
                amcl->max_particle_likelihood_num = i;
                max = amcl->particles[i].w;
            }
        }
    }
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
    // ndt
    ndt = new NDT;
//	if (!ndt->read_ndt_map("/home/akai/Dropbox/work/AutoNavi/ros/maps/nic_garage/ndt_map.txt"))
//	if (!ndt->read_ndt_map("/home/akai/maps/tctsls/ndt_map.txt"))
	if (!ndt->read_ndt_map("/home/akai/maps/tctsls/ndt_map_200cm.txt"))
	{
	    ROS_ERROR("NDT map could not be read.");
        return 1;
	}
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
        /*
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
        */
        amcl->evaluate_particles_using_likelihood_field_model(scan);
//        amcl->evaluate_particles_with_dspd(scan);
//        evaluate_particles_using_ndt_map(scan);
        double d_time = curr_time - prev_time;
        if (fabs(amcl->delta_dist) >= amcl->update_dist || fabs(amcl->delta_yaw) >= amcl->update_yaw || d_time >= amcl->update_time)
        {
//            evaluate_particles_using_moep(scan);
            amcl->compute_total_weight_and_effective_sample_size();
            amcl->estimate_robot_pose();
            amcl->compute_random_particle_rate();
//            amcl->publish_likelihood_distribution_map_vis(0.5, 0.05, scan);
//            amcl->resample_particles();
            prev_time = curr_time;
            amcl->delta_dist = amcl->delta_yaw = 0.0;
            printf("x = %.3lf [m], y = %.3lf [m], yaw = %.3lf [deg]\n", amcl->robot_pose.x, amcl->robot_pose.y, amcl->robot_pose.yaw * 180.0 / M_PI);
            printf("particle_num = %d\n", amcl->particle_num);
            printf("effective_sample_size = %lf, total_weight = %lf\n", amcl->effective_sample_size, amcl->total_weight);
            printf("random_particle_rate = %lf\n", amcl->random_particle_rate);
//            evaluate_dspd_accuracy(scan); // for iros
            printf("\n");
        }
        // publish localization result messages
        amcl->broadcast_tf();
        amcl->publish_pose();
        amcl->publish_particles();
        // for iros and itsc
        bool do_record = false;
        amcl->nh.param("/amcl/do_record", do_record, do_record);
        if (do_record)
        {
            for_iros(scan);
//            for_itsc(scan);
            amcl->nh.setParam("/amcl/do_record", false);
            amcl->use_test_range_measurement = false;
            printf("finish to record\n\n");
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
