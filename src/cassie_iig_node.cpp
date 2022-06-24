#include <string>
#include <iostream>
#include <ros/ros.h>
#include "bgkoctomap.h"
#include "markerarray_pub.h"
#include "cassie_iig_util_yaml.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cassie_node");
    ros::NodeHandle nh("~");

    // initialize and set default params
    int num_class = 2;
    std::string static_frame("/odom");
    std::string dir;
    std::string prefix;
    int scan_num = 0;
    std::string map_topic("/semantic_map");
    std::string traversability_map_topic("/traversability_map");
    double max_range = -1;
    double resolution = 0.1;
    int block_depth = 4;
    double sf2 = 1.0;
    double ell = 1.0;
    double free_resolution = 0.5;
    double ds_resolution = 0.1;
    double free_thresh = 0.3;
    double occupied_thresh = 0.7;
    double min_z = 0;
    double max_z = 0;
    bool original_size = false;
    float var_thresh = 1.0f;
    float prior_A = 1.0f;
    float prior_B = 1.0f;
    float prior = 1.0f;

    //sjy edit: add params required by planner
    double search_distance = 2.0;
    double cos_view = cos(Pi / 3);
    double pitch = 0 * Pi /3;
    double sm = 2.0;
    double prob_stat = 0.75;
    double delta_ric = 0.01;
    double budget = 5.0;

    double bb_blocks_size = 5.0;
    double view_angle = 1.04719753;

    // read params from ros params saved in YAML file
    nh.param<int>("num_class", num_class, num_class);
    nh.param<std::string>("static_frame", static_frame, static_frame);
    nh.param<std::string>("dir", dir, dir);
    nh.param<std::string>("prefix", prefix, prefix);
    nh.param<std::string>("topic", map_topic, map_topic);
    nh.param<std::string>("traversability_map_topic", traversability_map_topic, traversability_map_topic);
    nh.param<int>("scan_num", scan_num, scan_num);
    nh.param<double>("max_range", max_range, max_range);
    nh.param<double>("resolution", resolution, resolution);
    nh.param<int>("block_depth", block_depth, block_depth);
    nh.param<double>("sf2", sf2, sf2);
    nh.param<double>("ell", ell, ell);
    nh.param<double>("free_resolution", free_resolution, free_resolution);
    nh.param<double>("ds_resolution", ds_resolution, ds_resolution);
    nh.param<double>("free_thresh", free_thresh, free_thresh);
    nh.param<double>("occupied_thresh", occupied_thresh, occupied_thresh);
    nh.param<double>("min_z", min_z, min_z);
    nh.param<double>("max_z", max_z, max_z);
    nh.param<bool>("original_size", original_size, original_size);
    nh.param<float>("var_thresh", var_thresh, var_thresh);
    nh.param<float>("prior_A", prior_A, prior_A);
    nh.param<float>("prior_B", prior_B, prior_B);
    nh.param<float>("prior", prior, prior);

    // sjy edit
    nh.param<double>("search_distance", search_distance, search_distance);
    nh.param<double>("cos_view", cos_view, cos_view);
    nh.param<double>("pitch", pitch, pitch);
    nh.param<double>("sm", sm, sm);
    nh.param<double>("prob_stat", prob_stat, prob_stat);
    nh.param<double>("delta_ric", delta_ric, delta_ric);
    nh.param<double>("budget", budget, budget);

    nh.param<double>("bb_blocks_size", bb_blocks_size, bb_blocks_size);
    nh.param<double>("view_angle", view_angle, view_angle);

    ROS_INFO_STREAM("Parameters:" << std::endl <<
            "num_class: " << num_class << std::endl <<
            "static_frame: " << static_frame << std::endl <<
            "dir: " << dir << std::endl <<
            "prefix: " << prefix << std::endl <<
            "topic: " << map_topic << std::endl <<
            "scan_sum: " << scan_num << std::endl <<
            "max_range: " << max_range << std::endl <<
            "resolution: " << resolution << std::endl <<
            "block_depth: " << block_depth << std::endl <<
            "sf2: " << sf2 << std::endl <<
            "ell: " << ell << std::endl <<
            "free_resolution: " << free_resolution << std::endl <<
            "ds_resolution: " << ds_resolution << std::endl <<
            "free_thresh: " << free_thresh << std::endl <<
            "occupied_thresh: " << occupied_thresh << std::endl <<
            "min_z: " << min_z << std::endl <<
            "max_z: " << max_z << std::endl <<
            "original_size: " << original_size << std::endl <<
            "var_thresh: " << var_thresh << std::endl <<
            "prior_A: " << prior_A << std::endl <<
            "prior_B: " << prior_B << std::endl <<
            "prior: " << prior
            );

    CassieData cassie_data(nh, static_frame, map_topic, traversability_map_topic,
        resolution, block_depth, num_class,
        sf2, ell, free_thresh, occupied_thresh, var_thresh,
        prior_A, prior_B, prior,
        ds_resolution, free_resolution, max_range,
        // add planner params
        search_distance, cos_view, pitch, sm, prob_stat, delta_ric, budget,
        bb_blocks_size, view_angle);

    ros::Subscriber ssub = nh.subscribe("/labeled_pointcloud", 1, &CassieData::SemanticPointCloudCallback, &cassie_data);
    ros::Subscriber tsub = nh.subscribe("/labeled_pointcloud_traversability", 1, &CassieData::TraversabilityPointCloudCallback, &cassie_data);
    ros::Subscriber psub = nh.subscribe("/cassie/pose", 1, &CassieData::RobotPoseCallBack, &cassie_data);

    ros::spin();

    return 0;
}
