# Awesome rosbag Utilities

**In Process**
This is a list of various utilities repos in `github` for easily using `rosbag`. It's an attempt to gather useful material to process data collected by `rosbag`. The search keywords contains `rosbag` and `ros bag`.

## Toolbox
[rosbag-tools](https://github.com/Kautenja/rosbag-tools) Tools and scripts for working with ROS bag files.
[rosbag_toolkit](https://github.com/neufieldrobotics/rosbag_toolkit) bag2image, bag2mat, bag2pandas, bag_correction, bagmerge, bagselector, downsample_bags, image2bag, image_resize.

## Data visualisation
[PlotJuggler](https://github.com/facontidavide/PlotJuggler) A tool to visualize time series that is fast, powerful and intuitive.
[annotator](https://github.com/dsgou/annotator) Annotate video and audio streams either in common formats(mp4, avi, mkv, wav, mp3) or rosbag files.

## Data Format Convert
[rosbag_pandas](https://github.com/eurogroep/rosbag_pandas) Python library (and some tools) for converting ROS bagfiles to Pandas dataframes.
[rosbag_to_csv](https://github.com/AtsushiSakai/rosbag_to_csv) Converter from ros bag to csv.

## Data Extract
[bagpy](https://github.com/jmscslgroup/bagpy) Python package for reading, and extracting data from rosbag file and perform any analysis on it.
[rosbag.js](rosbag.js) ROS bag file reader for JavaScript.
[matlab_rosbag](https://github.com/bcharrow/matlab_rosbag) A small library for reading ROS bags in Matlab
[rosbag2video](rosbag2video) Converting image sequence in ros bag files to video files.
[bag2video](https://github.com/OSUrobotics/bag2video) Converting images in a rosbag to a video.
[ros_msg_parser](https://github.com/facontidavide/ros_msg_parser) Deserialized any ROS messages, even if you don't know its type. Successor of ros_type_introspection.
[tf_bag](https://github.com/IFL-CAMP/tf_bag) Utilities to transparently use tf data recorded with rosbag in Python scripts.
[rgbd_export]()

## Data Create/Editor
[rosbag_editor](https://github.com/facontidavide/rosbag_editor) Create/Edit a rosbag from a given one, using a simple GUI
[BagFromImages](https://github.com/search?o=desc&p=2&q=rosbag&s=stars&type=Repositories) Create a rosbag from a collection of images.
[rosbag_direct_write](https://github.com/osrf/rosbag_direct_write) C++ API for (potentially) faster writing of rosbag's using O_DIRECT with memory alignment.
[rosbag_filter_gui](https://github.com/AtsushiSakai/rosbag_filter_gui) A GUI tool for rosbag filtering.
[dataset2bag](https://github.com/lrse/dataset2bag) This package allows to convert non-ROS datasets containing images and other sensor data to a rosbag file.
[importRosbag](https://github.com/event-driven-robotics/importRosbag) Import rosbag data - standalone - no ROS installation required

## rosbag controller
[rosbag_fancy](https://github.com/xqms/rosbag_fancy) Fancy terminal UI for rosbag.
[record_ros](https://github.com/epfl-lasa/record_ros) A callback wrapper for rosbag record, where a ros service allows a user to start recording a set of topics and also stop.
[bag_recorder](https://github.com/joshs333/bag_recorder) Code for a programmatic Rosbag Recorder through ROS.

## Dataset Editor
[didi_challenge_ros](didi_challenge_ros) Roslaunch to visualize a rosbag file from Udacity (DiDi Challenge)
[udacity-driving-reader](https://github.com/rwightman/udacity-driving-reader) Scripts to read and dump data from the rosbag format used in the udacity self-driving dataset(s).

## Dateset Convert
[kitti2bag](https://github.com/tomas789/kitti2bag) Convert [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) dataset to ROS bag file the easy way!
[kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) Dataset tools for working with the [KITTI dataset raw data](http://www.cvlibs.net/datasets/kitti/raw_data.php) and converting it to a ROS bag. Also allows a library for direct access to poses, velodyne scans, and images.
[kitti_to_rosbag_for_vio](https://github.com/PetWorm/kitti_to_rosbag_for_vio) A modified version based on [ethz-asl/kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) to record image and imu data for running VIOs
[Tools_RosBag2KITTI](https://github.com/leofansq/Tools_RosBag2KITTI) Complete the conversion from the .bag file to the .png and .bin files.
[lidar2rosbag_KITTI](https://github.com/AbnerCSZ/lidar2rosbag_KITTI) A simple way to convert KITTI LiDAR data to rosbag.
[nuscenes2bag](https://github.com/clynamen/nuscenes2bag) [nuScenes](https://www.nuscenes.org/) dataset to rosbag format.

## Distributed system
[ros_hadoop](https://github.com/autovia/ros_hadoop) Hadoop splittable InputFormat for ROS. Process rosbag with Hadoop Spark and other HDFS compatible systems.
[rbb_core])(https://github.com/AMZ-Driverless/rbb_core) This repository contains the core of the Rosbag Bazaar (RBB), a tool to index/visualize/manage rosbags on remote storage systems.
[rosbag-uploader-ros1](https://github.com/search?o=desc&p=4&q=rosbag&s=stars&type=Repositories) ROS packages for uploading rosbags to AWS cloud services.

## Others
[rosbag_fixer](https://github.com/gavanderhoorn/rosbag_fixer) Quick tool to try and work around [Message Headers missing dependency information](Quick tool to try and work around Message Headers missing dependency information.).
[Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics)  [Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping.](https://arxiv.org/abs/1910.02490)
[Kimera-VIO-ROS](https://github.com/MIT-SPARK/Kimera-VIO-ROS) [Kimera: an Open-Source Library for Real-Time Metric-Semantic Localization and Mapping.](https://arxiv.org/abs/1910.02490)
[SARosPerceptionKitti](https://github.com/appinho/SARosPerceptionKitti) ROS package for the Perception (Sensor Processing, Detection, Tracking and Evaluation) of the [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) Vision Benchmark
[SASensorProcessing](SASensorProcessing) ROS node to create pointcloud out of stereo images from the [KITTI](http://www.cvlibs.net/datasets/kitti/index.php) Vision Benchmark Suite.
[Position-Control-Using-ORBSLAM2-on-the-Jetson-Nano](https://github.com/tau-adl/Position-Control-Using-ORBSLAM2-on-the-Jetson-Nano) Run ORBSLAM2 on the Jetson Nano, using recorded rosbags (e.g., EUROC) or live footage from a Bebop2 Drone