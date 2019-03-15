//
// Created by wang on 19-3-5.
//

//# include "stdafx.h"
# include "opencv2/imgproc/imgproc.hpp"
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>
#include "example.hpp"          // Include short list of convenience functions for rendering
// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <algorithm>            // std::min, std::max
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>   // Include OpenCV API
#include <mutex>
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

using namespace cv;

using namespace std;

int main(int argc, char * argv[])

{

// RealSense pipeline

    rs2::pipeline pipe;

    rs2::config cfg;

// depth stream config

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

// colour stream config

    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);

// start pipeline and get sensor profile data

    auto pipeProfile = pipe.start(cfg);

    auto sensor = pipeProfile.get_device().first<rs2::depth_sensor>();

    auto depthScale = sensor.get_depth_scale();

// depth and color streams

    auto align = new rs2::align(RS2_STREAM_COLOR);

    auto depth_stream = pipeProfile.get_stream(1).as<rs2::video_stream_profile>();

    auto color_stream = pipeProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    auto depthToColor = depth_stream.get_extrinsics_to(color_stream);

    auto colorToDepth = color_stream.get_extrinsics_to(depth_stream);

    auto depthIntrinsic = depth_stream.get_intrinsics();

    auto colorIntrinsic = color_stream.get_intrinsics();

// OpenCV and PCL windows

    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);

    cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

    pcl::visualization::CloudViewer viewer("Viewer");

// main loop (while PCL viewer is open)

    while (!viewer.wasStopped()) {

// get new frames

        rs2::frameset frames = pipe.wait_for_frames();

        auto processedFrames = align->process(frames);

        rs2::frame frameDepth = processedFrames.get_depth_frame();

        rs2::frame frameRGB = processedFrames.get_color_frame();

// frame size

        const int w = color_stream.width();

        const int h = color_stream.height();

// RGB and depth images in OpenCV format

        cv::Mat image(cv::Size(w, h), CV_8UC3, (void*)frameRGB.get_data(), cv::Mat::AUTO_STEP);

        cv::Mat image_depth(cv::Size(w, h), CV_16U, (uchar*)frameDepth.get_data(), cv::Mat::AUTO_STEP);

// generate output images for RGB and depth (normalize depth)

        cv::Mat image_bgr, image_map, image_nrm;

        cv::cvtColor(image, image_bgr, cv::COLOR_RGB2BGR);

//        image_depth.convertTo(image_nrm, CV_32F);
//
//        double dMin, dMax;
//
//        minMaxIdx(image_depth, &dMin, &dMax);
//
//        image_nrm /= 65535.0;
//
//        normalize(image_nrm, image_nrm, 0, 1, NORM_MINMAX);
//
//        image_nrm *= 255;
//
//        image_nrm.convertTo(image_nrm, CV_8U);
//
//        applyColorMap(image_nrm, image_map, 2);

// update image windows

        imshow("Image", image_bgr);

//        imshow("Depth", image_map);

        waitKey(10);

// new point cloud

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

        int N_bad = 0; // bad point counter (i.e., depth = 0)

// for each pixel...

        for (int u = 0; u < h; u++) {

            for (int v = 0; v < w; v++) {

// get depth from image and scale

                uint16_t depth_value = image_depth.at<uint16_t>(u, v);

                float depth_in_meters = depth_value * depthScale;

// new point

                pcl::PointXYZRGB point;

// set bad points to NAN

                if (depth_value < 0) {

                    N_bad++;

                    point.x = NAN;

                    point.y = NAN;

                    point.z = NAN;

                    point.r = 255;

                    point.g = 255;

                    point.b = 255;

                }

                else {

// use intrinsics to map image coordinates to real world coordinates

                    float depth_pixel[2];

                    depth_pixel[0] = v;

                    depth_pixel[1] = u;

                    float depth_point[3];

                    rs2_deproject_pixel_to_point(depth_point, &depthIntrinsic, depth_pixel, depth_in_meters);

                    point.x = depth_point[0];

                    point.y = depth_point[1];

                    point.z = depth_point[2];

// get corresponding RGB data and map onto point

                    auto pix = image.at<cv::Vec3b>(u, v);

                    point.r = pix[0];

                    point.g = pix[1];

                    point.b = pix[2];

                }

// add point to cloud

                cloud->points.push_back(point);

            }

        }

// output cloud to viewer

        viewer.showCloud(cloud);

    }

// terminate

    return 0;

}