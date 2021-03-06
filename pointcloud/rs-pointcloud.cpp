// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
//TdLib
#include "TdLibrary/threadSafeStructure.h"
#include "TdLibrary/realsense.h"
typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;
using pixel = std::pair<int, int>;
//Global variable
int i = 1; // Index for incremental file name
int width = 848;
int height = 480;
float scalaColor2Depth = 1.0;
using namespace std;
std::mutex lk;
// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);
std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* p_pcl_point_cloud_);
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
void Load_PCDFile(void);
int main(int argc, char * argv[]) try
{


    rs2::align align_to(RS2_STREAM_COLOR);
    // Decimation filter reduces the amount of data (while preserving best samples)
    rs2::decimation_filter dec;
    // If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
    // but you can also increase the following parameter to decimate depth more (reducing quality)
    dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
    // Define transformations from and to Disparity domain
    rs2::disparity_transform depth2disparity;
    rs2::disparity_transform disparity2depth(false);
    // Define spatial filter (edge-preserving)
    rs2::spatial_filter spat;
    // Enable hole-filling
    // Hole filling is an agressive heuristic and it gets the depth wrong many times
    // However, this demo is not built to handle holes
    // (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
    spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
    // Define temporal filter
    rs2::temporal_filter temp;

    // After initial post-processing, frames will flow into this queue:
    rs2::frame_queue postprocessed_frames;
    // In addition, intrinsics  will also flow into this queue:
//    td::threadsafe_queue<rs2_intrinsics> intri_queue;
    rs2::frame_queue cv_queue;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,30);
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR, width,height,RS2_FORMAT_BGR8,30);
    cfg.enable_stream(RS2_STREAM_INFRARED,1,width,height,RS2_FORMAT_Y8,30);
    cfg.enable_stream(RS2_STREAM_INFRARED,2,width,height,RS2_FORMAT_Y8,30);

    // Start streaming with default recommended configuration
    auto profile = pipe.start(cfg);
    auto stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    cv::Point_<u_int32_t> left_up(200,100);
    cv::Point_<u_int32_t> right_up(550,100);
    cv::Point_<u_int32_t> right_down(550,400);
    cv::Point_<u_int32_t> left_down(200,400);

//    cv::Point_<u_int32_t> left_up(0,0);
//    cv::Point_<u_int32_t> right_up(width,0);
//    cv::Point_<u_int32_t> right_down(width,height);
//    cv::Point_<u_int32_t> left_down(0,height);
    vector<cv::Point_<u_int32_t >> vertex;
    vertex.push_back(left_up);
    vertex.push_back(right_up);
    vertex.push_back(right_down);
    vertex.push_back(left_down);


    // Alive boolean will signal the worker threads to finish-up
    std::atomic_bool alive{ true };
    // Video-processing thread will fetch frames from the camera,
    // apply post-processing and send the result to the main thread for rendering
    // It recieves synchronized (but not spatially aligned) pairs
    // and outputs synchronized and aligned pairs
//    std::mutex m;
    std::thread video_processing_thread([&]() {
        // In order to generate new composite frames, we have to wrap the processing
        // code in a lambda
        rs2::processing_block frame_processor(
                [&](rs2::frameset data, // Input frameset (from the pipeline)
                    rs2::frame_source& source) // Frame pool that can allocate new frames
                {
                    // First make the frames spatially aligned
                    data = data.apply_filter(align_to);
                    // Decimation will reduce the resultion of the depth image,
                    // closing small holes and speeding-up the algorithm
//                    data = data.apply_filter(dec);//low resolution don't need Decimation

                    // To make sure far-away objects are filtered proportionally
                    // we try to switch to disparity domain
                    data = data.apply_filter(depth2disparity);

                    // Apply spatial filtering
                    data = data.apply_filter(spat);

                    // Apply temporal filtering
                    data = data.apply_filter(temp);

                    // If we are in disparity domain, switch back to depth
                    data = data.apply_filter(disparity2depth);
                    rs2::video_stream_profile depth_profile(data.get_depth_frame().get_profile());
                    rs2::video_stream_profile color_profile(data.get_color_frame().get_profile());
//                    intri_queue.push(depth_profile.get_intrinsics());
                    scalaColor2Depth = color_profile.width() / depth_profile.width();
                    // Send the composite frame for rendering
                    source.frame_ready(data);

                });
        // Indicate that we want the results of frame_processor
        // to be pushed into postprocessed_frames queue
        frame_processor >> postprocessed_frames;

        while (alive)
        {
            // Fetch frames from the pipeline and send them for processing
            rs2::frameset fs;
            if (pipe.poll_for_frames(&fs)) frame_processor.invoke(fs);
        }
    });


    std::thread img_show_thread([&]() {
        using namespace cv;
        const auto color_window = "Display Color";
        const auto depth_window = "Display Depth";
        const auto infraredLeft_window = "Display infraredLeft";
        const auto infraredRight_window = "Display infraredRight";

        namedWindow(color_window, WINDOW_AUTOSIZE);
        namedWindow(depth_window, WINDOW_AUTOSIZE);
        namedWindow(infraredLeft_window, WINDOW_AUTOSIZE);
        namedWindow(infraredRight_window, WINDOW_AUTOSIZE);

        while (waitKey(1) < 0 && getWindowProperty(color_window, WND_PROP_AUTOSIZE) >= 0 && alive)
        {
            rs2::frameset framesetForShow;
            cv_queue.poll_for_frame(&framesetForShow);
            if(framesetForShow)
            {
                rs2::frame color = framesetForShow.get_color_frame();
                rs2::frame depth = framesetForShow.get_depth_frame();
                rs2::frame infraredLeft = framesetForShow.get_infrared_frame(1);
                rs2::frame infraredRight = framesetForShow.get_infrared_frame(2);
                rs2::video_stream_profile depth_profile(depth.get_profile());
                rs2::video_stream_profile color_profile(color.get_profile());
                rs2::video_stream_profile infraredLeft_profile(infraredLeft.get_profile());
                rs2::video_stream_profile infraredRight_profile(infraredRight.get_profile());


                rs2::colorizer color_map;
                depth = depth.apply_filter(color_map);

                // Query frame size (width and height)
                const int w = width;
                const int h = height;
                // Create OpenCV matrix of size (w,h) from the color frame.
                Mat image_BGR;
                {
                    Mat image(Size(color_profile.width(), color_profile.height()), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
                    image_BGR = image.clone();
                }

                // Update the window with new data
                cv::line(image_BGR,scalaColor2Depth*left_up,scalaColor2Depth*right_up,cv::Scalar_<int>(0,0,255));
                cv::line(image_BGR,scalaColor2Depth*right_up,scalaColor2Depth*right_down,cv::Scalar_<int>(0,0,255));
                cv::line(image_BGR,scalaColor2Depth*right_down,scalaColor2Depth*left_down,cv::Scalar_<int>(0,0,255));
                cv::line(image_BGR,scalaColor2Depth*left_down,scalaColor2Depth*left_up,cv::Scalar_<int>(0,0,255));
                imshow(color_window, image_BGR);

                Mat image_depth;
                {
                    Mat image(Size(depth_profile.width(), depth_profile.height()), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
                    image_depth = image.clone();
                }
                cv::line(image_depth,left_up,right_up,cv::Scalar_<int>(0,0,255));
                cv::line(image_depth,right_up,right_down,cv::Scalar_<int>(0,0,255));
                cv::line(image_depth,right_down,left_down,cv::Scalar_<int>(0,0,255));
                cv::line(image_depth,left_down,left_up,cv::Scalar_<int>(0,0,255));
                imshow(depth_window, image_depth);

                cv::Mat dMat_left = cv::Mat(cv::Size(infraredLeft_profile.width(), infraredLeft_profile.height()), CV_8UC1, (void*)infraredLeft.get_data());
                cv::Mat dMat_right = cv::Mat(cv::Size(infraredRight_profile.width(), infraredRight_profile.height()), CV_8UC1, (void*)infraredRight.get_data());
                imshow(infraredLeft_window,dMat_left);
                imshow(infraredRight_window,dMat_right);
            }


        }
        cv::destroyAllWindows();
        alive = false;

    });

    // Allocate PCL point cloud at the resolution,organized clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // Create the PCL point cloud visualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(p_pcl_point_cloud);
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&p_pcl_point_cloud);


    while (!viewer->wasStopped() && alive) {
        static rs2::frameset current_frameset;
        postprocessed_frames.poll_for_frame(&current_frameset);
        cv_queue.enqueue(current_frameset);
//        rs2_intrinsics intr_;
//        intri_queue.wait_and_pop(intr_);
        if(current_frameset)
        {

            auto depth = current_frameset.get_depth_frame();
            auto color = current_frameset.get_color_frame();
//            rs2::video_stream_profile depth_profile(depth.get_profile());
//            intr_ = depth_profile.get_intrinsics();

            cloud_pointer cloud = td::rs::PCL_Conversion_rect(depth, color, vertex, scalaColor2Depth);



            //========================================
            // Filter PointCloud (PassThrough Method)
            //========================================
//                pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
                Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
                Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
                Cloud_Filter.setFilterLimits (0.0, 1.5);      // Set accepted interval values
                Cloud_Filter.filter (*p_pcl_point_cloud);              // Filtered Cloud Outputted

        }
        viewer->updatePointCloud(p_pcl_point_cloud);
        viewer->spinOnce(10);


    }


    // Close the viewer
    viewer->close();
    // Signal the processing thread to stop, and join
    // (Not the safest way to join a thread, please wrap your threads in some RAII manner)
    alive = false;
    video_processing_thread.join();
    img_show_thread.join();
    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
/**
 *  This function creates a PCL visualizer
 **/
std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud) {
    // Open 3D viewer and add point cloud
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("PCL Realsense 3D Viewer"));
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return (viewer);
}
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* p_pcl_point_cloud_void)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pcl_point_cloud_ = *static_cast<pcl::PointCloud<pcl::PointXYZRGB>::Ptr *> (p_pcl_point_cloud_void);
    if (event.getKeySym () == "m" && event.keyDown ())
    {
        i++; // Increment File Name
        std::string cloudFile = std::to_string(height)+"_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileASCII (cloudFile, *p_pcl_point_cloud_);
        std::cerr << "Saved " << (*p_pcl_point_cloud_).points.size () << " data points to " <<cloudFile<< std::endl;
    }
}

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
    // Get Width and Height coordinates of texture
    int width  = texture.get_width();  // Frame width in pixels
    int height = texture.get_height(); // Frame height in pixels

    // Normals to Texture Coordinates conversion
    int x_value = min(max(int(Texture_XY.u * width  + .5f), 0), width - 1);
    int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

    int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
    int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
    int Text_Index =  (bytes + strides);

    const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

    // RGB components to save in tuple
    int NT1 = New_Texture[Text_Index];
    int NT2 = New_Texture[Text_Index + 1];
    int NT3 = New_Texture[Text_Index + 2];

    return std::tuple<int, int, int>(NT1, NT2, NT3);
}
void Load_PCDFile(void)
{
    std::string openFileName;

    // Generate object to store cloud in .pcd file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView (new pcl::PointCloud<pcl::PointXYZRGB>);

    openFileName = "Captured_Frame" + std::to_string(i) + ".pcd";
    pcl::io::loadPCDFile (openFileName, *cloudView); // Load .pcd File

    //==========================
    // Pointcloud Visualization
    //==========================
    // Create viewer object titled "Captured Frame"
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Captured Frame"));

    // Set background of viewer to black
    viewer->setBackgroundColor (0, 0, 0);
    // Add generated point cloud and identify with string "Cloud"
    viewer->addPointCloud<pcl::PointXYZRGB> (cloudView, "Cloud");
    // Default size for rendered points
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
    // Viewer Properties
    viewer->initCameraParameters();  // Camera Parameters for ease of viewing

    cout << endl;
    cout << "Press [Q] in viewer to continue. " << endl;

    viewer->spin(); // Allow user to rotate point cloud and view it

    // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.


}
