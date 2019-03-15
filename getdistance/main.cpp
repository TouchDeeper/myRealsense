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
typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
using pixel = std::pair<int, int>;
//Global variable
cv::Point_<u_int32_t> left_up(540,240);
cv::Point_<u_int32_t> right_up(740,240);
cv::Point_<u_int32_t> right_down(740,420);
cv::Point_<u_int32_t> left_down(540,420);
int i = 1; // Index for incremental file name
int width = 1280;
int height = 720;
int Nbad;
using namespace std;
void getXYZfromUV(const rs2::depth_frame& frame,pixel u, float upoint[3], const rs2_intrinsics intr_,float depthScale);
std::shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
int main() {

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


    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,30);
    // For the color stream, set format to RGBA
    // To allow blending of the color frame on top of the depth frame
    cfg.enable_stream(RS2_STREAM_COLOR, width,height,RS2_FORMAT_BGR8,30);
    // Start streaming with default recommended configuration
    auto profile = pipe.start(cfg);
    auto intr = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
    auto sensor = profile.get_device().first<rs2::depth_sensor>();
    auto depthScale = sensor.get_depth_scale()*1000;//mm
//    cout<<endl;
//    cout<< typeid(intr).name()<<endl;

    // Alive boolean will signal the worker threads to finish-up
    std::atomic_bool alive{ true };

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
                    data = data.apply_filter(dec);

                    // To make sure far-away objects are filtered proportionally
                    // we try to switch to disparity domain
                    data = data.apply_filter(depth2disparity);

                    // Apply spatial filtering
                    data = data.apply_filter(spat);

                    // Apply temporal filtering
                    data = data.apply_filter(temp);

                    // If we are in disparity domain, switch back to depth
                    data = data.apply_filter(disparity2depth);

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







//    cloud_pointer cloud(new point_cloud);
//    cloud->width  = static_cast<uint32_t>( right_up.x-left_up.x  );
//    cloud->height = static_cast<uint32_t>( left_down.y-left_up.y );
//    cloud->is_dense = false;
//    cloud->points.resize( cloud->width * cloud->height );
//    // Create the PCL point cloud visualizer
//    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(cloud);
////    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&p_pcl_point_cloud);
//
//    while(!viewer->wasStopped())
//    {
//        rs2::frameset data;
////        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
//        postprocessed_frames.poll_for_frame(&data); // Wait for next set of frames from the camera
//        if(data)
//        {
//            rs2::depth_frame depth = data.get_depth_frame();
////
//            for (int j = 0; j < cloud->height; ++j) {
//                for (int k = 0; k < cloud->width; ++k) {
//
//                    pixel v;
//                    v.first = left_up.x + k;
//                    v.second = left_up.y + j;
//                    float point[3];
////                    cout<<"u="<<v.first<<",v="<<v.second<<endl;
//                    getXYZfromUV(depth,v,point, intr,depthScale);
////                    cout<<"u="<<v.first<<",v="<<v.second<<endl;
//
//                    int index_pcl = j * cloud->width + k;
//                    cloud->points[index_pcl].x = point[0];
//                    cloud->points[index_pcl].y = point[1];
//                    cloud->points[index_pcl].z = point[2];
//////                cout<<point[0]<<","<<point[1]<<","<<point[2]<<"    ";
//                }
//            }
//        }
//        viewer->updatePointCloud(cloud);
//        viewer->spinOnce(10);
//    }




    cloud_pointer cloud(new point_cloud);
    cloud->width  = static_cast<uint32_t>( width  );
    cloud->height = static_cast<uint32_t>( height );
    cloud->is_dense = false;
    cloud->points.resize( width*height );
    // Create the PCL point cloud visualizer
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer = createRGBVisualizer(cloud);
//    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&p_pcl_point_cloud);

    while(!viewer->wasStopped())
    {
        rs2::frameset data;
//        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        postprocessed_frames.poll_for_frame(&data); // Wait for next set of frames from the camera
        if(data)
        {
            rs2::depth_frame depth = data.get_depth_frame();
//
            for (int j = 0; j < height; ++j) {
                for (int k = 0; k < width; ++k) {

                    pixel v;
                    v.first = k;
                    v.second = j;
                    float point[3];
//                    cout<<"u="<<v.first<<",v="<<v.second<<endl;
                    getXYZfromUV(depth,v,point, intr,depthScale);
//                    cout<<"u="<<v.first<<",v="<<v.second<<endl;

                    int index_pcl = j * cloud->width + k;
                    cloud->points[index_pcl].x = point[0];
                    cloud->points[index_pcl].y = point[1];
                    cloud->points[index_pcl].z = point[2];
////                cout<<point[0]<<","<<point[1]<<","<<point[2]<<"    ";
                }
            }
        }
        viewer->updatePointCloud(cloud);
        viewer->spinOnce(10);
    }
    // Close the viewer
    viewer->close();
    // Signal the processing thread to stop, and join
    // (Not the safest way to join a thread, please wrap your threads in some RAII manner)
    alive = false;
    video_processing_thread.join();
    return 0;
}
void getXYZfromUV(const rs2::depth_frame& frame,pixel u, float upoint[3], const rs2_intrinsics intr_, float depthScale)
{
    float upixel[2]; // From pixel
    // Copy pixels into the arrays (to match rsutil signatures)
    upixel[0] = u.first;
    upixel[1] = u.second;
    // Query the frame for distance
    // Note: this can be optimized
    // It is not recommended to issue an API call for each pixel
    // (since the compiler can't inline these)
    // However, in this example it is not one of the bottlenecks
//    cout<<"u="<<upixel[0]<<",v="<<upixel[1]<<endl;
    auto udist = frame.get_distance(upixel[0], upixel[1]);
    float depth_in_meters = udist * depthScale;
    rs2_deproject_pixel_to_point(upoint, &intr_, upixel, depth_in_meters);
//    cout<<udist<<" ";
//    cout<<"distance = "<<udist<<endl;
//    cout<<"u="<<upixel[0]<<",v="<<upixel[1]<<endl;
    // Deproject from pixel to point in 3D
//    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data


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