#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#if defined(_MSC_VER) && defined(_DEBUG)
#include <crtdbg.h>
#endif
#include "detector.h"
#include <pthread.h>
#include <signal.h>
#include <mutex>
#include "interface.h"

//Ros headers
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ngale_ros/start_service.h"
#include "ngale_ros/stop_service.h"

extern int gi_stop;
int r_rec, r_det, t_rec, t_det;
int r_265, t_265;
int r_enable, t_enable;
std::string r_fname;
std::string t_fname;
int detection_display;

int RGB_model_loaded, THERMAL_model_loaded;
extern pthread_cond_t rgb_full_cond ;
extern pthread_mutex_t rgb_full_mut ;
extern pthread_cond_t thr_full_cond ;
extern pthread_mutex_t thr_full_mut ;

std::mutex mutex;

pthread_t rgb_thread, thermal_thread;
bool streams_active = false;

bool stop(ngale_ros::stop_service::Request  &req,
          ngale_ros::stop_service::Response &res)
{
    std::lock_guard<std::mutex> lock(mutex);
    if (streams_active)
    {
        ROS_INFO("Got stop stream service call");

        {
            if(r_enable || r_det)
            {
                pthread_mutex_lock(&rgb_full_mut);
            }
            else if(t_enable || t_det)
            {
                pthread_mutex_lock(&thr_full_mut);
            }
        }

        {
            if(gi_stop == 1)
            {
                res.stop_status = "No Nightingale Pipeline to Stop";
            }
            else {
                gi_stop = 1;
                res.stop_status = "";
                if(r_enable || r_det)
                {
                    pthread_cond_wait(&rgb_full_cond, &rgb_full_mut);
                    res.stop_status = "Nightingale RGB Pipeline Stopped";
                    ROS_INFO("Stoping RGB Nightingale Pipeline");
                }
                else if(t_enable || t_det)
                {
                    pthread_cond_wait(&thr_full_cond, &thr_full_mut);
                    res.stop_status = res.stop_status + "Nightingale THERMAL Pipeline Stopped";
                    ROS_INFO("Stoping THEMAL Nightingale Pipeline");
                }
            }
        }

        {
            if(r_enable || r_det)
                pthread_mutex_unlock(&rgb_full_mut);
            else if(t_enable || t_det)
                pthread_mutex_unlock(&thr_full_mut);
        }

        if(r_enable||r_det)
        {
            ROS_INFO("Joining rundetector RGB thread");
            // pthread_join(rgb_thread, NULL);
            struct timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
                /* Handle error */
                ROS_ERROR("rgb_thread get clock time failure");
            }
            else
            {
                ts.tv_sec += 5;
                int s;
                s = pthread_timedjoin_np(rgb_thread, NULL, &ts);
                if (s != 0) {
                  ROS_ERROR("rgb_thread join failed");
                }
                else {
                  ROS_INFO("rgb_thread join success");
                }
            }
        }

        if(t_enable||t_det)
        {
            ROS_INFO("Joining rundetector Thermal thread");
            // pthread_join(thermal_thread, NULL);
            struct timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
                /* Handle error */
                ROS_ERROR("thermal_thread get clock time failure");
            }
            else
            {
                ts.tv_sec += 5;
                int s;
                s = pthread_timedjoin_np(thermal_thread, NULL, &ts);
                if (s != 0) {
                  ROS_ERROR("thermal_thread join failed");
                }
                else {
                  ROS_INFO("thermal_thread join success");
                }
            }
        }

        ROS_INFO("Returning Stop node");
        streams_active = false;
    }
    else
    {
        ROS_INFO("streams already stopped, do nothing");
    }
    ROS_ERROR("hack to restart ngale_exe node so thermal interface doesn't hang!");
    ros::shutdown();
    return true;
}

bool start(ngale_ros::start_service::Request  &req,
           ngale_ros::start_service::Response &res)
{
    std::lock_guard<std::mutex> lock(mutex);
    ROS_INFO("Got start stream service call");
    if (streams_active)
    {
        ROS_INFO("stream already active, do nothing");
        return true;
    }
    if(gi_stop == 1)
    {
        gi_stop = 0;

        std::string r_vision_prefix = "/drone/vision/video1";
        r_rec = ros::param::param<bool>(r_vision_prefix + "/record_only", (bool) req.rgb_cam);
        r_det = (int) ros::param::param<bool>(r_vision_prefix + "/enable_detection", (int) req.rgb_det);
        r_265 = ros::param::param<bool>(r_vision_prefix + "/h265", true);  // default h264
        r_enable = ros::param::param<bool>(r_vision_prefix + "/enable", true);

        std::string t_vision_prefix = "/drone/vision/video2";
        t_rec = ros::param::param<bool>(t_vision_prefix + "/record_only", (bool) req.thr_cam);
        t_det = (int) ros::param::param<bool>(t_vision_prefix + "/enable_detection", (int) req.thr_det);
        t_265 = ros::param::param<bool>(t_vision_prefix + "/h265", true);  // default h264
        t_enable = !ros::param::param<bool>("/drone/vision/ffmpeg_thermal", false);
        t_enable = ros::param::param<bool>(t_vision_prefix + "/enable", t_enable);  // overload ffmpeg_thermal

        r_fname =       req.rgb_filename;
        t_fname =       req.thr_filename;
        ROS_ERROR("rgb record_only[%d] enable_detection[%d] 265[%d] enable[%d] fname[%s]", r_rec, r_det, r_265, r_enable, r_fname.c_str());
        ROS_ERROR("thr record_only[%d] enable_detection[%d] 265[%d] enable[%d] fname[%s]", t_rec, t_det, t_265, t_enable, t_fname.c_str());


        res.start_status = "";
        if(r_enable || r_det)
        {
            res.start_status = " RGB Pipeline was enabled.";
            ROS_INFO("calling run detector RGB");
            pthread_create(&rgb_thread, NULL, run_detector_rgb, (void*)(r_fname.c_str()) );
        }

        if(t_enable || t_det)
        {
            res.start_status = res.start_status + " Thermal pipeline was enabled. ";
            ROS_INFO("calling run detector THERMAL");
            pthread_create(&thermal_thread, NULL, run_detector_thermal, (void*)(t_fname.c_str()));
        }

        ROS_INFO("Returning start node");
        streams_active = true;
        return true;
    }

    else
    {
        res.start_status = "Another Pipeline already running please terminate it before starting a new one.";
        ROS_INFO("Returning start node");
        return true;
    }
}

void mySigintHandler(int sig)
{

    ROS_INFO("Terminating ROS node ngale_exe");
    gi_stop = 1;
    usleep(100000);

    ros::shutdown();
}

void* start_ros(void* temp)
{
    ros::NodeHandle start_node;
    ros::ServiceServer start_service = start_node.advertiseService("start_ngale_pipeline",start);
    ros::Rate loop_rate(5);

    ROS_INFO("Ready to start nightingale pipline ");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void* stop_ros(void* temp)
{
    ros::NodeHandle stop_node;
    ros::ServiceServer stop_service = stop_node.advertiseService("stop_ngale_pipeline",stop);
    ros::Rate loop_rate(5);

    ROS_INFO("Ready to stop nightingale pipline ");
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

}


int main(int argc, char **argv)
{

    #ifdef _DEBUG
            _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
    #endif

    int gpu_index = 0;

    /*#ifndef GPU
        gpu_index = -1;
    #else
        if(gpu_index >= 0){
            cuda_set_device(gpu_index);
            CHECK_CUDA(cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync));
        }
    #endif*/
    RGB_model_loaded = 0;
    THERMAL_model_loaded = 0;
    gi_stop = 1;
    ros::init(argc, argv, "cheese_trimmer");
    ros::NodeHandle main_node;  // just to force ros timer start
    ros::start();

    pthread_t start_ros_thread, stop_ros_thread;
    signal(SIGINT, mySigintHandler);
    pthread_create(&start_ros_thread, NULL, start_ros, NULL );
    pthread_create(&stop_ros_thread, NULL, stop_ros, NULL );

    // pthread_join(start_ros_thread, NULL);
    // pthread_join(stop_ros_thread, NULL);

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
       /* Handle error */
       ROS_ERROR("start_ros_thread get clock time failure");
    }
    else
    {
       ts.tv_sec += 5;
       int s;
       s = pthread_timedjoin_np(start_ros_thread, NULL, &ts);
       if (s != 0) {
         ROS_INFO("start_ros_thread join failed");
       }
       else {
         ROS_ERROR("start_ros_thread join failed");
       }
    }
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
       /* Handle error */
       ROS_ERROR("stop_ros_thread get clock time failure");
    }
    else
    {
       ts.tv_sec += 5;
       int s;
       s = pthread_timedjoin_np(stop_ros_thread, NULL, &ts);
       if (s != 0) {
         ROS_INFO("stop_ros_thread join failed");
       }
       else {
         ROS_ERROR("stop_ros_thread join success");
       }
    }
    return 0;
}
