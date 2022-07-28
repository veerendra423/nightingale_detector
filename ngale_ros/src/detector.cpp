#include "interface.h"
#include <opencv2/opencv.hpp>
#include "TrtNet.h"
#include <chrono>
#include "YoloLayer.h"
//#include "YoloConfigs.h"	//<sagarwal> importing will cause recursive reference generating error
#include "utils.h"
#include <stdlib.h>

#ifndef __COMPAR_FN_T
#define __COMPAR_FN_T
typedef int (*__compar_fn_t)(const void*, const void*);
#ifdef __USE_GNU
typedef __compar_fn_t comparison_fn_t;
#endif
#endif

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/opencv_modules.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/gstbuffer.h>
#include <gst/base/gstbytewriter.h>
#include <unistd.h>
#include <math.h>
#include <mutex>
#include <linux/version.h>
#include <time.h>
//ros headers
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sstream>
#include <birdseye_msgs/BirdStatus.h>
#include <birdseye_msgs/Object.h>
#include <birdseye_msgs/DetectionResult.h>
#include <angles/angles.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#define ERR_CHECK(expression, ret)  if (expression) return(ret);

#include <thread>   //<sagarwal>
// exif
extern "C" {
#include <libexif/exif-data.h>
#include <libjpeg/jpeg-data.h>
#include <JpegEncoderEXIF/JpegEncoderEXIF.h>
#include "misbst0601.h"
};
/* byte order to use in the EXIF block */
#define FILE_BYTE_ORDER EXIF_BYTE_ORDER_INTEL
/* comment to write into the EXIF block */
#define FILE_COMMENT "libexif test message"
/* special header required for EXIF_TAG_USER_COMMENT */
#define ASCII_COMMENT "ASCII\0\0\0"



// namespaces
using namespace std;
using namespace Tn;
using namespace Yolo;


// drone telemetry receiver
class DroneState {

public:

    // Properties from the YAML
    int image_width;
    int image_height;
    // float hfov;
    // float vfov;
    float thresh;
    float hier_thresh;
    float yaw_offset;
    float tilt_offset;

    float vehicle_lat;
    float vehicle_lon;
    float vehicle_alt_asl;
    float vehicle_alt_rel;
    float vehicle_hdg;
    float vehicle_hdg_last;
    float vehicle_hdg_interpolated;
    float vehicle_tlt;
    float gimbal_rol;
    float gimbal_pit;
    float gimbal_yaw;
    float gimbal_yaw_last;
    float gimbal_yaw_interpolated;
    float gimbal_yaw_offset;

    // estimate gimbal true heading using:
    //     1. gimbal_yaw readings (may drift overtime) and
    //     2. vehicle_hdg (absolute but may offset from gimbal yaw)
    // assume that gimbal true heading is vehicle heading when vehicle is yawing slowly
    // when vehicle is yawing fast, differeence between gimbal_yaw and vehicle_hdg may change
    // use saved offset to estimate how much change happened
    float gimbal_true_hdg;

    float attitude_yawspeed;
    float last_error_est;

    ros::Time status_stamp;
    ros::Time status_stamp_last;
    ros::Time cb_time;
    bool bird_status_available;
    bool tracking_updated;

    // mission related
    std::string cmd_id;
    std::string launch_id;
    std::string mission_id;

    // Constructor
    DroneState() {
        cmd_id = "000000000000000000000000";
        launch_id = "000000000000000000000000";
        mission_id = "000000000000000000000000";
        bird_status_available = false;
        last_error_est = 0;
        tracking_updated = false;
    }

    void UpdateGimbalTrueHdg(double detection_timestamp_sec) {
        // limit timestamp so we interpolate and not extrapolate
        double t0 = status_stamp_last.toSec();
        double t1 = status_stamp.toSec();
        double ts = detection_timestamp_sec;
        if (detection_timestamp_sec < t0) detection_timestamp_sec = t0;
        if (detection_timestamp_sec > t1) detection_timestamp_sec = t1;
        // interpolate vehicle heading using time
        double nn = (detection_timestamp_sec - status_stamp_last.toSec());
        double dd = (status_stamp.toSec()    - status_stamp_last.toSec());
        if (dd < 1.0e-5) dd = 1.0;  // prevent near zero denominator and exploding interpolation
        double ss = nn/dd;
        vehicle_hdg_interpolated = vehicle_hdg_last +
             ss * angles::shortest_angular_distance(vehicle_hdg_last*M_PI/180.0, vehicle_hdg*M_PI/180.0)*180.0/M_PI;
        // interpolate yaw using time
        gimbal_yaw_interpolated = gimbal_yaw_last +
             ss * angles::shortest_angular_distance(gimbal_yaw_last*M_PI/180.0, gimbal_yaw*M_PI/180.0)*180.0/M_PI;

        float yaw_tol = ros::param::param<float>("filter/yaw_tol", 0.1);

        // use gimbal_yaw_interpolated and vehicle_hdg_interpolated to estimate gimbal_true_hdg
        // caching an offset between gimbal yaw and vehicle heading. If drone is moving fast,
        // use cached offset to estimate vehicle heading
        if (fabs(attitude_yawspeed) < yaw_tol) {
            gimbal_yaw_offset = angles::shortest_angular_distance(gimbal_yaw_interpolated*M_PI/180.0, vehicle_hdg_interpolated*M_PI/180.0)*180.0/M_PI;
            // assume that gimbal true heading is vehicle heading when vehicle is moving slow
            gimbal_true_hdg = angles::normalize_angle_positive(vehicle_hdg_interpolated*M_PI/180.0)*180.0/M_PI;
        } else {
            // yawing fast, camera angle and drone heading may be different
            // when vehicle is yawing fast, differeence between gimbal_yaw and vehicle_hdg may change
            // use saved offset to estimate how much change happened
            // double gimbal_yaw_offset_fast = vehicle_hdg_interpolated - gimbal_yaw_interpolated;
            // double gimbal_yaw_offset_fast_diff = gimbal_yaw_offset_fast - gimbal_yaw_offset;
            gimbal_true_hdg = angles::normalize_angle_positive(gimbal_yaw_interpolated*M_PI/180.0 + gimbal_yaw_offset*M_PI/180.0)*180.0/M_PI;
        }
        ROS_INFO("INTERPOLATION DEBUG: t0 %f t1 %f t %f nn %f dd %f ss %f hdg %f yspd %f veh_hdg %f gmb_yaw %f last veh_hdg %f gmb_yaw %f lat/lon(%f, %f)",
            t0, t1-t0, ts-t0,
            nn, dd, ss, gimbal_true_hdg, attitude_yawspeed, vehicle_hdg, gimbal_yaw, vehicle_hdg_last, gimbal_yaw_last,
            vehicle_lat, vehicle_lon);
    }

};

DroneState drone_state_;
std::vector<DroneState> drone_state_queue_;

/* consider putting tracking into here
class TrackedObjects {
    public:

        ros::Time initial_detection;
        ros::Time most_recent_detection;
        int track_id;
        float bbox_x;
        float bbox_y;
        float bbox_x;
        float bbox_y;

    TrackedObjects() {
    }
};

TrackedObjects tracked_objects_;
*/

std::mutex bird_status_mutex_;
void bird_status_callback_fast(const std_msgs::Float64MultiArray::ConstPtr _msg) {

    // Set the variables
    std::lock_guard<std::mutex> lock(bird_status_mutex_);
    ros::Time now_is = ros::Time::now();  // time we received this status

    /* delay drone state test
    double detection_delay_sec = ros::param::param<double>("detection_delay", 0.35);
    // status_stamp contains the detected object's timestamp at time of detection.
    // we should use drone status that's older thatn detected object state plus some delay due to time it takes to detect
    double too_early_by = _msg->data[0] - (drone_state_.status_stamp.toSec() - detection_delay_sec);
    if (too_early_by > 0)
    {
        // ROS_INFO("fast[%f] det[%f] delay[%f] early[%f]",
        //     _msg->data[0], drone_state_.status_stamp.toSec(), detection_delay_sec, too_early_by);
        if (too_early_by > 2) too_early_by = 2;  // truncate to 2 seconds
        // skip if drone status is too new
        usleep(too_early_by * 1000000);
    }
    else
    {
        // ROS_INFO("fast[%f] det[%f] delay[%f] late[%f]",
        //     _msg->data[0], drone_state_.status_stamp.toSec(), detection_delay_sec, too_early_by);
    }
    */
    drone_state_.cb_time = now_is;  // save time we received this status, not used
    drone_state_.status_stamp_last = drone_state_.status_stamp;
    drone_state_.status_stamp = ros::Time(_msg->data[0]);

    ROS_INFO("BSF: age of bird_status [%f] time since last update [%f] yawspeed [%f]",
             (now_is - drone_state_.status_stamp).toSec(),
             (drone_state_.status_stamp - drone_state_.status_stamp_last).toSec(),
             drone_state_.attitude_yawspeed
            );

    // drone_state_.hfov = ros::param::param<float>("/drone/detection/rgb/hfov", 2.0*0.8255407362);
    // drone_state_.vfov = ros::param::param<float>("/drone/detection/rgb/vfov", 2.0*0.547430511);

    // not used
    drone_state_.image_width = ros::param::param<int>("/drone/detection/rgb/image_width", 416);
    drone_state_.image_height = ros::param::param<int>("/drone/detection/rgb/image_height", 416);
    drone_state_.yaw_offset = ros::param::param<float>("/drone/detection/rgb/yaw_offset", 0);
    drone_state_.tilt_offset = ros::param::param<float>("/drone/detection/rgb/tilt_offset", 0);
    drone_state_.thresh = ros::param::param<float>("/drone/detection/rgb/thresh", 0.5);
    drone_state_.hier_thresh = ros::param::param<float>("/drone/detection/rgb/hier_thresh", 0.5);

    drone_state_.vehicle_lat = _msg->data[1];
    drone_state_.vehicle_lon = _msg->data[2];
    drone_state_.vehicle_alt_asl = _msg->data[3];
    drone_state_.vehicle_alt_rel = _msg->data[4];
    drone_state_.vehicle_hdg_last = drone_state_.vehicle_hdg;
    drone_state_.vehicle_hdg = _msg->data[5];

    drone_state_.vehicle_tlt = _msg->data[6];
    drone_state_.attitude_yawspeed = _msg->data[7];
    drone_state_.gimbal_rol = _msg->data[8];
    drone_state_.gimbal_pit = _msg->data[9];
    drone_state_.gimbal_yaw_last = drone_state_.gimbal_yaw;
    drone_state_.gimbal_yaw = _msg->data[10];

    drone_state_.bird_status_available = true;
    ROS_INFO("BIRD_STATUS: vehicle status: heading filter: yawspeed[%f] corrected [%f] gimbal [%f] offset [%f]",
        drone_state_.attitude_yawspeed,
        drone_state_.vehicle_hdg, drone_state_.gimbal_yaw, drone_state_.gimbal_yaw_offset);
    ROS_INFO("BIRD_STATUS: vehicle status: gps(%f, %f) msl(%f) agl(%f) hdg(%f) tlt(%f) rpy(%f, %f, %f)",
        drone_state_.vehicle_lat, drone_state_.vehicle_lon,
        drone_state_.vehicle_alt_asl, drone_state_.vehicle_alt_rel,
        drone_state_.vehicle_hdg, drone_state_.vehicle_tlt,
        drone_state_.gimbal_rol, drone_state_.gimbal_pit, drone_state_.gimbal_yaw);
      //set lattitude
      setLatitude(drone_state_.vehicle_lat);
      //set longitude
      setLongitude(drone_state_.vehicle_lon);
      //set altitude
      setAltitude(drone_state_.vehicle_alt_asl);
      //set Drol
      setRol(drone_state_.gimbal_rol);
      //set Dpitch
      setPitch(drone_state_.gimbal_pit);
      //set Dyaw
      setYaw(drone_state_.gimbal_yaw);
      //set hdg
      setHdg(drone_state_.vehicle_hdg);

    drone_state_queue_.push_back(drone_state_);
}



void bird_status_callback_dcs(const birdseye_msgs::BirdStatus::ConstPtr _msg) {

    // Set the variables
    std::lock_guard<std::mutex> lock(bird_status_mutex_);

    // TODO: actually state keep cmd_ids as a list, extract first one?
    // try { drone_state_.cmd_id = _msg->cmd_id; }
    // catch (...) { ROS_ERROR("BIRD_STATUS_DCS: exception getting cmd_id"); }

    try { drone_state_.launch_id = _msg->current_active_command.launch_id; }
    catch (...) { ROS_ERROR("BIRD_STATUS_DCS: exception getting launch_id"); }

    try { drone_state_.mission_id = _msg->current_active_command.mission_id; }
    catch (...) { ROS_ERROR("BIRD_STATUS_DCS: exception getting mission_id"); }

    ROS_INFO("BIRD_STATUS_DCS: cmd_id [%s] launch_id [%s] mission_id [%s]",
        drone_state_.cmd_id.c_str(), drone_state_.launch_id.c_str(), drone_state_.mission_id.c_str());
}



//Gstreamer tensorRT integration
typedef struct stram_obj_t_
{
    stream_config_t s_stream_config;
    GstElement *s_pipeline;
    volatile int i_stopped;
} stream_obj_t;

//gloabl variables
extern int r_rec, r_det, t_rec, t_det;
extern int r_265, t_265;
extern int r_enable, t_enable;
extern int detection_display;

enum gi_states {
    RUNNING = 0,
    STOPPED = 1
};
enum pipe_states {
    CAP_IDLE = 20,
    CAP_INIT = 10,
    CAP_READY = 0,
    DET_READY = 1,
    PUB_READY = 5,
    VIS_READY = 8
};

int check_mistakes = 0;
int gi_stop = 0;
int gi_log_level = 0;
extern int RGB_model_loaded, THERMAL_model_loaded;
int rgb_enc_width, rgb_enc_height;
int thr_enc_width, thr_enc_height;
int rgb_det_width, rgb_det_height;
int thr_det_width, thr_det_height;
int rgb_snapshot_width, rgb_snapshot_height;

int klv_frame_counter = 0;

// Multithreading Value holders
pthread_t det_thread_rgb;
//printf("%li\n", (unsigned long int) det_thread_rgb);
pthread_t det_thread_thr;

// pthread_cond_t rgb_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t rframe_rgb_mut = PTHREAD_MUTEX_INITIALIZER;

pthread_t snapshot_thread_rgb;
pthread_cond_t snapshot_rgb_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t snapshot_rgb_mut = PTHREAD_MUTEX_INITIALIZER;
ros::Time last_snapshot(0);

// pthread_cond_t thr_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t rframe_thr_mut = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t rgb_full_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t rgb_full_mut = PTHREAD_MUTEX_INITIALIZER;

pthread_cond_t thr_full_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t thr_full_mut = PTHREAD_MUTEX_INITIALIZER;

GstBusSyncReply cb_message(GstBus *ps_bus, GstMessage *ps_message, gpointer user_data)
{

    stream_obj_t *ps_stream = (stream_obj_t *)user_data;
    GError *ps_err = NULL;
    gchar *debug   = NULL;

    /* Check what the message type is */
    switch(GST_MESSAGE_TYPE (ps_message))
    {
    case GST_MESSAGE_ERROR:
        gst_message_parse_error (ps_message, &ps_err, &debug);
        ROS_ERROR("%s", ps_err->message);
        ps_stream->i_stopped = 1;
        break;
    case GST_MESSAGE_WARNING:
        gst_message_parse_warning (ps_message, &ps_err, &debug);
        ROS_WARN("%s", ps_err->message);
        break;
    case GST_MESSAGE_EOS:
        ROS_INFO("Reached EOS");
        ps_stream->i_stopped = 1;
        break;
    default:
        ROS_DEBUG("Unknown message");
    }

    if (ps_err)
        g_error_free (ps_err);
    if (debug)
        g_free (debug);
    /* remove message from the queue */
    return GST_BUS_PASS;
}

GstElement *element_create(GstElement *ps_pipeline, const char * pc_name)
{
    GstElement *ps_element = gst_element_factory_make(pc_name, NULL);
    if (ps_element == NULL)
    {
        ROS_ERROR("Error in creating element %s", pc_name);
        return(NULL);
    }
    gst_bin_add_many(GST_BIN(ps_pipeline), ps_element, NULL);
    return(ps_element);
}

GstFlowReturn new_sample_rgb(GstAppSink *elt, gpointer ptr);
GstFlowReturn new_sample_thermal(GstAppSink *elt, gpointer ptr);

GstFlowReturn new_snapshot_rgb(GstAppSink *elt, gpointer ptr);


GstSample *sample_gst_rgb;
GstBuffer *buffer_gst_rgb;
GstMapInfo map_rgb;

GstSample *snapshot_gst_rgb;
GstBuffer *buffer_snapshot_rgb;
GstMapInfo snapshot_map_rgb;

GstSample *sample_gst_thr;
GstBuffer *buffer_gst_thr;
GstMapInfo map_thr;
static GMainLoop *loop;
GstElement *rec_metasrc;
// ros rate control for detection
ros::Time t_det_last_update;
ros::Duration t_det_duration(0.0);
ros::Time r_det_last_update;
ros::Duration r_det_duration(0.0);

bool rgb_start_success = false;
bool thr_start_success = false;
int snapshot_cnt_rgb = CAP_IDLE;
int cnt_rgb = CAP_IDLE;
int cnt_thr = CAP_IDLE;
cv::Mat rframe_rgb;
cv::Mat rframe_thr;

cv::Mat snapshot_rframe_rgb;

float limit_x;
float limit_y;
int im_w;
int im_h;

bool compareByTopLeft(const Bbox &a, const Bbox &b)
{

    const float delta_y = a.top - b.top;
    const float delta_x = a.left - b.left;

    if (fabs(delta_y) < limit_y)
    {
        return a.left < b.left;
    }
    else
    {
        if (fabs(delta_x) < limit_x) {
            return a.top < b.top;
        }
        else
        {
            return a.left < b.left;
        }
    }
}

bool has_any_digits(const std::string& s)
{
    return std::any_of(s.begin(), s.end(), ::isdigit);
}

class nightingale_detector {

public :

    int modelid;
    int frame_id;
    std::unique_ptr<trtNet> net;
    birdseye_msgs::DetectionResult detection_raw_viz;
    birdseye_msgs::DetectionResult detection_raw_syn;
    std::mutex detection_raw_viz_mutex;
    std::mutex detection_raw_syn_mutex;
    std::vector<birdseye_msgs::DetectionResult> detection_raw_queue;

    std::string fname;
    bool railcar;

    /* Text Overlay */
    GstElement *r_text_overlay;

    std::vector<GstElement *> s_text_overlay_p;
    std::vector<GstElement *> s_text_overlay_c;
    GstElement *s_text_overlay2;

    /* text overlay */
    bool stream_test_textoverlay;
    bool write_test_textoverlay;

    GstElement *g_st_enc;

    /* detection */
    float hfov, vfov;
    float tmargin;
    //float nmsThresh = 0.45;
    DroneState local_drone_state;
    //printf("***********************************************************************IGNORE_THRESH %f",Yolo::IGNORE_THRESH);
   // sprintf("Ignore Thresh %f", IGNORE_THRESH);
    void detect_frame(cv::Mat img, std::unique_ptr<trtNet> &net) {
        //std::cout << "Ignore Thresh" << IGNORE_THRESH << std::endl;
        list<vector<Bbox>> outputs;
        int classNum, num_chans, det_h, det_w;
        int batchSize = 1;
        int batchCount = 1;

        // Original image size
        int ori_w, ori_h;

        if (modelid == 0)
        {
            classNum = 2;
            num_chans = 3;
            det_h = rgb_det_height;
            det_w = rgb_det_width;

            ori_w = rgb_enc_width;
            ori_h = rgb_enc_height;

            if (railcar)
                classNum = 36;
        }
        else
        {
            classNum = 2;
            num_chans = 3;
            det_h = thr_det_height;
            det_w = thr_det_width;

            ori_w = thr_enc_width;
            ori_h = thr_enc_height;
        }

        int outputCount = net->getOutputSize()/sizeof(float);
        unique_ptr<float[]> outputData(new float[outputCount]);

        vector<float> inputData;
        inputData.reserve(det_h*det_w*num_chans*batchSize);
        vector<cv::Mat> inputImgs;

        vector<float> curInput = prepareImage(img,num_chans,det_h,det_w);
        inputImgs.emplace_back(img);

        inputData.insert(inputData.end(), curInput.begin(), curInput.end());


	    double ignoreThresh;				//<sagarwal>specifying the value of ignore_thresh
        ros::param::param("/ignoreThresh",ignoreThresh,0.45);	//<sagarwal> reading threshold from configuration
        //std::cout <<"Ignore Thresh" << ignoreThresh << std::endl;
        
        net->doInference(inputData.data(), outputData.get(),ignoreThresh, batchCount);		//<sagarwal> added parameter ignore_thresh
        
        //Get Output
        auto output = outputData.get();
        auto outputSize = net->getOutputSize()/ sizeof(float) / batchCount;

        //first detect count
        int detCount = output[0];
        //later detect result
        vector<Detection> result;
        result.resize(detCount);
        memcpy(result.data(), &output[1], detCount*sizeof(Detection));

        auto boxes = postProcessImg(inputImgs[0],result,classNum, det_h, det_w);
        //float msThresh = 0.45;
       // auto boxes = postProcessImg(inputImgs[0],result,classNum, det_h, det_w,nmsThresh);
        inputImgs.clear();
        inputData.clear();

        birdseye_msgs::DetectionResult detection_raw_local;
        detection_raw_local.width = det_w;  // deprecate this
        detection_raw_local.height = det_h;  // deprecate this
        detection_raw_local.objects.clear();

        if (!railcar) {
            // send alone image pixel size with message
            ros::Time current_time;

            outputs.emplace_back(boxes);
            output += outputSize;
            auto bbox = *outputs.begin();
            {
                // ROS_ERROR("RGB5: fill detection_raw_local");

                {
                    // copy DroneState drone_state_
                    // std::lock_guard<std::mutex> lock(bird_status_mutex_);
                    // local_drone_state = drone_state_;

                    current_time = ros::Time::now();  // do this here in case drone_state_ mutex lock takes long

                    ROS_INFO("DETECT FRAME: now is %f outputs %d", current_time.toSec(), (int)outputs.size());

                    //
                    // TODO: queue detction data (detection_raw_local) and
                    // synchronize in time with drone_state_ data
                    //
                    // ROS_INFO("INTERPOLATION TIMING DEBUG: now is %f", current_time.toSec());
                    // local_drone_state.UpdateGimbalTrueHdg(current_time.toSec());
                }

                for(const auto& item : bbox)
                {

                    // hardcoded names based on class_id
                    std::string objname;
                    if (item.classId == 0)
                        objname = "pedestrian";
                    else
                        objname = "vehicle";

                    // sstring<<"\t[class: " << objname
                    //        <<" left_x: "<< item.left
                    //        <<"  top_y: " << item.top
                    //        <<"  right: " << item.right
                    //        <<"  bot: "<< item.bot
                    //        <<" ],\n";
                    // msg.data = msg.data + sstring.str();
                    // sstring.str("");

                    // object message
                    birdseye_msgs::Object object;
                    object.header.stamp = current_time;

                    // image size
                    object.width = (float)det_w;
                    object.height = (float)det_h;

                    // bounding box center and size
                    object.bbox_x = 0.5*(item.left + item.right);
                    object.bbox_y = 0.5*(item.top + item.bot);
                    object.bbox_w = item.right - item.left;
                    object.bbox_h = item.bot - item.top;

                    // compute angles from pixels
                    // non-dimensional bounding box center and size
                    float px = object.bbox_x/(float)det_w - 0.5;  // normalized coordinate from center of screen, + going right
                    float py = object.bbox_y/(float)det_h - 0.5;  // normalized coordinate from center of screen, + going down
                    if (px < -0.5) {
                        ROS_ERROR("weird, out of range px %f", px);
                        px = -0.5;
                    } else if (px > 0.5) {
                        ROS_ERROR("weird, out of range px %f", px);
                        px = 0.5;
                    }
                    if (py < -0.5) {
                        ROS_ERROR("weird, out of range py %f", py);
                        py = -0.5;
                    } else if (py > 0.5) {
                        ROS_ERROR("weird, out of range py %f", py);
                        py = 0.5;
                    }
                    // float hfov2 = 0.5*local_drone_state.hfov;
                    // float vfov2 = 0.5*local_drone_state.vfov;
                    float hfov2 = 0.5*hfov;
                    float vfov2 = 0.5*vfov;
                    float dx = 0.5 / tan(hfov2);
                    float dy = 0.5 / tan(vfov2);

                    // angle_x is the angle from centerline to vehicle right
                    // therefore bearing to target is vehicle heading + angle_x
                    // angle_y is the angle from centerline towards back of vehicle
                    // therefore pitch (tilt) angle to target is gimbal tilt + angle_y
                    object.angle_x = atan2(px, dx);
                    object.angle_y = atan2(py, dy);

                    if (object.angle_x < -hfov2) {
                        ROS_ERROR("weird, out of range object.angle_x %f", object.angle_x);
                        object.angle_x = -hfov2;
                    } else if (object.angle_x > hfov2) {
                        ROS_ERROR("weird, out of range object.angle_x %f", object.angle_x);
                        object.angle_x = hfov2;
                    }
                    if (object.angle_y < -vfov2) {
                        ROS_ERROR("weird, out of range object.angle_y %f", object.angle_y);
                        object.angle_y = -vfov2;
                    } else if (object.angle_y > vfov2) {
                        ROS_ERROR("weird, out of range object.angle_y %f", object.angle_y);
                        object.angle_y = vfov2;
                    }

                    ROS_INFO("debug %f %f %f, %f %f %f", px, dx, object.angle_x, py, dy, object.angle_y);

                    // class_name and class_id
                    object.class_name = std::string(objname);
                    object.class_id = item.classId;
                    object.name = object.class_name;  // for backwards compatibility
                    object.id = object.class_id;  // for backwards compatibility

                    /*
                    // fill out ground location
                    object.drone_location.lat = local_drone_state.vehicle_lat;
                    object.drone_location.lon = local_drone_state.vehicle_lon;
                    object.drone_location.alt = local_drone_state.vehicle_alt_asl;
                    object.drone_location.tilt = local_drone_state.gimbal_pit;
                    object.drone_location.hdg = local_drone_state.gimbal_true_hdg;

                    // object alt is ASL of the ground below drone, should be similar to home origin or base alt
                    object.ground_location.alt = local_drone_state.vehicle_alt_asl - local_drone_state.vehicle_alt_rel;
                    */

                    // confidence
                    object.confidence = item.score;



                    /* consider putting tracking into here
                    ////////////////////////////////////////////////////////////////////////
                    //                                                                    //
                    //  TRACKING                                                          //
                    //                                                                    //
                    ////////////////////////////////////////////////////////////////////////
                    // timing
                    //     time initial_detection
                    //     time most_recent_detection


                    // tracking id
                    // int32 track_id  # unique number per object tracked
                    */



                    // append to array
                    detection_raw_local.objects.push_back(object);



                    // internal counter
                    frame_id++;


                    // visualization
                    detection_display = false;
                    if (detection_display) {
                        cv::rectangle(img,cv::Point(item.left,item.top),cv::Point(item.right,item.bot),cv::Scalar(0,0,255),1,8,0);
                        putText(img, objname, cv::Point2f(item.left,item.top-4), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
                        // cv::imshow("result",img);
                        // cv::waitKey(1);
                    }
                }
                // ROS_ERROR("RGB5: fill detection_raw_local done");
            }
        } else {

            // FOR RAILCAR TESTS
            // FOR RAILCAR TESTS
            // FOR RAILCAR TESTS

            outputs.emplace_back(boxes);
            output += outputSize;
            auto bbox = *outputs.begin();

            for(const auto& item : bbox)
            {

                const char textarr[]= {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
                std::string chrname;
                chrname.push_back(textarr[item.classId]);

                // sstring<<"\t[class: " << chrname
                //        <<" left_x: "<< item.left
                //        <<"  top_y: " << item.top
                //        <<"  right: " << item.right
                //        <<"  bot: "<< item.bot
                //        <<" ],\n";

                // msg.data = msg.data + sstring.str();
                // sstring.str("");

                // FOR RAILCAR TESTS
                // object message
                birdseye_msgs::Object object;
                object.header.stamp = ros::Time::now();
                // bounding box center and size
                object.bbox_x = 0.5*(item.left + item.right);
                object.bbox_y = 0.5*(item.top + item.bot);
                object.bbox_w = item.right - item.left;
                object.bbox_h = item.bot - item.top;
                // class_name and class_id
                object.class_name = std::string(chrname);
                object.class_id = item.classId;
                object.name = object.class_name;  // for backwards compatibility
                object.id = object.class_id;  // for backwards compatibility
                // append to array
                detection_raw_local.objects.push_back(object);

                frame_id++;
                
                detection_display = false;
                if (detection_display) {
                    cv::rectangle(img,cv::Point(item.left,item.top),cv::Point(item.right,item.bot),cv::Scalar(0,0,255),1,8,0);
                    putText(img, chrname, cv::Point2f(item.left+30,item.top-4), cv::FONT_HERSHEY_PLAIN, 0.5,  cv::Scalar(0,0,255,255));
                    //cv::imshow("result",img);
                    //cv::waitKey(1);
                }
            }

            // FOR RAILCAR TESTS
            // group detected characters
            /*
            im_w = det_w;
            im_h = det_h;
            limit_y = 7.0;
            limit_x = 30.0;
            int left, top, right, bot;
            int old_left = 9999, old_top = 9999, old_right = 9999, old_bot = 9999;
            int l_top = 0;
            int l_left = 0;
            char word[100]="";
            int i=0;
            int counter = 0;
            int sline = 0;
            int w_top = 0, w_left = 0;
            int width;
            char labelstr[4096];
            int det_ind[int(boxes.size())];
            int ind = 0, root_ind = 0;

            std::sort(boxes.begin(), boxes.end(), compareByTopLeft);

            outputs.emplace_back(boxes);
            output += outputSize;
            auto bbox = *outputs.begin();

            for(const auto& item : bbox)
            {
                const char textarr[]= {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};
                std::string chrname;
                chrname.push_back(textarr[item.classId]);

                width = det_w * .006;
                if (width < 1)
                    width = 1;

                left = item.left;
                right = item.right;
                top = item.top;
                bot = item.bot;

                if (left < 0) left = 0;
                if (right > det_w - 1) right = det_w - 1;
                if (top < 0) top = 0;
                if (bot > det_h - 1) bot = det_h - 1;


                if ((i!=0) && ((fabs(old_left-left) > limit_x) || (fabs(old_top-top) > limit_y )))
                {
                    if (counter > 3)
                    {

                        if (!has_any_digits(labelstr) && sline == 0) {
                            sline = 1;
                            counter = 1;
                        }

                        if (sline == 0) {

                            sstring<<"\t[class: " << labelstr
                                   <<" left_x: "<< w_left
                                   <<"  top_y: " << w_top
                                   <<"  right: " << old_right
                                   <<"  bot: "<< old_bot
                                   <<" ],\n";

                            ROS_INFO("\n----------\n");
                            ROS_INFO("\n%s: %d, %d, %d, %d\n", labelstr, w_left, w_top, old_right, old_bot );
                            cv::rectangle(img,cv::Point(w_left,w_top),cv::Point(old_right,old_bot),cv::Scalar(0,0,255),1,8,0);
                            putText(img, labelstr, cv::Point2f(w_left,w_top-4), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));

                            for(int j=0; j<=counter; j++)
                            {
                                det_ind[ind] = root_ind;
                                ind++;
                                root_ind++;
                            }
                            counter = 0;
                        }

                        if (sline == 1 && counter > 3) {

                            sstring<<"\t[class: " << labelstr
                                   <<" left_x: "<< w_left
                                   <<"  top_y: " << w_top
                                   <<"  right: " << old_right
                                   <<"  bot: "<< old_bot
                                   <<" ],\n";

                            ROS_INFO("\n----------\n");
                            ROS_INFO("\n%s: %d, %d, %d, %d\n", labelstr, w_left, w_top, old_right, old_bot );
                            cv::rectangle(img,cv::Point(w_left,w_top),cv::Point(old_right,old_bot),cv::Scalar(0,0,255),1,8,0);
                            putText(img, labelstr, cv::Point2f(w_left,w_top-4), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));

                            for(int j=0; j<=counter; j++)
                            {
                                det_ind[ind] = root_ind;
                                ind++;
                                root_ind++;
                            }
                            counter = 0;
                            sline = 0;
                        }

                    } else {
                        root_ind = 0;
                        counter = 0;
                    }

                }
                if (sline ==1 && counter ==1) {
                    l_top = top;
                    l_left = left;
                }

                if (counter == 0) {
                    labelstr[0] = NULL;
                    w_top = top;
                    w_left = left;
                    counter++;
                    root_ind = i;
                    strcat(labelstr, chrname.c_str());
                }
                else {
                    counter++;
                    strcat(labelstr, chrname.c_str());
                }

                old_left = left;
                old_top  = top;
                old_right = right;
                old_bot = bot;
                i++;
                chrname = "";

            }
            if (sline == 1 && counter > 3) {
                if (w_left !=0 && w_top !=0) {

                    sstring<<"\t[class: " << labelstr
                           <<" left_x: "<< w_left
                           <<"  top_y: " << w_top
                           <<"  right: " << old_right
                           <<"  bot: "<< old_bot
                           <<" ],\n";

                    ROS_INFO("\n----------\n");
                    ROS_INFO("\n%s: %d, %d, %d, %d\n", labelstr, w_left, w_top, old_right, old_bot );
                    cv::rectangle(img,cv::Point(w_left,w_top),cv::Point(old_right,old_bot),cv::Scalar(0,0,255),1,8,0);
                    putText(img, labelstr, cv::Point2f(w_left,w_top-4), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
                }
            }
            if (sline == 0 && counter > 3) {
                if (w_left !=0 && w_top !=0) {

                    sstring<<"\t[class: " << labelstr
                           <<" left_x: "<< w_left
                           <<"  top_y: " << w_top
                           <<"  right: " << old_right
                           <<"  bot: "<< old_bot
                           <<" ],\n";

                    ROS_INFO("\n----------\n");
                    ROS_INFO("\n%s: %d, %d, %d, %d\n", labelstr, w_left, w_top, old_right, old_bot );
                    cv::rectangle(img,cv::Point(w_left,w_top),cv::Point(old_right,old_bot),cv::Scalar(0,0,255),1,8,0);
                    putText(img, labelstr, cv::Point2f(w_left,w_top-4), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,0,255,255));
                }
            }

            cv::imshow("result",img);
            cv::waitKey(1);
            frame_id++;
            i = -1;
            */

        }

        ///////////////////////////////////////////////////////////////////////////////////////
        //                                                                                   //
        // queue up to synchronize with incoming drone_state_, which might be delayed        //
        //                                                                                   //
        ///////////////////////////////////////////////////////////////////////////////////////
        if (detection_raw_local.objects.size() > 0)
            detection_raw_queue.push_back(detection_raw_local);

        ///////////////////////////////////////////////////////////////////////////////////////
        //                                                                                   //
        // copy latest detection raw for visualization                                       //
        //                                                                                   //
        ///////////////////////////////////////////////////////////////////////////////////////
        {
            ROS_INFO("RGB DETECTION PIPE: (3) COPY DETECTIONS");
            std::lock_guard<std::mutex> lock(detection_raw_viz_mutex);
            detection_raw_viz.width = detection_raw_local.width;
            detection_raw_viz.height = detection_raw_local.height;
            detection_raw_viz.objects.clear();
            for (int i = 0; i < detection_raw_local.objects.size(); ++i)
                detection_raw_viz.objects.push_back(detection_raw_local.objects[i]);
            // ROS_ERROR("RGB5: clear detection_raw_viz done");

        }

        ///////////////////////////////////////////////////////////////////////////////////////
        //                                                                                   //
        //                                                                                   //
        //                                                                                   //
        // check for syncd detection raw, fill in the dorne data and publish to localizer    //
        //                                                                                   //
        //                                                                                   //
        //                                                                                   //
        ///////////////////////////////////////////////////////////////////////////////////////
        // grab drone state
        std::vector<DroneState>::iterator drone_state_it;
        DroneState local_drone_state;
        if (!drone_state_queue_.empty()) {
            // copy oldest member of DroneState drone_state_queue_;
            std::lock_guard<std::mutex> lock(bird_status_mutex_);
            drone_state_it = drone_state_queue_.begin();
            local_drone_state = DroneState(*drone_state_it);
        }

        // - look for synchronized drone state + detection timestamps
        // TODO: ROS_ERROR("RGB5: clear detection_raw_syn?");
        bool syncd = false;
        // - look at oldest element of detection_raw_queue,
        //   check if timestamp has caught up to drone_state_ time
        std::vector<birdseye_msgs::DetectionResult>::iterator syncd_it;
        ROS_INFO("SYNC: queue size %d", (int)detection_raw_queue.size());
        if (detection_raw_queue.size() > 0) {
            for (std::vector<birdseye_msgs::DetectionResult>::iterator it = detection_raw_queue.begin();
                 it != detection_raw_queue.end(); it++) {

                ROS_INFO("SYNC: queue objects size %d", (int)(it->objects.size()));
                if (it->objects.size() > 0) {
                    //
                    // SYNCD CHECK
                    //
                    double t0diff = it->objects[0].header.stamp.toSec() - local_drone_state.status_stamp_last.toSec();
                    double t1diff = local_drone_state.status_stamp.toSec() - it->objects[0].header.stamp.toSec();
                    ROS_INFO("SYNC: check timestamps: state %f detect %f", it->objects[0].header.stamp.toSec(), it->objects[0].header.stamp.toSec());

                    // if drone_state_ is lagging, read it again
                    while (t1diff < 0-tmargin && drone_state_queue_.size() > 1) {
                        // move to next DroneState drone_state_;
                        std::lock_guard<std::mutex> lock(bird_status_mutex_);  // DANGER: NESTED LOCKS
                        if (drone_state_queue_.size() > 1) {
                            ROS_INFO("SYNC: t0diff %f t1diff %f drone_state old, get next drone state", t0diff, t1diff);
                            drone_state_queue_.erase(drone_state_queue_.begin());
                            drone_state_it = drone_state_queue_.begin();
                            local_drone_state = DroneState(*drone_state_it);

                            // update t0diff and t1diff
                            t0diff = it->objects[0].header.stamp.toSec() - local_drone_state.status_stamp_last.toSec();
                            t1diff = local_drone_state.status_stamp.toSec() - it->objects[0].header.stamp.toSec();

                        } else {
                            ROS_WARN("SYNC: t0diff %f t1diff %f drone_state too old, no new ones!", t0diff, t1diff);
                            break;
                        }
                    }

                    if (t0diff >= 0-tmargin && t1diff >= 0-tmargin) {

                        ROS_INFO("SYNC: t0diff %f t1diff %f - synchronized", t0diff, t1diff);

                        // perfect scenario, detection falls between two drone_state_ readings
                        syncd_it = it;
                        syncd = true;
                    } else {
                        if (syncd) {
                            ROS_INFO("SYNC: t0diff %f t1diff %f - synchronize done, break out of search", t0diff, t1diff);
                            // previously syncd but now exceeding the time limit
                            break;
                        }
                    }
                }
            }
        }

        ///////////////////////////////////////////////////////////////////////////////////////
        //                                                                                   //
        // copy syncd detection for publishing                                               //
        //                                                                                   //
        ///////////////////////////////////////////////////////////////////////////////////////
        // - if times are synchronized, erase up to latest detection result
        if (syncd) {

            ROS_INFO("RGB DETECTION PIPE: (3) COPY DETECTIONS");
            std::lock_guard<std::mutex> lock(detection_raw_syn_mutex);

            ROS_WARN("SYNC: found a synchronized detection_raw_syn");
            // - copy out latest element to local storage
            // detection_raw_syn = birdseye_msgs::DetectionResult(*syncd_it);
            detection_raw_syn.width = syncd_it->width;
            detection_raw_syn.height = syncd_it->height;
            detection_raw_syn.objects.clear();
            for (int i = 0; i < syncd_it->objects.size(); ++i)
                detection_raw_syn.objects.push_back(syncd_it->objects[i]);

            // - and erase it
            detection_raw_queue.erase(detection_raw_queue.begin(), syncd_it+1);

            // todo: should we also copy latest detection to detection_raw_syn for visualization?
            // or just use detection_raw_queue's end element

            // - interpolate heading for the detection time and store in detection_raw_syn
            local_drone_state.UpdateGimbalTrueHdg(detection_raw_syn.objects[0].header.stamp.toSec());
            for (int i = 0; i < detection_raw_syn.objects.size(); ++i) {

                // - copy drone_location into detection_raw_syn (fill out ground location)
                detection_raw_syn.objects[i].drone_location.lat = local_drone_state.vehicle_lat;
                detection_raw_syn.objects[i].drone_location.lon = local_drone_state.vehicle_lon;
                detection_raw_syn.objects[i].drone_location.alt = local_drone_state.vehicle_alt_asl;
                detection_raw_syn.objects[i].drone_location.tilt = local_drone_state.gimbal_pit;
                detection_raw_syn.objects[i].drone_location.hdg = local_drone_state.gimbal_true_hdg;

                // and object alt is ASL of the ground below drone, should be similar to home origin or base alt
                detection_raw_syn.objects[i].ground_location.alt =
                    local_drone_state.vehicle_alt_asl - local_drone_state.vehicle_alt_rel;
            }

            // - publish it: detection_raw_syn is used for detection_raw_pub.publish(detection_raw_syn);
        } else {
            ROS_INFO("RGB DETECTION PIPE: (4) PUBLISH DETECTION (skip because of timing)");
        }


        ///////////////////////////////////////////////////////////////////////////////////////
        //                                                                                   //
        // just some visual debugging aide                                                   //
        //                                                                                   //
        ///////////////////////////////////////////////////////////////////////////////////////
        detection_display = false;
        if (detection_display) {
            cv::imshow("result",img);
            cv::waitKey(1);
        }
    }

    static void
    on_pad_added (GstElement *element,
                  GstPad *pad,
                  gpointer data)
    {
        GstPad *sinkpad;
        GstCaps *caps;
        GstElement *parser = (GstElement *) data;
        GstStructure *str;
        /* We can now link this pad with the h264parse sink pad */
        caps =  gst_pad_get_current_caps (pad);

        sinkpad = gst_element_get_static_pad (parser, "sink");
        gst_pad_link (pad, sinkpad);
        gst_object_unref (sinkpad);
    }


    static void install_probe_for_metadata_mux (GstElement * monitor)
    {
  	//GstElement *src;
 	 GstPad *pad;
	 ROS_INFO("@@@@@@@@@@@@@@install_probe_for_metadata_mux@@@@@@@@@@@@@@@@\n");
 	 //src = gst_bin_get_by_name (GST_BIN (pipeline), "vts");
	  pad = gst_element_get_static_pad (monitor, "src");

 	 gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_BUFFER, meta_data_handle_buffer, NULL, NULL);

  	gst_object_unref (pad);
  	//gst_object_unref (videotestsrc);
    }

    static GstPadProbeReturn
	meta_data_handle_buffer (GstPad * pad, GstPadProbeInfo * info,
    gpointer user_data)
    {  
	 
	static GstClockTime timestamp = 0;
  	GstBuffer *klvbuffer =  gst_buffer_new_allocate(NULL, PACKET_LENGTH, NULL);
  	gsize buf_size=0, klv_size = sizeof(packetBuffer);
	ROS_INFO("@@@@@@@@@@@@@@@@@@@meta_data_handle_buffer@@@@@@@@@@@@@@\n");
#if 1   
	makePacket(packetBuffer);
  	buf_size = gst_buffer_fill(klvbuffer, 0, packetBuffer, klv_size);
  	//buf_size = gst_buffer_fill(klvbuffer, 0, (guint8 *)&rtp_KLV_frame_data, klv_size);

 	GST_BUFFER_PTS (klvbuffer) = timestamp;
  	GST_BUFFER_DTS (klvbuffer) = timestamp;
  	GST_BUFFER_DURATION (klvbuffer) = gst_util_uint64_scale_int (1, GST_SECOND, 30);

  	GstFlowReturn ret;
  	g_signal_emit_by_name(rec_metasrc, "push-buffer", klvbuffer, &ret);

  	klv_frame_counter += 1;
 	timestamp += GST_BUFFER_DURATION (klvbuffer);

  	if (ret != GST_FLOW_OK) {
    		/* something wrong, stop pushing */
   		ROS_INFO("KLV_frame_insertion_failed:%d\n", klv_frame_counter);
    		gst_buffer_unref (klvbuffer);
    		//Kill the pipeline
		g_main_loop_quit (loop);
 	}
  	ROS_INFO("KLV_frame_insertion_successful:%d  -%" GST_TIME_FORMAT "\n", klv_frame_counter, GST_TIME_ARGS(timestamp));
#endif

	return GST_PAD_PROBE_OK;
     }

    //gst funtion start
    void* stream_start_rgb()
    {
       // printf("madhu stream_star_rgb: %l\n", ros::time::now());     
        /*Declare and initalize Elements, pads, caps, bus etc*/
        //auto t1 = std::chrono::high_resolution_clock::now();        // testing initialisation time
        stream_obj_t *ps_stream = NULL;
        GstBus *ps_bus = NULL;

        GstElement *ps_pipeline, *ps_src, *ps_capsfilter, *ps_tee;

        GstElement *rec_queue, *rec_resize, *rec_resize_cap, *rec_enc, *rec_capsfilter, *rec_mux, *rec_sink;
        GstPad *tee_rec_pad,*queue_rec_pad;
	//GstElement *rec_metasrc;

        GstElement *overlay_queue;
        GstElement *nested_tee, *nested_queue;
        GstPad *nested_tee_pad, *nested_queue_pad;

        GstElement *st_queue, *st_resize, *st_resize_cap, *st_capsfilter, *st_mux, *st_sink;
        GstPad *tee_st_pad,*queue_st_pad;

        GstElement *lp_queue, *lp_scale, *lp_capsfilter, *lp_sink;
        GstPad *queue_lp_pad, *tee_lp_pad;

        GstElement *ss_queue, *ss_scale, *ss_capsfilter, *ss_sink;
        GstPad *queue_ss_pad, *tee_ss_pad;

        GstElement *parse;

        //Video_input
        GstElement *qtdemux_v, *parse_v, *dec_v;

        GstCaps *ps_caps = NULL;
        char ac_temp[128], ac_src_caps[1024];

        /* Frame_rate */
        GstElement *videorate;
        GstElement * s_nv1, *s_nv1_cap, *s_nv2, *s_nv2_cap, *s_videorate;

        ///////////////////////////////////////////////////////////////////////
        // read stuff from ros param
        ///////////////////////////////////////////////////////////////////////
        //  RGB
        std::string vision_prefix = "/drone/vision/video1";

        // MISC  // ask MCW what this is?
        int split_dur                           = ros::param::param<int>(vision_prefix + "/split_duration"             , 0);  // ask MCW what is this?

        // CAMERA
        int camera_framerate_override           = ros::param::param<int>(vision_prefix + "/camera/framerate"           , 30);
        int camera_sensor_id                    = ros::param::param<int>(vision_prefix + "/camera/sensor_id"           , 0);
        int camera_auto_exposure                = ros::param::param<int>(vision_prefix + "/camera/auto_exposure"       , 1);

        // WRITE
        // dangerous if framerate is non-zero, write stops
        int write_framerate_override            = ros::param::param<int>(vision_prefix + "/write/framerate"            , 0);
        int write_width_override                = ros::param::param<int>(vision_prefix + "/write/width_override"       , 3840);
        int write_height_override               = ros::param::param<int>(vision_prefix + "/write/height_override"      , 2160);
        int write_bitrate                       = ros::param::param<int>(vision_prefix + "/write/bitrate_override"     , 35000000);
        std::string write_qp_range      = ros::param::param<std::string>(vision_prefix + "/write/qp_range"             , "-1,-1:-1,-1:-1,-1");
        std::string write_level         = ros::param::param<std::string>(vision_prefix + "/write/level"                , "high5");
        std::string write_muxer         = ros::param::param<std::string>(vision_prefix + "/write/muxer"                , "mpegtsmux");
        int write_rate_control                  = ros::param::param<int>(vision_prefix + "/write/rate_control"         , 1);
        int write_preset_level                  = ros::param::param<int>(vision_prefix + "/write/preset_level"         , 0);
        int write_EnableTwopassCBR              = ros::param::param<int>(vision_prefix + "/write/EnableTwopassCBR"     , 0);
        int write_vbv_size                      = ros::param::param<int>(vision_prefix + "/write/vbv_size"             , 10);
        int write_temporal_tradeoff             = ros::param::param<int>(vision_prefix + "/write/temporal_tradeoff"    , 0);
        int write_iframeinterval                = ros::param::param<int>(vision_prefix + "/write/iframeinterval"       , 0);
        int write_profile                       = ros::param::param<int>(vision_prefix + "/write/profile"              , 1);
        int write_b_frames                      = ros::param::param<int>(vision_prefix + "/write/b_frames"             , 0);
        int write_insert_sps_pps                = ros::param::param<int>(vision_prefix + "/write/insert_sps_pps"       , 1);
        write_test_textoverlay                 = ros::param::param<bool>(vision_prefix + "/write/overlay/enable"       , true);
        int write_enc_type                      = ros::param::param<int>(vision_prefix + "/write/enc_type"             , ENC_HW);

        // STREAM
        std::string stream_target_ip    = ros::param::param<std::string>(vision_prefix + "/stream_target_ip_override"  , "192.168.168.4");
        int stream_target_port                  = ros::param::param<int>(vision_prefix + "/stream_target_port_override", 13001);
        stream_target_ip                = ros::param::param<std::string>(vision_prefix + "/stream_target_ip"           , stream_target_ip);
        stream_target_port                      = ros::param::param<int>(vision_prefix + "/stream_target_port"         , stream_target_port);
        stream_target_ip                = ros::param::param<std::string>(vision_prefix + "/stream/target_ip"           , stream_target_ip);
        stream_target_port                      = ros::param::param<int>(vision_prefix + "/stream/target_port"         , stream_target_port);

        // dangerous if framerate is non-zero, stream stops
        int stream_framerate_override           = ros::param::param<int>(vision_prefix + "/stream/framerate"           , 0);
        int stream_width_override               = ros::param::param<int>(vision_prefix + "/stream/width_override"      , 960);
        int stream_height_override              = ros::param::param<int>(vision_prefix + "/stream/height_override"     , 540);

        stream_test_textoverlay                = ros::param::param<bool>(vision_prefix + "/stream/overlay/enable"      , false);
        bool overlay_target                    = ros::param::param<bool>(vision_prefix + "/stream/overlay/target"      , true);
        double overlay_x                     = ros::param::param<double>(vision_prefix + "/stream/overlay/x"           , 0.5);
        double overlay_y                     = ros::param::param<double>(vision_prefix + "/stream/overlay/y"           , 0.5);

        int stream_enc_type                     = ros::param::param<int>(vision_prefix + "/stream/enc_type"            , ENC_HW);

        int stream_rate_control                 = ros::param::param<int>(vision_prefix + "/stream/rate_control"        , 1);
        int stream_bitrate                      = ros::param::param<int>(vision_prefix + "/stream/bitrate_override"    , 400000);
        int stream_preset_level                 = ros::param::param<int>(vision_prefix + "/stream/preset_level"        , 0);
        std::string stream_qp_range     = ros::param::param<std::string>(vision_prefix + "/stream/qp_range"            , "-1,-1:-1,-1:-1,-1");
        bool stream_EnableTwopassCBR           = ros::param::param<bool>(vision_prefix + "/stream/EnableTwopassCBR"    , true);
        int stream_vbv_size                     = ros::param::param<int>(vision_prefix + "/stream/vbv_size"            , 10);
        int stream_temporal_tradeoff            = ros::param::param<int>(vision_prefix + "/stream/temporal_tradeoff"   , 0);
        int stream_iframeinterval               = ros::param::param<int>(vision_prefix + "/stream/iframe_interval"     , 60);
        std::string stream_muxer        = ros::param::param<std::string>(vision_prefix + "/stream/muxer"               , "mpegtsmux");
        int stream_alignment                    = ros::param::param<int>(vision_prefix + "/stream/alignment"           , 7);
        int stream_si_interval                  = ros::param::param<int>(vision_prefix + "/stream/si_interval"         , 1000);
        std::string stream_level        = ros::param::param<std::string>(vision_prefix + "/stream/level"               , "high5");

        // h264 only
        int stream_profile                      = ros::param::param<int>(vision_prefix + "/stream/profile"             , 2);
        int stream_b_frames                     = ros::param::param<int>(vision_prefix + "/stream/b_frames"            , 0);
        int stream_insert_sps_pps               = ros::param::param<int>(vision_prefix + "/stream/insert_sps_pps"      , 1);

        // DETECTION
        int detect_width_override               = ros::param::param<int>(vision_prefix + "/detect/width_override"      , 864);
        int detect_height_override              = ros::param::param<int>(vision_prefix + "/detect/height_override"     , 480);
        std::string detect_format_enc   = ros::param::param<std::string>(vision_prefix + "/detect/format_encoded"      , "I420");
        std::string detect_format_dec   = ros::param::param<std::string>(vision_prefix + "/detect/format_decoded"      , "BGR");
        double detect_framerate              = ros::param::param<double>(vision_prefix + "/detect/framerate"           , 5.0);

        // SNAPSHOTTER
        bool snapshot_enabled                  = ros::param::param<bool>(vision_prefix + "/snapshot/enabled"           , true);
        int snapshot_width_override             = ros::param::param<int>(vision_prefix + "/snapshot/width_override"    , 3840);
        int snapshot_height_override            = ros::param::param<int>(vision_prefix + "/snapshot/height_override"   , 2160);
        std::string snapshot_format_enc = ros::param::param<std::string>(vision_prefix + "/snapshot/format_encoded"    , "I420");
        std::string snapshot_format_dec = ros::param::param<std::string>(vision_prefix + "/snapshot/format_decoded"    , "BGRx");

        // debug tool
        bool debug_file_input                  = ros::param::param<bool>(vision_prefix + "/debug/file_input"           , false);
        std::string video_fname         = ros::param::param<std::string>(vision_prefix + "/debug/fname"                , "/data/test.mp4");
        // end of ros param block
        //auto t1stop = std::chrono::high_resolution_clock::now();
	    //float total = std::chrono::duration<float, milli>(t1stop - t1).count();
	    //std::cout<<"\n\n["<<std::this_thread::get_id()<<"] Time taken for initialization stream start rgb | :"<<total<< "ms\n\n";

        if (!r_265)
          ROS_INFO("Encoder type is X264");
        else
          ROS_INFO("Encoder type is X265");




        /* stream object setup */
        ps_stream = (stream_obj_t *)malloc(sizeof(stream_obj_t));
        if (ps_stream == NULL)
        {
            ROS_ERROR("Error in allocating memory for object");
            return(NULL);
        }
        memset(ps_stream, 0,sizeof(stream_obj_t));

        /* Init Gstreamer components */
        gst_init(0, NULL);

        /* Create the pipeline */
        ps_pipeline = ps_stream->s_pipeline = gst_pipeline_new("pipeline");
        ERR_CHECK(ps_pipeline == NULL, NULL);

        /* Add a bus to the pipeline */
        ps_bus = gst_pipeline_get_bus (GST_PIPELINE(ps_pipeline));
        gst_bus_set_sync_handler (ps_bus, cb_message, (gpointer) ps_stream, NULL);
        gst_object_unref(ps_bus);

        /* Create the elements and then start linking them */
        /* Src */
        if (debug_file_input)
        {
            //std::cout<<"Thread ID | "<<std::this_thread::get_id()<<" | reading from file\n\n";
            char in[100];
            // test from file input
            ps_src = element_create(ps_pipeline, "filesrc");
            ERR_CHECK(ps_src == NULL, NULL);
            strcpy(in, video_fname.c_str());
            g_object_set(G_OBJECT(ps_src), "location", in, NULL);//file name

            qtdemux_v = element_create(ps_pipeline, "qtdemux");
            ERR_CHECK(qtdemux_v == NULL, NULL);
            g_object_set(G_OBJECT(qtdemux_v), "name", "demux", NULL);

            parse_v = element_create(ps_pipeline, "h264parse");
            ERR_CHECK( parse_v == NULL, NULL);

            dec_v = element_create(ps_pipeline, "omxh264dec");
            ERR_CHECK( dec_v== NULL, NULL);
             
            /* Tee for creating Two branches */
            // ps_tee = element_create(ps_pipeline, "nvtee");
            ps_tee = element_create(ps_pipeline, "tee");
            ERR_CHECK(ps_tee == NULL, NULL);

            if (gst_element_link_many(ps_src, qtdemux_v , NULL) !=TRUE)
            {
                ROS_ERROR("Error linking elements of pipeline");
                return(NULL);
            }

            /* Link upstream elements */
            if (gst_element_link_many( parse_v, dec_v, ps_tee, NULL) !=TRUE)
            {
                ROS_ERROR("Error linking elements of pipeline");
                return(NULL);
            }

            g_signal_connect (qtdemux_v, "pad-added", G_CALLBACK (on_pad_added), parse_v);

        }
        else if (camera_sensor_id != -1)
        {
            ps_src = element_create(ps_pipeline, "nvcamerasrc");
            ERR_CHECK(ps_src == NULL, NULL);
            g_object_set(G_OBJECT(ps_src), "sensor-id", camera_sensor_id, NULL);
            g_object_set(G_OBJECT(ps_src), "auto-exposure", (gint)camera_auto_exposure, NULL);
            // g_object_set(G_OBJECT(ps_src), "exposure-time", (gint)0.005, NULL);
            strcpy(ac_src_caps, "video/x-raw(memory:NVMM),");
        }
        else
        {
            ps_src = element_create(ps_pipeline, "videotestsrc");
            ERR_CHECK(ps_src == NULL, NULL);
            strcpy(ac_src_caps, "video/x-raw,");
            g_object_set(G_OBJECT(ps_src), "is-live", 1, NULL);
        }

        if (!debug_file_input)
        {
            /* Capsfilter */
            ps_capsfilter = element_create(ps_pipeline, "capsfilter");
            ERR_CHECK(ps_capsfilter == NULL,  NULL);
            sprintf(ac_temp, "width=%d,", write_width_override);
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "height=%d,", write_height_override);
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "format=%s", "I420");
            strcat(ac_src_caps, ac_temp);

            if ((!r_265) && ((!write_framerate_override)||(!stream_framerate_override)))  // FIXME: what if camera_framerate_override is 0?
            {
                sprintf(ac_temp, ",framerate=(fraction)%d/1", camera_framerate_override);
                strcat(ac_src_caps, ac_temp);
            }

            ps_caps = gst_caps_from_string(ac_src_caps);
            g_object_set(G_OBJECT(ps_capsfilter), "caps", ps_caps, NULL);
            gst_caps_unref(ps_caps);
            ROS_DEBUG("Src caps = %s", ac_src_caps);

            /* Tee for creating Two branches */
            // ps_tee = element_create(ps_pipeline, "nvtee");
            ps_tee = element_create(ps_pipeline, "tee");
            ERR_CHECK(ps_tee == NULL, NULL);

            /* Link upstream elements */
            if (gst_element_link_many(ps_src, ps_capsfilter, ps_tee, NULL) !=TRUE)
            {
                ROS_ERROR("Error linking elements of pipeline");
                return(NULL);
            }
        }

        //Enable record and stream
        if (r_enable) {

            ROS_INFO("Enabled RGB Record and streaming ");

            /* Queue */
            rec_queue = element_create(ps_pipeline, "queue");
            ERR_CHECK(rec_queue == NULL, NULL);
            queue_rec_pad = gst_element_get_static_pad(rec_queue, "sink");

            /* request pads from tee */
            // tee_rec_pad = gst_element_get_request_pad(ps_tee, "pre_src");
            tee_rec_pad = gst_element_get_request_pad(ps_tee, "src_%u");
            ERR_CHECK(tee_rec_pad == NULL, NULL);

            /* Link I/O pads */
            if ( gst_pad_link (tee_rec_pad, queue_rec_pad) != GST_PAD_LINK_OK)
            {
                ROS_ERROR("Tee could not be linked. \n");
                return(NULL);
            }
            gst_object_unref(queue_rec_pad);

            /* Frame rate control
                   - if custom write rate is specified, use nvvidconv + videorate
                   - else skip the videorate plugin
            */
            if (write_framerate_override) {  // custom write framerate control, use videorate plugin
                ROS_INFO("write framerate (fps) assigned [%d], file write stalls\n", write_framerate_override);
                rec_resize = element_create(ps_pipeline, "nvvidconv");
                ERR_CHECK(rec_resize == NULL, NULL);
                rec_resize_cap = element_create(ps_pipeline, "capsfilter");
                ERR_CHECK(rec_resize_cap == NULL, NULL);
		if(debug_file_input)
                  strcpy(ac_src_caps, "video/x-raw,");
                else
		strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");
                sprintf(ac_temp, ", width=%d", write_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", height=%d", write_height_override);
                strcat(ac_src_caps, ac_temp);
                // sprintf(ac_temp, ", format=I420");   // testing raw filesink
                // strcat(ac_src_caps, ac_temp);

                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(rec_resize_cap), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

                /* videorate plugin */
                videorate = element_create(ps_pipeline,"videorate");
                ERR_CHECK(videorate == NULL, NULL);
                g_object_set(G_OBJECT(videorate),"max-rate", write_framerate_override, NULL);  // TODO: is this write fps?

            }
            else {  // no write framerate specified, good old nvvidconv only
                ROS_INFO("write framerate not assigned, using camera framerate (fps) only if h264 [%d]", camera_framerate_override);
                //video resize and fps control
                rec_resize = element_create(ps_pipeline, "nvvidconv");
                ERR_CHECK(rec_resize == NULL, NULL);

                rec_resize_cap = element_create(ps_pipeline, "capsfilter");
                ERR_CHECK(rec_resize_cap == NULL, NULL);
                strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");
                sprintf(ac_temp, ", width=%d", write_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", height=%d", write_height_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", format=I420");   // testing raw filesink
                strcat(ac_src_caps, ac_temp);
		if(debug_file_input)
          	    strcpy(ac_src_caps, "video/x-raw");
        	else
          	strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");

        	sprintf(ac_temp, ", width=%d", write_width_override);
      	  	strcat(ac_src_caps, ac_temp);
        	sprintf(ac_temp, ", height=%d", write_height_override);
        	strcat(ac_src_caps, ac_temp);

                /* some legacy stuff for h264, leave it for now, but not tested */
                if (!r_265)
                {
                    sprintf(ac_temp, ", framerate=(fraction)%d/1", camera_framerate_override);
                    strcat(ac_src_caps, ac_temp);
                }

                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(rec_resize_cap), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

                write_test_textoverlay = false;  // does not work without write_framerate_override
            }

            /* add textoverlay if specified */
            if (write_test_textoverlay)
            {
                r_text_overlay = element_create(ps_pipeline, "textoverlay");
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
                std::string test1_str = "";  // "ok\nthis is a sentence\n";
                std::string whole_str = test1_str + iso_time_str;
                g_object_set(G_OBJECT(r_text_overlay), "text", whole_str.c_str(), NULL);

                // g_object_set(G_OBJECT(r_text_overlay),"text","SAMPLE TEXT OVERLAY",NULL);
                ERR_CHECK(r_text_overlay == NULL, NULL);
            } else {
                r_text_overlay = element_create(ps_pipeline, "identity");
                ERR_CHECK(r_text_overlay == NULL, NULL);
            }

            /* Video encode */
            if (write_enc_type == ENC_HW) {
                if (!r_265)
                    rec_enc = element_create(ps_pipeline, "omxh264enc");
                else
                    rec_enc = element_create(ps_pipeline, "omxh265enc");

                ERR_CHECK(rec_enc == NULL, NULL);

                g_object_set(G_OBJECT(rec_enc), "qp-range", write_qp_range.c_str(), NULL);
                g_object_set(G_OBJECT(rec_enc), "control-rate", write_rate_control, NULL);
                g_object_set(G_OBJECT(rec_enc), "bitrate", write_bitrate, NULL);
                g_object_set(G_OBJECT(rec_enc), "preset-level", write_preset_level, NULL);
                g_object_set(G_OBJECT(rec_enc), "EnableTwopassCBR", write_EnableTwopassCBR, NULL);
                g_object_set(G_OBJECT(rec_enc), "vbv-size", write_vbv_size, NULL);
                g_object_set(G_OBJECT(rec_enc), "temporal-tradeoff", write_temporal_tradeoff, NULL);
                g_object_set(G_OBJECT(rec_enc), "iframeinterval", write_iframeinterval, NULL);

                if (!r_265)
                {
                    g_object_set(G_OBJECT(rec_enc), "profile", write_profile, NULL);
                    g_object_set(G_OBJECT(rec_enc), "num-B-Frames", write_b_frames, NULL);
                    g_object_set(G_OBJECT(rec_enc), "insert-sps-pps", write_insert_sps_pps, NULL);
                }

                ROS_INFO("write qp range [%s]", write_qp_range.c_str());

                rec_capsfilter = element_create(ps_pipeline, "capsfilter");
                ERR_CHECK(rec_capsfilter == NULL,NULL);

                if (!r_265)
                    strcpy(ac_src_caps, "video/x-h264");
                else
                    strcpy(ac_src_caps, "video/x-h265");
                sprintf(ac_temp, ", level=(string)%s", write_level.c_str());
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", stream-format=%s", "byte-stream");
                strcat(ac_src_caps, ac_temp);
                if (!r_265)
                {
                    sprintf(ac_temp, ", framerate=(fraction)%d/1", camera_framerate_override);
                    strcat(ac_src_caps, ac_temp);
                }

                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(rec_capsfilter), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

            }
            else { // not ENC_HW, so software encoding?
                if (!r_265)
                    rec_enc = element_create(ps_pipeline, "x264enc");
                else
                    rec_enc = element_create(ps_pipeline, "x265enc");

                ERR_CHECK(rec_enc == NULL, NULL);
                g_object_set(G_OBJECT(rec_enc), "bitrate", write_bitrate/1000, NULL);

                rec_capsfilter = element_create(ps_pipeline,"capsfilter");
                ERR_CHECK(rec_capsfilter == NULL, NULL);
            }

            /* Muxer & Sink */
            if (!r_265) {  // h264
                // parse = element_create(ps_pipeline , "h264parse" );
                // rec_sink = element_create(ps_pipeline, "splitmuxsink");
                rec_mux = element_create(ps_pipeline, write_muxer.c_str());
                rec_sink = element_create(ps_pipeline, "filesink");
		//adding appsrc
		//rec_metasrc = element_create(ps_pipeline, "appsrc");
		rec_metasrc = element_create(ps_pipeline, "metasrc");
		//adding metadata
		GstCaps *klv_caps = gst_caps_new_simple("meta/x-klv",
                         "parsed", G_TYPE_BOOLEAN, TRUE,
                     //    "sparse", G_TYPE_BOOLEAN, TRUE,
                     //    "is_live", G_TYPE_BOOLEAN, TRUE,
                         nullptr);
		
		g_object_set(G_OBJECT(rec_metasrc), "caps", klv_caps, NULL);
		g_object_set(G_OBJECT(rec_metasrc), "format", GST_FORMAT_TIME, NULL);
  		g_object_set(G_OBJECT(rec_metasrc), "stream-type", 0, NULL);
  		g_object_set(G_OBJECT(rec_metasrc), "do-timestamp", 1, NULL);


                ERR_CHECK(rec_sink == NULL, NULL);

                ROS_INFO("RGB Streaming fname: %s", this->fname.c_str());

                g_object_set(G_OBJECT(rec_sink), "location", this->fname.c_str(), NULL);
                //g_object_set(G_OBJECT(rec_sink), "muxer", rec_mux , NULL);
                // g_object_set(G_OBJECT(rec_sink), "async-handling", false, NULL);
                g_object_set(G_OBJECT(rec_sink), "async", true, NULL);  // for filesink

                // if (split_dur != 0)
                //     g_object_set(G_OBJECT(rec_sink), "max-size-time", split_dur * (long long int)1000000000 , NULL);
		//adapa install_probe_for_metadata_mux (rec_enc);

                if (write_framerate_override) {
                    if (gst_element_link_many(rec_queue, rec_resize, rec_resize_cap, videorate, r_text_overlay, rec_enc, rec_capsfilter, rec_mux, rec_sink, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of record pipeline");
                        return(NULL);
                    }
		    if (gst_element_link_many(rec_metasrc, rec_mux, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of klv mux pipeline");
                        return(NULL);
                    }

                }
                else {
                    if (gst_element_link_many(rec_queue, rec_resize, rec_resize_cap, rec_enc, rec_capsfilter, rec_mux, rec_sink, NULL) != TRUE )
                    {
                        ROS_ERROR("Error linking elements of record pipeline");
                        return(NULL);
                    }
		    if (gst_element_link_many(rec_metasrc, rec_mux, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of klv mux pipeline");
                        return(NULL);
                    }
                }
            }
            else {  // h265

                // record video on disk using splitmuxsink
                // rec_mux = gst_element_factory_make(write_muxer.c_str(), NULL);
                // rec_sink = element_create(ps_pipeline, "splitmuxsink");

                // record video on disk using filesink
                rec_mux = element_create(ps_pipeline, write_muxer.c_str());
                rec_sink = element_create(ps_pipeline, "filesink");
		 //adding appsrc
                //rec_metasrc = element_create(ps_pipeline, "appsrc");
                rec_metasrc = element_create(ps_pipeline, "metasrc");
                //adding metadata
                GstCaps *klv_caps = gst_caps_new_simple("meta/x-klv",
                         "parsed", G_TYPE_BOOLEAN, TRUE,
                       //  "sparse", G_TYPE_BOOLEAN, TRUE,
                      //   "is_live", G_TYPE_BOOLEAN, TRUE,
                         nullptr);

                g_object_set(G_OBJECT(rec_metasrc), "caps", klv_caps, NULL);
                g_object_set(G_OBJECT(rec_metasrc), "format", GST_FORMAT_TIME, NULL);
                g_object_set(G_OBJECT(rec_metasrc), "stream-type", 0, NULL);
                g_object_set(G_OBJECT(rec_metasrc), "do-timestamp", 1, NULL);
	
	
                ERR_CHECK(rec_sink == NULL, NULL);

                ROS_INFO("RGB Streaming fname: [%s]", this->fname.c_str());

                g_object_set(G_OBJECT(rec_sink), "location", this->fname.c_str(), NULL);
                // adapa g_object_set(G_OBJECT(rec_sink), "muxer", rec_mux , NULL);
                // g_object_set(G_OBJECT(rec_sink), "async-handling", true, NULL);  // for splitmuxsink
                g_object_set(G_OBJECT(rec_sink), "async", true, NULL);  // for filesink

                /*GstElement *vidconv,*vidconvcap;
                vidconv = element_create(ps_pipeline,"videoconvert");
                ERR_CHECK(vidconv == NULL, NULL);
                vidconvcap = element_create(ps_pipeline,"capsfilter");
                ERR_CHECK(vidconvcap == NULL, NULL);*/

                // option only for splitmuxsink
                // if (split_dur != 0)
                //     g_object_set(G_OBJECT(rec_sink), "max-size-time", split_dur * (long long int)1000000000 , NULL);
		// adapa install_probe_for_metadata_mux (rec_enc);
		install_probe_for_metadata_mux (rec_enc);

                if (write_framerate_override) {  // custom write framerate control, use videorate plugin
                    if (gst_element_link_many(rec_queue, rec_resize, rec_resize_cap, videorate, r_text_overlay, rec_enc, rec_capsfilter, rec_mux, rec_sink, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of record pipeline (write framerate)");
                        return(NULL);
                    }
		    if (gst_element_link_many(rec_metasrc, rec_mux, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of klv mux pipeline");
                        return(NULL);
                    }

                }
                else {  // no write framerate specified, good old nvvidconv
                    if (gst_element_link_many(rec_queue, rec_resize, rec_resize_cap, rec_enc, rec_capsfilter, rec_mux, rec_sink, NULL) != TRUE )
                    {
                        ROS_ERROR("Error linking elements of record pipeline (no write framerate)");
                        return(NULL);
                    }
		    if (gst_element_link_many(rec_metasrc, rec_mux, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of klv mux pipeline");
                        return(NULL);
                    }

                }
            }

            // STREAM if record_only is false
            if (!r_rec) {
                // TODO: copy to global for other use, consider making ori_w and ori_h member vars directoy
                rgb_enc_width = stream_width_override;
                rgb_enc_height = stream_height_override;

                /* request pads from queue */
                st_queue = element_create(ps_pipeline, "queue");
                ERR_CHECK(st_queue == NULL, NULL);
                queue_st_pad = gst_element_get_static_pad(st_queue, "sink");

                /* get pad from tee */
                // tee_st_pad = gst_element_get_request_pad(ps_tee, "vid_src");
                tee_st_pad = gst_element_get_request_pad(ps_tee, "src_%u");
                ERR_CHECK(tee_st_pad == NULL, NULL);

                /* Link I/O pads */
                if (gst_pad_link (tee_st_pad, queue_st_pad) != GST_PAD_LINK_OK)
                {
                    ROS_ERROR("tream Tee could not be linked exiting \n");
                    return NULL;
                }
                gst_object_unref(queue_st_pad);

                /* resize st_resize */
                bool detect_in_stream_pipe = ros::param::param<bool>("/drone/vision/video1/detect/detect_in_stream_pipe", false);
                if (detect_in_stream_pipe) {  // experimenetal detect in stream pipe
                    /* experimental detect in stream pipe */
                    s_nv1 = element_create(ps_pipeline, "nvvidconv");
                    ERR_CHECK(s_nv1 == NULL, NULL);

                    s_nv1_cap = element_create(ps_pipeline, "videoconvert");
                    ERR_CHECK(s_nv1_cap == NULL, NULL);

                    s_nv2 = element_create(ps_pipeline, "cvlaplace");
                    g_object_set(G_OBJECT(s_nv2),"width", stream_width_override, NULL);
                    g_object_set(G_OBJECT(s_nv2),"height", stream_height_override, NULL);
                    std::string home_dir = ros::param::param<std::string>("/drone/vision/video1/detect/home_dir", "/home/nsd_user");
                    std::string rgb_vp = ros::param::param<std::string>("/drone/vision/video1/detect/rgb_person_vehicle_model",
                        home_dir + "/code/catkin_birdseye/install/share/ngale_ros/models/yolov3_rgb_fp16.engine");
                    g_object_set(G_OBJECT(s_nv2),"model", rgb_vp.c_str(), NULL);
                    ERR_CHECK(s_nv2 == NULL, NULL);
     
                    s_nv2_cap = element_create(ps_pipeline, "videoconvert");
                    ERR_CHECK(s_nv2_cap == NULL, NULL);
                } else {
                    // non-experimental streaming w/o detection inline
                    /* resize */
                    if (stream_framerate_override) {  // custom stream framerate is secified, use videoscale? (why write pipe uses videorate?)
                        ROS_INFO("RGB stream framerate assigned (fps) [%d]", stream_framerate_override);
                        // if stream framerate is requested, use stream framerate (this needs testing)

                        st_resize = element_create(ps_pipeline, "nvvidconv");
                        ERR_CHECK(st_resize == NULL, NULL);

                        st_resize_cap = element_create(ps_pipeline, "capsfilter");
                        ERR_CHECK(st_resize_cap == NULL, NULL);
                        strcpy(ac_src_caps, "video/x-raw(memory:NVMM), ");
                        sprintf(ac_temp,"width=%d, ", stream_width_override);
                        strcat(ac_src_caps, ac_temp);
                        sprintf(ac_temp,"height=%d", stream_height_override);
                        strcat(ac_src_caps, ac_temp);
                        ps_caps = gst_caps_from_string(ac_src_caps);
                        g_object_set(G_OBJECT(st_resize_cap), "caps", ps_caps, NULL);
                        gst_caps_unref(ps_caps);

                        ROS_ERROR("TODO: rate control is actually not working, so this is ignoring stream_framerate_override");
                        /* TODO: rate control is actually not working, so this is ignoring stream_framerate_override */
                        // rate control seems to cause hangs after stream is running for a while
                        // s_videorate = element_create(ps_pipeline,"videorate");
                        // ERR_CHECK(s_videorate == NULL, NULL);
                        // g_object_set(G_OBJECT(s_videorate),"max-rate", stream_framerate_override, NULL);

                        /* videoscale plugin use, not needed if nvvicconv resize works */
                        // s_nv2 = element_create(ps_pipeline, "videoscale");
                        // ERR_CHECK(s_nv2 == NULL, NULL);
                        // s_nv2_cap = element_create(ps_pipeline, "capsfilter");
                        // ERR_CHECK(s_nv2_cap == NULL, NULL);
                        // strcpy(ac_src_caps, "video/x-raw,");
                        // sprintf(ac_temp,"width=%d,", stream_width_override);
                        // strcat(ac_src_caps,ac_temp);
                        // sprintf(ac_temp,"height=%d,", stream_height_override);
                        // strcat(ac_src_caps,ac_temp);
                        // sprintf(ac_temp,"format=%s", "I420");
                        // strcat(ac_src_caps,ac_temp);
                        // ps_caps = gst_caps_from_string(ac_src_caps);
                        // g_object_set(G_OBJECT(s_nv2_cap), "caps", ps_caps, NULL);
                        // gst_caps_unref(ps_caps);

                    } else {  // no custom stream framerate
                        ROS_INFO("stream framerate not assigned, using camera framerate (fps) only if h264 [%d]", camera_framerate_override);
                        // if stream framerate is not requested, use camera framerate in h264
                        st_resize = element_create(ps_pipeline, "nvvidconv");
                        ERR_CHECK(st_resize == NULL, NULL);

                        st_resize_cap = element_create(ps_pipeline, "capsfilter");
                        strcpy(ac_src_caps, "video/x-raw");
                        // sprintf(ac_temp,", width=%d", stream_width_override);
                        // strcat(ac_src_caps, ac_temp);
                        // sprintf(ac_temp,", height=%d", stream_height_override);
                        // strcat(ac_src_caps, ac_temp);
                        // sprintf(ac_temp,", format=%s", "I420");
                        // strcat(ac_src_caps, ac_temp);

                        // only for h264
                        if (!r_265)
                        {
                            sprintf(ac_temp, ",framerate=(fraction)%d/1", camera_framerate_override);
                            strcat(ac_src_caps, ac_temp);
                        }

                        ps_caps = gst_caps_from_string(ac_src_caps);
                        g_object_set(G_OBJECT(st_resize_cap), "caps", ps_caps, NULL);
                        gst_caps_unref(ps_caps);

                        /* videoscale plugin use, not needed if nvvicconv resize works */
                        s_nv2 = element_create(ps_pipeline, "videoscale");
                        ERR_CHECK(s_nv2 == NULL, NULL);
                        s_nv2_cap = element_create(ps_pipeline, "capsfilter");
                        ERR_CHECK(s_nv2_cap == NULL, NULL);
                        strcpy(ac_src_caps, "video/x-raw,");
                        sprintf(ac_temp,"width=%d,", stream_width_override);
                        strcat(ac_src_caps,ac_temp);
                        sprintf(ac_temp,"height=%d,", stream_height_override);
                        strcat(ac_src_caps,ac_temp);
                        sprintf(ac_temp,"format=%s", "I420");
                        strcat(ac_src_caps,ac_temp);
                        ps_caps = gst_caps_from_string(ac_src_caps);
                        g_object_set(G_OBJECT(s_nv2_cap), "caps", ps_caps, NULL);
                        gst_caps_unref(ps_caps);
                    }
                }

                if (detect_in_stream_pipe) {
                    nested_tee = element_create(ps_pipeline, "tee");
                    ERR_CHECK(nested_tee == NULL, NULL);

                    if (gst_element_link_many(st_queue, s_nv1, s_nv1_cap, s_nv2, s_nv2_cap, nested_tee, NULL) != TRUE)
                    {
                        ROS_ERROR("Error linking elements of stream1 pipeline");
                        return NULL ;
                    }
                } else {
                    overlay_queue = element_create(ps_pipeline, "queue");
                    ERR_CHECK(overlay_queue == NULL, NULL);
                    if (stream_framerate_override) {
                        if (gst_element_link_many(st_queue, st_resize, st_resize_cap, overlay_queue, NULL) != TRUE) {
                            ROS_ERROR("Error linking elements of stream1 pipeline with stream framerate override");
                            return NULL ;
                        }
                    } else {
                        if (gst_element_link_many(st_queue, st_resize, st_resize_cap, s_nv2, s_nv2_cap, overlay_queue, NULL) != TRUE)
                        {
                            ROS_ERROR("Error linking elements of stream1 pipeline without stream framerate override");
                            return NULL ;
                        }
                    }
                }

                if (detect_in_stream_pipe) {
                    // create nested queue for inline detection
                    nested_queue = element_create(ps_pipeline, "queue");
                    ERR_CHECK( nested_queue == NULL, NULL);

                    nested_tee_pad = gst_element_get_request_pad(nested_tee, "src_%u");
                    ERR_CHECK(nested_tee_pad == NULL, NULL);
                    nested_queue_pad = gst_element_get_static_pad(nested_queue, "sink");

                    /* Link I/O pads */
                    if ( gst_pad_link (nested_tee_pad, nested_queue_pad) != GST_PAD_LINK_OK)
                    {
                        ROS_ERROR("tee could not be linked exiting \n");
                        return NULL;
                    }
                    gst_object_unref(nested_queue_pad);
                }

                /* initialize text overlay */
                // if (stream_test_textoverlay && stream_framerate_override) \{  // text overlay
                int num_markers = 3;
                if (stream_test_textoverlay) {  // text overlay
                    ROS_INFO("RGB STREAM TEXTOVERLAY IS ON");
                    if (overlay_target) {
                        for (int i = 0; i < num_markers; ++i) {
                            s_text_overlay_c.push_back(element_create(ps_pipeline, "textoverlay"));
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "text", "☉", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "color", 0xffff0000, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "font-desc", "Sans, 20", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "auto-resize", true, NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "text-width", 30, NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "text-height", 30, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "valignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "halignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "x-absolute", overlay_x, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "y-absolute", overlay_y, NULL);
                            ERR_CHECK(s_text_overlay_c[i] == NULL, NULL);

                            s_text_overlay_p.push_back(element_create(ps_pipeline, "textoverlay"));
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "text", "+", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "font-desc", "Sans, 20", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "color", 0xffff0000, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "valignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "halignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "x-absolute", overlay_x, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "y-absolute", overlay_y, NULL);
                            ERR_CHECK(s_text_overlay_p[i] == NULL, NULL);
                        }
                    }

                    s_text_overlay2 = element_create(ps_pipeline, "textoverlay");
                    {
                        // sprintf(ac_temp, "%f", ros::Time::now().toSec());
                        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                        std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

                        std::string test1_str = "";  // "ok\nthis is a sentence\n";
                        std::string whole_str = test1_str + iso_time_str;

                        g_object_set(G_OBJECT(s_text_overlay2), "text", whole_str.c_str(), NULL);
                    }
                    ERR_CHECK(s_text_overlay2 == NULL, NULL);
                } else {
                    ROS_INFO("have to turn text overlay off because it does not work without stream_framerate_override");
                    for (int i = 0; i < num_markers; ++i) {
                        s_text_overlay_c.push_back(element_create(ps_pipeline, "identity"));
                        s_text_overlay_p.push_back(element_create(ps_pipeline, "identity"));
                    }
                    s_text_overlay2 = element_create(ps_pipeline, "identity");
                }

                /* Video encode */
                if (stream_enc_type == ENC_HW)
                {
                    ROS_INFO("rgb encode video in HW");
                    if (!r_265)
                        g_st_enc = element_create(ps_pipeline, "omxh264enc");
                    else
                        g_st_enc = element_create(ps_pipeline, "omxh265enc");

                    ERR_CHECK(g_st_enc == NULL, NULL);

                    g_object_set(G_OBJECT(g_st_enc), "control-rate", stream_rate_control, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "bitrate", int (stream_bitrate * .83333), NULL);
                    g_object_set(G_OBJECT(g_st_enc), "peak-bitrate", int (stream_bitrate), NULL);
                    g_object_set(G_OBJECT(g_st_enc), "preset-level", stream_preset_level, NULL);

                    g_object_set(G_OBJECT(g_st_enc), "qp-range", stream_qp_range.c_str(), NULL);
                    g_object_set(G_OBJECT(g_st_enc), "EnableTwopassCBR", stream_EnableTwopassCBR, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "vbv-size", stream_vbv_size, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "temporal-tradeoff", stream_temporal_tradeoff, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "iframeinterval", stream_iframeinterval, NULL);

                    if (!r_265)
                    {
                        g_object_set(G_OBJECT(g_st_enc), "profile", stream_profile, NULL);
                        g_object_set(G_OBJECT(g_st_enc), "num-B-Frames", stream_b_frames, NULL);
                        g_object_set(G_OBJECT(g_st_enc), "insert-sps-pps", stream_insert_sps_pps, NULL);
                    }
                    ROS_INFO("rgb stream qp range: [%s]", stream_qp_range.c_str());

                    st_capsfilter = element_create(ps_pipeline, "capsfilter");
                    ERR_CHECK(st_capsfilter == NULL, NULL);

                    if (!r_265)
                        strcpy(ac_src_caps, "video/x-h264,");
                    else
                        strcpy(ac_src_caps, "video/x-h265,");

                    sprintf(ac_temp, "level=(string)%s, ", stream_level.c_str());
                    strcat(ac_src_caps, ac_temp);
                    sprintf(ac_temp, "stream-format=%s", "byte-stream");
                    strcat(ac_src_caps, ac_temp);
                    // if (!r_265)
                    // {
                    //     sprintf(ac_temp, ",framerate=(fraction)%d/1", camera_framerate_override);
                    //     strcat(ac_src_caps, ac_temp);
                    // }

                    ps_caps = gst_caps_from_string(ac_src_caps);
                    g_object_set(G_OBJECT(st_capsfilter), "caps", ps_caps, NULL);
                    gst_caps_unref(ps_caps);

                } else { // not ENC_HW, so software encoding?
                    if (!r_265)
                        g_st_enc = element_create(ps_pipeline, "x264enc");
                    else
                        g_st_enc = element_create(ps_pipeline, "x265enc");

                    g_object_set(G_OBJECT(g_st_enc), "bitrate", stream_bitrate/1000, NULL);
                    ERR_CHECK(g_st_enc == NULL, NULL);

                    st_capsfilter = element_create(ps_pipeline, "capsfilter");
                    ERR_CHECK(st_capsfilter == NULL, NULL);

                }

                /* Payload & Sink */
                if (stream_muxer.empty()) {
                    if (!r_265)
                        st_mux = element_create(ps_pipeline, "rtph264pay");
                    else
                        st_mux = element_create(ps_pipeline, "rtph265pay");
                } else {
                    // only this works with wowzap
                    st_mux = element_create(ps_pipeline, stream_muxer.c_str());
                }

                g_object_set(G_OBJECT(st_mux), "alignment", stream_alignment, NULL);
                g_object_set(G_OBJECT(st_mux), "si-interval", stream_si_interval, NULL);
                // st_mux.si_interval = 1000;

                ERR_CHECK(st_mux == NULL, NULL);

                st_sink = element_create(ps_pipeline, "udpsink");
                ERR_CHECK(st_sink == NULL, NULL);

                ROS_INFO("rgb stream_target: %s:%d", stream_target_ip.c_str(), stream_target_port);
                g_object_set(G_OBJECT(st_sink), "host", stream_target_ip.c_str(), NULL);
                g_object_set(G_OBJECT(st_sink), "port", stream_target_port, NULL);
                g_object_set(G_OBJECT(st_sink), "max-bitrate", stream_bitrate, NULL);

                /*
                if (detect_in_stream_pipe) {
                    if (gst_element_link_many(nested_queue,
                                              s_text_overlay_c1, s_text_overlay_p1,
                                              s_text_overlay_c2, s_text_overlay_p2,
                                              s_text_overlay_c3, s_text_overlay_p3,
                                              s_text_overlay2, g_st_enc, st_capsfilter, st_mux, st_sink, NULL) != TRUE) {
                        ROS_ERROR("Error linking elements of stream1 pipeline");
                        return NULL ;
                    }
                } else {
                    if (gst_element_link_many(overlay_queue,
                                              s_text_overlay_c1, s_text_overlay_p1,
                                              s_text_overlay_c2, s_text_overlay_p2,
                                              s_text_overlay_c3, s_text_overlay_p3,
                                              s_text_overlay2, g_st_enc, st_capsfilter, st_mux, st_sink, NULL) != TRUE) {
                        ROS_ERROR("Error linking elements of stream1 pipeline");
                        return NULL ;
                    }
                }
                */
                if (detect_in_stream_pipe) {
                    if (gst_element_link(nested_queue       , s_text_overlay_c[0]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                } else {
                    if (gst_element_link(overlay_queue       , s_text_overlay_c[0]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                }
                if (gst_element_link(s_text_overlay_c[0], s_text_overlay_c[1]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay_c[1], s_text_overlay_c[2]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay_c[2], s_text_overlay_p[0]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay_p[0], s_text_overlay_p[1]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay_p[1], s_text_overlay_p[2]) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay_p[2], s_text_overlay2    ) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(s_text_overlay2    , g_st_enc           ) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(g_st_enc           , st_capsfilter      ) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(st_capsfilter      , st_mux             ) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
                if (gst_element_link(st_mux             , st_sink            ) != TRUE) { ROS_ERROR("Error linking elements"); return NULL ; }
            }

            if (r_det) {
                /* Detect */
                rgb_det_width = detect_width_override;
                rgb_det_height = detect_height_override;

                ROS_INFO("Enabling RGB detection");

                /* request pad from queue */
                lp_queue = element_create(ps_pipeline, "queue");
                ERR_CHECK(lp_queue == NULL, NULL);
                queue_lp_pad = gst_element_get_static_pad(lp_queue, "sink");

                /* get pad from tee */
                bool detect_in_stream_pipe = ros::param::param<bool>("/drone/vision/video1/detect/detect_in_stream_pipe", false);
                if (detect_in_stream_pipe) // experimenetal detect in stream pipe
                    tee_lp_pad = gst_element_get_request_pad (nested_tee, "src_%u");
                else
                    // tee_lp_pad = gst_element_get_request_pad (ps_tee, "img_src");
                    tee_lp_pad = gst_element_get_request_pad (ps_tee, "src_%u");
                ERR_CHECK(tee_lp_pad == NULL, NULL);

                /* Link I/O pads */
                ROS_INFO("RGB DETECT: building RGB I/O pads");
                if (gst_pad_link (tee_lp_pad, queue_lp_pad) != GST_PAD_LINK_OK)
                {
                    ROS_ERROR("RGB DETECT: tee could not be linked exiting \n");
                    return NULL;
                }
                gst_object_unref(queue_lp_pad);

                /* build lp_scale filer */
                if (stream_framerate_override && detect_in_stream_pipe)
                {
                    ROS_INFO("RGB DETECT: building RGB lp_scale with framerate and detect_in_stream_pipe");
                    lp_scale = element_create(ps_pipeline, "nvvidconv");
                    // lp_scale = element_create(ps_pipeline, "videoscale");
                    ERR_CHECK(lp_scale == NULL, NULL);
                }
                else
                {
                    ROS_INFO("RGB DETECT: building RGB lp_scale without framerate or detect");
                    lp_scale = element_create(ps_pipeline, "nvvidconv");
                    // lp_scale = element_create(ps_pipeline, "videoscale");
                    ERR_CHECK(lp_scale == NULL, NULL);
                }

                /* build lp_scale capsfilter */
                lp_capsfilter = element_create(ps_pipeline, "capsfilter");

                ERR_CHECK(lp_capsfilter == NULL,  NULL);
                strcpy(ac_src_caps, "video/x-raw(ANY)");
                // strcpy(ac_src_caps, "video/x-raw");
                sprintf(ac_temp, ", width=%d", detect_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", height=%d", detect_height_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", format=%s", detect_format_enc.c_str());  // setting this to BGRx and removing vidconv, detection fails
                strcat(ac_src_caps, ac_temp);
                // sprintf(ac_temp, ", framerate=(fraction)%d/1", detect_framerate);  // enabling this kills stream
                // strcat(ac_src_caps, ac_temp);
                ROS_INFO("RGB DETECT: Src caps = %s", ac_src_caps);

                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(lp_capsfilter), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

                /* copy out of NVMM */
                GstElement *vidconv, *vidconvcap;

                vidconv = element_create(ps_pipeline,"videoconvert");
                ERR_CHECK(vidconv == NULL, NULL);
                vidconvcap = element_create(ps_pipeline,"capsfilter");
                ERR_CHECK(vidconvcap == NULL, NULL);

                strcpy(ac_src_caps, "video/x-raw,");
                sprintf(ac_temp, "format=%s", detect_format_dec.c_str());
                strcat(ac_src_caps, ac_temp);
                // sprintf(ac_temp, ", framerate=(fraction)%d/1", detect_framerate);  // internal data flow error if enabled
                // strcat(ac_src_caps, ac_temp);
                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(vidconvcap), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

                /* setup callback */
                r_det_last_update = ros::Time::now();
                r_det_duration = ros::Duration(1.0/detect_framerate);
                //ROS_INFO("R_DET_DURATION");
                //ROS_INFO(r_det_duration); 
                ROS_INFO("RGB DETECT: building RGB callback for new_sample_rgb");
                GstAppSinkCallbacks callback = {NULL, NULL, new_sample_rgb};
                lp_sink = element_create(ps_pipeline, "appsink");
                ERR_CHECK(lp_sink == NULL, NULL);

                g_object_set (G_OBJECT(lp_sink), "async", TRUE, NULL);
                g_object_set (G_OBJECT(lp_sink), "sync", TRUE, NULL);
                g_object_set (G_OBJECT(lp_sink), "wait-on-eos", FALSE, NULL);
                //drop frames if delay is more that 16 frames
                g_object_set (G_OBJECT(lp_sink), "max-buffers", 16, NULL);
                g_object_set (G_OBJECT(lp_sink), "drop", TRUE, NULL);

                ROS_INFO("RGB DETECT: setting RGB callback for new_sample_rgb");
                gst_app_sink_set_callbacks (GST_APP_SINK(lp_sink), &callback, NULL, NULL);

                ROS_INFO("RGB DETECT: linking RGB detect elements");
                if (gst_element_link_many(lp_queue, lp_scale, lp_capsfilter, vidconv, vidconvcap, lp_sink, NULL) != TRUE)
                {
                    ROS_ERROR("RGB DETECT: error linking RGB Detect elements");
                    return NULL;
                }
            }

            /* test another tee to snapshot frames */
            if (snapshot_enabled) {
                /* snapshot */
                ROS_INFO("Enabling RGB frame snapshotter");

                ss_queue = element_create(ps_pipeline, "queue");
                ERR_CHECK(ss_queue == NULL, NULL);
                // tee_ss_pad = gst_element_get_request_pad (ps_tee, "vsnap_src");
                tee_ss_pad = gst_element_get_request_pad (ps_tee, "src_%u");
                ERR_CHECK(tee_ss_pad == NULL, NULL);
                queue_ss_pad = gst_element_get_static_pad(ss_queue,"sink");

                /* Link I/O pads */
                if ( gst_pad_link (tee_ss_pad, queue_ss_pad) != GST_PAD_LINK_OK)
                {
                    ROS_ERROR("ee could not be linked exiting \n");
                    return NULL;
                }
                gst_object_unref(queue_ss_pad);

                // if (r_enable)
                // {
                //     ss_scale = element_create(ps_pipeline, "videoscale");
                //     ERR_CHECK(ss_scale == NULL, NULL);
                // }
                // else
                // {
                    ss_scale = element_create(ps_pipeline, "nvvidconv");
                    ERR_CHECK(ss_scale == NULL, NULL);
                // }

                rgb_snapshot_width = snapshot_width_override;
                rgb_snapshot_height = snapshot_height_override;

                ss_capsfilter = element_create(ps_pipeline, "capsfilter");
                ERR_CHECK(ss_capsfilter == NULL,  NULL);
                strcpy(ac_src_caps, "video/x-raw, ");
                sprintf(ac_temp, "width=%d, ", snapshot_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, "height=%d, ", snapshot_height_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, "format=%s", snapshot_format_enc.c_str());
                strcat(ac_src_caps, ac_temp);
                ROS_DEBUG("Src caps = %s", ac_src_caps);
                ps_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(ss_capsfilter), "caps", ps_caps, NULL);
                gst_caps_unref(ps_caps);

                // // decode from I420 to RGB
                // GstElement *vidconv,*vidconvcap;
                // vidconv = element_create(ps_pipeline,"videoconvert");
                // ERR_CHECK(vidconv == NULL, NULL);
                // vidconvcap = element_create(ps_pipeline,"capsfilter");
                // ERR_CHECK(vidconvcap == NULL, NULL);
                // // maybe decoding here is expensive?
                // strcpy(ac_src_caps, "video/x-raw,");
                // sprintf(ac_temp, "format=%s", snapshot_format_dec.c_str());
                // strcat(ac_src_caps, ac_temp);
                // ps_caps = gst_caps_from_string(ac_src_caps);
                // g_object_set(G_OBJECT(vidconvcap), "caps", ps_caps, NULL);
                // gst_caps_unref(ps_caps);

                GstAppSinkCallbacks callback = {NULL, NULL, new_snapshot_rgb};
                ss_sink = element_create(ps_pipeline, "appsink");
                ERR_CHECK(ss_sink == NULL, NULL);

                g_object_set (G_OBJECT(ss_sink), "async", TRUE, NULL);
                g_object_set (G_OBJECT(ss_sink), "sync", TRUE, NULL);
                g_object_set (G_OBJECT(ss_sink), "wait-on-eos", FALSE, NULL);
                //drop frames if delay is more that 16 frames
                g_object_set (G_OBJECT(ss_sink), "max-buffers", 16, NULL);
                g_object_set (G_OBJECT(ss_sink), "drop", TRUE, NULL);

                gst_app_sink_set_callbacks (GST_APP_SINK(ss_sink), &callback, NULL, NULL);

                if (gst_element_link_many(ss_queue, ss_scale, ss_capsfilter, /*vidconv, vidconvcap,*/ ss_sink, NULL) != TRUE)
                {
                    ROS_ERROR("rror linking snapshot elements");
                    return NULL;
                }
            }
        }

        /* Set the pipeline to playing */
        int i_status = gst_element_set_state (ps_pipeline, GST_STATE_PLAYING);
        if (i_status == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("Error in setting pipeline state to playing");
            return(NULL);
        }
        return(ps_stream);
    }  // stream_start_rgb

    //Start thermal pipeline
    void* stream_start_thermal()
    {
        /*Declare and initialize Elements, Pads, Caps, bus etc*/
        stream_obj_t *ts_stream = NULL;
        GstBus * ts_bus = NULL;

        //camera and frame modification
        GstElement *ts_pipeline, *ts_src, *ts_src_cap, *flip, *vid_conv, *vid_conv_cap, *ts_rate, *ts_tee;
        GstElement *nv_vid_conv, *nv_vid_conv_cap;
        GstPad *tee_src_pad;

        //detection
        GstElement *queue_det, *det_sink, *det_scale, *det_scale_cap;  // , *det_vidconv, *det_vidconvcap;
        GstPad *queue_det_pad, *tee_src_det_pad;

        //store
        GstElement *queue_enc, *enc_convert, *enc_enc, *enc_enc_cap, *store_mux, *store_sink;
        GstElement *enc_convert_cap;
        GstPad *queue_enc_pad, *tee_src_enc_pad;

        //stream
        GstElement *st_convert, *st_convert_cap, *st_enc_cap,*stream_queue, *stream_pkg, *stream_sink;
        GstPad *queue_stream_pad, *tee_enc_stream_pad;

        GstCaps *ts_caps;
        char ac_temp[128], ac_src_caps[1024];

        ///////////////////////////////////////////////////////////////////////
        // read stuff from ros param
        ///////////////////////////////////////////////////////////////////////
        std::string vision_prefix = "/drone/vision/video2";

        // WRITE
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,4,38)
        // for 3.10 kernel, device shows up as /dev/video0
        std::string device_override = "/dev/video0";
#else
        // for 4.4 kernel, device shows up as /dev/video1
        std::string device_override = "/dev/video1";
#endif

        // Thermal stream settings
        // CAMERA
        int camera_width                        = ros::param::param<int>(vision_prefix + "/camera/width"               , 640);
        int camera_height                       = ros::param::param<int>(vision_prefix + "/camera/height"              , 512);
        std::string camera_format       = ros::param::param<std::string>(vision_prefix + "/camera/format"              , "BGR");
        std::string camera_format2      = ros::param::param<std::string>(vision_prefix + "/camera/format2"             , "I420");
        int camera_framerate                    = ros::param::param<int>(vision_prefix + "/camera/framerate"           , 30);

        // WRITE
        int write_framerate_override            = ros::param::param<int>(vision_prefix + "/write/framerate"            , 30);
        int write_width_override                = ros::param::param<int>(vision_prefix + "/write/width_override"       , 640);
        int write_height_override               = ros::param::param<int>(vision_prefix + "/write/height_override"      , 512);
        int write_bitrate                       = ros::param::param<int>(vision_prefix + "/write/bitrate_override"     , 1500000);
        std::string write_qp_range      = ros::param::param<std::string>(vision_prefix + "/write/qp_range"             , "-1,-1:-1,-1:-1,-1");
        std::string write_level         = ros::param::param<std::string>(vision_prefix + "/write/level"                , "high5");
        std::string write_muxer         = ros::param::param<std::string>(vision_prefix + "/write/muxer"                , "mpegtsmux");
        std::string write_format        = ros::param::param<std::string>(vision_prefix + "/write/format"               , "I420");

        // Why did MCW skip assigning these?
        // more options
        int write_rate_control                  = ros::param::param<int>(vision_prefix + "/write/rate_control"         , 1);
        int write_preset_level                  = ros::param::param<int>(vision_prefix + "/write/preset_level"         , 0);
        int write_EnableTwopassCBR              = ros::param::param<int>(vision_prefix + "/write/EnableTwopassCBR"     , 0);
        int write_vbv_size                      = ros::param::param<int>(vision_prefix + "/write/vbv_size"             , 10);
        int write_temporal_tradeoff             = ros::param::param<int>(vision_prefix + "/write/temporal_tradeoff"    , 0);
        int write_iframeinterval                = ros::param::param<int>(vision_prefix + "/write/iframeinterval"       , 0);
        int write_profile                       = ros::param::param<int>(vision_prefix + "/write/profile"              , 1);
        int write_b_frames                      = ros::param::param<int>(vision_prefix + "/write/b_frames"             , 0);
        int write_insert_sps_pps                = ros::param::param<int>(vision_prefix + "/write/insert_sps_pps"       , 1);

        write_test_textoverlay                 = ros::param::param<bool>(vision_prefix + "/write/overlay/enable"       , true);
        int write_enc_type                      = ros::param::param<int>(vision_prefix + "/write/enc_type"             , ENC_HW);

        // STREAM
        std::string stream_target_ip    = ros::param::param<std::string>(vision_prefix + "/stream_target_ip_override"  , "192.168.168.4");
        int stream_target_port                  = ros::param::param<int>(vision_prefix + "/stream_target_port_override", 14001);
        stream_target_ip                = ros::param::param<std::string>(vision_prefix + "/stream_target_ip"           , stream_target_ip);
        stream_target_port                      = ros::param::param<int>(vision_prefix + "/stream_target_port"         , stream_target_port);
        stream_target_ip                = ros::param::param<std::string>(vision_prefix + "/stream/target_ip"           , stream_target_ip);
        stream_target_port                      = ros::param::param<int>(vision_prefix + "/stream/target_port"         , stream_target_port);

        int stream_framerate_override           = ros::param::param<int>(vision_prefix + "/stream/framerate"           , 15);
        int stream_width_override               = ros::param::param<int>(vision_prefix + "/stream/width_override"      , 320);
        int stream_height_override              = ros::param::param<int>(vision_prefix + "/stream/height_override"     , 256);

        stream_test_textoverlay                = ros::param::param<bool>(vision_prefix + "/stream/overlay/enable"      , true);
        bool overlay_target                    = ros::param::param<bool>(vision_prefix + "/stream/overlay/target"      , true);
        double overlay_x                     = ros::param::param<double>(vision_prefix + "/stream/overlay/x"           , 0.5);
        double overlay_y                     = ros::param::param<double>(vision_prefix + "/stream/overlay/y"           , 0.5);

        int stream_enc_type                     = ros::param::param<int>(vision_prefix + "/stream/enc_type"            , ENC_HW);

        int stream_rate_control                 = ros::param::param<int>(vision_prefix + "/stream/rate_control"        , 1);
        int stream_bitrate                      = ros::param::param<int>(vision_prefix + "/stream/bitrate_override"    , 200000);
        int stream_preset_level                 = ros::param::param<int>(vision_prefix + "/stream/preset_level"        , 0);
        std::string stream_qp_range     = ros::param::param<std::string>(vision_prefix + "/stream/qp_range"            , "-1,-1:-1,-1:-1,-1");

        // more options
        bool stream_EnableTwopassCBR           = ros::param::param<bool>(vision_prefix + "/stream/EnableTwopassCBR"    , false);
        int stream_vbv_size                     = ros::param::param<int>(vision_prefix + "/stream/vbv_size"            , 10);
        int stream_temporal_tradeoff            = ros::param::param<int>(vision_prefix + "/stream/temporal_tradeoff"   , 0);
        int stream_iframeinterval               = ros::param::param<int>(vision_prefix + "/stream/iframe_interval"     , 0);

        bool stream_slice_intra_refresh        = ros::param::param<bool>(vision_prefix + "/stream/slice_intra_refresh" , true);
        int stream_slice_intra_refresh_interval = ros::param::param<int>(vision_prefix + "/stream/slice_intra_refresh_interval", 60);
        bool stream_bit_packetization          = ros::param::param<bool>(vision_prefix + "/stream/bit_packetization"   , true);

        std::string stream_muxer        = ros::param::param<std::string>(vision_prefix + "/stream/muxer"               , "mpegtsmux");
        int stream_alignment                    = ros::param::param<int>(vision_prefix + "/stream/alignment"           , 7);
        int stream_si_interval                  = ros::param::param<int>(vision_prefix + "/stream/si_interval"         , 1000);
        std::string stream_level        = ros::param::param<std::string>(vision_prefix + "/stream/level"               , "high5");

        // h264 only
        int stream_profile                      = ros::param::param<int>(vision_prefix + "/stream/profile"             , 2);
        int stream_b_frames                     = ros::param::param<int>(vision_prefix + "/stream/b_frames"            , 0);
        int stream_insert_sps_pps               = ros::param::param<int>(vision_prefix + "/stream/insert_sps_pps"      , 1);

        // DETECTION
        int detect_width_override               = ros::param::param<int>(vision_prefix + "/detect/width_override"      , 416);
        int detect_height_override              = ros::param::param<int>(vision_prefix + "/detect/height_override"     , 416);
        std::string detect_format_enc   = ros::param::param<std::string>(vision_prefix + "/detect/format_encoded"      , "I420");
        std::string detect_format_dec   = ros::param::param<std::string>(vision_prefix + "/detect/format_decoded"      , "BGR");
        double detect_framerate              = ros::param::param<double>(vision_prefix + "/detect/framerate"           , 5.0);

        // debug tool
        bool debug_file_input                  = ros::param::param<bool>(vision_prefix + "/debug/file_input"           , false);
        std::string video_fname         = ros::param::param<std::string>(vision_prefix + "/debug/fname"                , "/data/test_t.mp4");
        // end of ros param block




        if (!t_265)
            ROS_INFO("Encoder type is X264");
        else
            ROS_INFO("Encoder type is X265");





        // CAMERA SOURCE
        ts_stream = (stream_obj_t *)malloc(sizeof(stream_obj_t));
        if (ts_stream == NULL)
        {
            ROS_ERROR("Error in allocating memory for object thermal");
            return(NULL);
        }
        memset(ts_stream, 0,sizeof(stream_obj_t));

        /* Init Gstreamer components */
        gst_init(0, NULL);

        /* Create the pipeline */
        ts_pipeline = ts_stream->s_pipeline = gst_pipeline_new("pipeline");
        if (ts_pipeline == NULL) {ROS_ERROR("Error ts_pipeline is NULL"); return NULL;}

        /* Add a bus to the pipeline  */
        ts_bus = gst_pipeline_get_bus (GST_PIPELINE(ts_pipeline));
        gst_bus_set_sync_handler (ts_bus, cb_message, (gpointer) ts_stream, NULL);
        gst_object_unref(ts_bus);

        /* Create Camera element v4l2src and modify format and frames*/
        ts_src = element_create(ts_pipeline, "v4l2src");
        ROS_INFO("Thermal device override: %s", device_override.c_str());
        g_object_set(G_OBJECT(ts_src), "device", device_override.c_str(), NULL);
        if (ts_src == NULL) {ROS_ERROR("Error ts_src"); return NULL;}

        /* ts_src capsfilter ts_src_cap */
        ts_src_cap = element_create(ts_pipeline, "capsfilter");
        if (ts_src_cap == NULL) {ROS_ERROR("Error ts_src_cap"); return NULL;}

        strcpy(ac_src_caps, "video/x-raw,");
        sprintf(ac_temp, "width=%d,", camera_width);
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "height=%d,", camera_height);
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "format=%s,", camera_format.c_str());
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "framerate=(fraction)%d/1", camera_framerate);
        strcat(ac_src_caps, ac_temp);

        ts_caps = gst_caps_from_string(ac_src_caps);
        g_object_set(G_OBJECT(ts_src_cap), "caps", ts_caps, NULL);
        gst_caps_unref(ts_caps);

        /* videoflip (this breaks pipe unless we change src format too)
        flip = element_create(ts_pipeline, "videoflip");
        if (flip == NULL) {ROS_ERROR("Error flip"); return NULL;}
        g_object_set(G_OBJECT(flip), "method", 2 , NULL);
        */

        /* videoconvert vid_conv */
        vid_conv =element_create(ts_pipeline,"videoconvert");
        if (vid_conv == NULL) {ROS_ERROR("Error vid_conv"); return NULL;}

        /* videoconvert capsfilter capsfilter vid_conv_cap */
        vid_conv_cap = element_create (ts_pipeline,"capsfilter");
        if (vid_conv_cap == NULL) {ROS_ERROR("Error vid_conv_cap"); return NULL;}

        // strcpy(ac_src_caps, "video/x-raw, format=(string)I420, framerate=(fraction)60/1");
        strcpy(ac_src_caps, "video/x-raw(ANY),");
        sprintf(ac_temp, "width=%d,", camera_width);
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "height=%d,", camera_height);
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "format=%s,", camera_format2.c_str());
        strcat(ac_src_caps, ac_temp);
        sprintf(ac_temp, "framerate=(fraction)%d/1", camera_framerate);
        strcat(ac_src_caps, ac_temp);

        ts_caps = gst_caps_from_string(ac_src_caps);
        g_object_set(G_OBJECT(vid_conv_cap), "caps", ts_caps, NULL);
        gst_caps_unref(ts_caps);

        /* nvvidconv and do flip here */
        nv_vid_conv =element_create(ts_pipeline,"nvvidconv");
        if (nv_vid_conv == NULL) {ROS_ERROR("Error nv_vid_conv"); return NULL;}
        g_object_set(G_OBJECT(nv_vid_conv), "flip-method", 2, NULL);

        /* nvvidconv capsfilter capsfilter nv_vid_conv_cap */
        nv_vid_conv_cap = element_create (ts_pipeline,"capsfilter");
        if (nv_vid_conv_cap == NULL) {ROS_ERROR("Error nv_vid_conv_cap"); return NULL;}

        // strcpy(ac_src_caps, "video/x-raw, format=(string)I420, framerate=(fraction)60/1");
        strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");
        // sprintf(ac_temp, ", flip-method=%d", 2);
        // strcat(ac_src_caps, ac_temp);
        // sprintf(ac_temp, ", width=%d", camera_width);
        // strcat(ac_src_caps, ac_temp);
        // sprintf(ac_temp, ", height=%d", camera_height);
        // strcat(ac_src_caps, ac_temp);

        ts_caps = gst_caps_from_string(ac_src_caps);
        g_object_set(G_OBJECT(nv_vid_conv_cap), "caps", ts_caps, NULL);
        gst_caps_unref(ts_caps);

        /* tee element ts_tee */
        ts_tee = element_create(ts_pipeline, "tee");
        // ts_tee = element_create(ts_pipeline, "nvtee");
        if (ts_tee == NULL) {ROS_ERROR("Error ts_tee"); return NULL;}

        /* from ts_src to ts_tee */
        // if (gst_element_link_many(ts_src, ts_src_cap, flip, vid_conv, vid_conv_cap, nv_vid_conv, nv_vid_conv_cap, ts_tee, NULL ) != TRUE)
        if (gst_element_link_many(ts_src, ts_src_cap, vid_conv, vid_conv_cap, nv_vid_conv, nv_vid_conv_cap, ts_tee, NULL ) != TRUE)
        {
            ROS_ERROR("Error linking elements of pipeline thermal");
            return(NULL);
        }

        if (t_enable)
        {
            // RECORD
            ROS_ERROR("Thermal Recording, on by default if enabled");

            /* get pad from queue */
            queue_enc = element_create(ts_pipeline, "queue");
            if (queue_enc == NULL) {ROS_ERROR("Error queue_enc"); return NULL;}
            queue_enc_pad = gst_element_get_static_pad(queue_enc, "sink");

            /* get pad from tee */
            tee_src_enc_pad = gst_element_get_request_pad(ts_tee, "src_%u");
            ERR_CHECK(tee_src_enc_pad == NULL, NULL);

            /* connect tee to queue */
            if (gst_pad_link(tee_src_enc_pad, queue_enc_pad) != GST_PAD_LINK_OK)
            {
                ROS_ERROR("Tee could not be linked exiting thermal");
                return NULL;
            }
            gst_object_unref(queue_enc_pad);

            /* nvvidconv */
            enc_convert = element_create(ts_pipeline, "nvvidconv");
            if (enc_convert == NULL) {ROS_ERROR("Error enc_convert"); return NULL;}

            /* nvvidconv capsfilter capsfilter enc_convert_cap */
            enc_convert_cap = element_create (ts_pipeline,"capsfilter");
            if (enc_convert_cap == NULL) {ROS_ERROR("Error enc_convert_cap"); return NULL;}

            // strcpy(ac_src_caps, "video/x-raw, format=(string)I420, framerate=(fraction)60/1");
            strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");

            ts_caps = gst_caps_from_string(ac_src_caps);
            g_object_set(G_OBJECT(enc_convert_cap), "caps", ts_caps, NULL);
            gst_caps_unref(ts_caps);

            /* EXPERIMENTAL write test textoverlay */
            if (false && write_test_textoverlay)  // todo: text overlay for thermal write needs fixing. probably need videorate too.
            {
                r_text_overlay = element_create(ts_pipeline, "textoverlay");
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
                std::string test1_str = "";  // "ok\nthis is a sentence\n";
                std::string whole_str = test1_str + iso_time_str;
                g_object_set(G_OBJECT(r_text_overlay),"text", whole_str.c_str(), NULL);
            } else {
                write_test_textoverlay = false;
                r_text_overlay = element_create(ts_pipeline, "identity");
            }

            /* enc_enc omxh265enc */
            if (!t_265)
                enc_enc = element_create(ts_pipeline,"omxh264enc");
            else
                enc_enc = element_create(ts_pipeline,"omxh265enc");
            if (enc_enc == NULL) {ROS_ERROR("Error enc_enc"); return NULL;}

            g_object_set(G_OBJECT(enc_enc), "control-rate", write_rate_control, NULL);
            g_object_set(G_OBJECT(enc_enc), "bitrate", write_bitrate, NULL);
            g_object_set(G_OBJECT(enc_enc), "preset-level", write_preset_level, NULL);

            g_object_set(G_OBJECT(enc_enc), "qp-range", write_qp_range.c_str(), NULL);
            g_object_set(G_OBJECT(enc_enc), "EnableTwopassCBR", write_EnableTwopassCBR, NULL);
            g_object_set(G_OBJECT(enc_enc), "vbv-size", write_vbv_size, NULL);
            g_object_set(G_OBJECT(enc_enc), "temporal-tradeoff", write_temporal_tradeoff, NULL);
            g_object_set(G_OBJECT(enc_enc), "iframeinterval", write_iframeinterval, NULL);

            if (!t_265)
            {
                g_object_set(G_OBJECT(enc_enc), "profile", write_profile, NULL);
                g_object_set(G_OBJECT(enc_enc), "num-B-Frames", write_b_frames, NULL);
                g_object_set(G_OBJECT(enc_enc), "insert-sps-pps", write_insert_sps_pps, NULL);
            }

            /* enc_enc_cap omxh265enc capsfilter */
            enc_enc_cap = element_create (ts_pipeline,"capsfilter");
            if (enc_enc_cap == NULL) {ROS_ERROR("Error enc_enc_cap"); return NULL;}

            if (!t_265)
                strcpy(ac_src_caps, "video/x-h264,");
            else
                strcpy(ac_src_caps, "video/x-h265,");
            sprintf(ac_temp, "level=(string)%s, ", write_level.c_str());
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "stream-format=%s, ", "byte-stream");
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "width=%d, ", write_width_override);
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "height=%d, ", write_height_override);
            strcat(ac_src_caps, ac_temp);
            sprintf(ac_temp, "framerate=(fraction)%d/1", write_framerate_override);
            strcat(ac_src_caps, ac_temp);

            ts_caps = gst_caps_from_string(ac_src_caps);
            g_object_set(G_OBJECT(enc_enc_cap), "caps", ts_caps, NULL);
            gst_caps_unref(ts_caps);

            /* store_mux write_muxer */
            store_mux = element_create(ts_pipeline, write_muxer.c_str());
            if (store_mux == NULL) {ROS_ERROR("Error store_mux"); return NULL;}

            /* store_sink filesink */
            store_sink = element_create(ts_pipeline, "filesink");
            if (store_sink == NULL) {ROS_ERROR("Error store_sink"); return NULL;}

            ROS_INFO("Thermal Streaming fname: %s", this->fname.c_str());
            g_object_set(G_OBJECT(store_sink),"location", this->fname.c_str() ,NULL);

            if ( gst_element_link_many (queue_enc, enc_convert, enc_convert_cap, r_text_overlay, enc_enc, enc_enc_cap, store_mux, store_sink,  NULL) != TRUE)
            {
                ROS_INFO("Error linking encode elements thermal");
                ROS_ERROR("Error linking encode elements thermal");
                return (NULL);
            }
            ROS_ERROR("Thermal Recording Setup Done");

            //STREAM
            if (!t_rec) {  // if not record_only
                ROS_ERROR("Thermal Streaming");
                thr_enc_width = stream_width_override;
                thr_enc_height = stream_height_override;

                /* get pad from queue */
                stream_queue = element_create(ts_pipeline, "queue");
                if (stream_queue == NULL) {ROS_ERROR("Error stream_queue"); return NULL;}
                ERR_CHECK(stream_queue == NULL, NULL);
                queue_stream_pad = gst_element_get_static_pad(stream_queue, "sink");

                /* get pad from tee */
                tee_enc_stream_pad = gst_element_get_request_pad(ts_tee, "src_%u");
                if (tee_enc_stream_pad == NULL) {ROS_ERROR("Error tee_enc_stream_pad"); return NULL;}
                ERR_CHECK(tee_enc_stream_pad == NULL, NULL);

                /* connect queue to tee */
                if (gst_pad_link(tee_enc_stream_pad, queue_stream_pad) != GST_PAD_LINK_OK)
                {
                    ROS_ERROR("Tee could not be linked exiting thermal");
                    return NULL;
                }
                gst_object_unref(queue_stream_pad);

                /* video convert */
                st_convert = element_create(ts_pipeline, "nvvidconv");
                if (st_convert == NULL) {ROS_ERROR("Error st_convert"); return NULL;}

                st_convert_cap = element_create (ts_pipeline, "capsfilter");
                if (st_convert_cap == NULL) {ROS_ERROR("Error st_convert_cap is NULL"); return NULL;}
                if (stream_test_textoverlay)
                    strcpy(ac_src_caps, "video/x-raw(ANY)");
                else
                    strcpy(ac_src_caps, "video/x-raw(memory:NVMM)");  // (memory:NMMM) works only if textoverlay is off
                // sprintf(ac_temp, ", width=%d", stream_width_override);
                // strcat(ac_src_caps, ac_temp);
                // sprintf(ac_temp, ", height=%d", stream_height_override);
                // strcat(ac_src_caps, ac_temp);
                ts_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(st_convert_cap), "caps", ts_caps, NULL);
                gst_caps_unref(ts_caps);

                /* initialize text overlay */
                int num_markers = 3;
                if (stream_test_textoverlay)
                {
                    ROS_ERROR("THR STREAM TEXTOVERLAY IS ON");
                    if (overlay_target)
                    {
                        for (int i = 0; i < num_markers; ++i) {
                            s_text_overlay_c.push_back(element_create(ts_pipeline, "textoverlay"));
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "text", "☉", NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "color", 0xffff0000, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "font-desc", "Sans, 20", NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "auto-resize", true, NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "text-width", 30, NULL);
                            // g_object_set(G_OBJECT(s_text_overlay_c[i]), "text-height", 30, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "valignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "halignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "x-absolute", overlay_x, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_c[i]), "y-absolute", overlay_y, NULL);
                            ERR_CHECK(s_text_overlay_c[i] == NULL, NULL);

                            s_text_overlay_p.push_back(element_create(ts_pipeline, "textoverlay"));
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "text", "+", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "font-desc", "Sans, 20", NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "color", 0xffff0000, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "valignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "halignment", 5, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "x-absolute", overlay_x, NULL);
                            g_object_set(G_OBJECT(s_text_overlay_p[i]), "y-absolute", overlay_y, NULL);
                            ERR_CHECK(s_text_overlay_p[i] == NULL, NULL);
                        }
                    }

                    // sprintf(ac_temp, "%f", ros::Time::now().toSec());
                    s_text_overlay2 = element_create(ts_pipeline, "textoverlay");
                    {
                        boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                        std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

                        std::string test1_str = "";  // "ok\nthis is a sentence\n";
                        std::string whole_str = test1_str + iso_time_str;

                        g_object_set(G_OBJECT(s_text_overlay2),"text", whole_str.c_str(), NULL);
                    }
                    if (s_text_overlay2 == NULL) {ROS_ERROR("Error s_text_overlay2 is NULL"); return NULL;}
                }
                else
                {
                    for (int i = 0; i < num_markers; ++i) {
                        s_text_overlay_c.push_back(element_create(ts_pipeline, "identity"));
                        s_text_overlay_p.push_back(element_create(ts_pipeline, "identity"));
                    }
                    s_text_overlay2 = element_create(ts_pipeline, "identity");
                }

                /* encode */
                if (!t_265)
                    g_st_enc = element_create(ts_pipeline, "omxh264enc");
                else
                    g_st_enc = element_create(ts_pipeline, "omxh265enc");
                if (g_st_enc == NULL) {ROS_ERROR("Error g_st_enc is NULL"); return NULL;}

                g_object_set(G_OBJECT(g_st_enc), "control-rate", stream_rate_control, NULL);
                g_object_set(G_OBJECT(g_st_enc), "bitrate", int(stream_bitrate * 0.83333), NULL);
                g_object_set(G_OBJECT(g_st_enc), "peak-bitrate", stream_bitrate, NULL);

                // commented out items are from gst-inspect, but not yet implemented
                // g_object_set(G_OBJECT(g_st_enc), "quant-i-frames", 0, NULL);
                // g_object_set(G_OBJECT(g_st_enc), "quant-p-frames", 0, NULL);
                // g_object_set(G_OBJECT(g_st_enc), "quant-b-frames", 0, NULL);

                g_object_set(G_OBJECT(g_st_enc), "iframeinterval", stream_iframeinterval, NULL);

                g_object_set(G_OBJECT(g_st_enc), "SliceIntraRefreshEnable", stream_slice_intra_refresh, NULL);
                g_object_set(G_OBJECT(g_st_enc), "SliceIntraRefreshInterval", stream_slice_intra_refresh_interval, NULL);
                g_object_set(G_OBJECT(g_st_enc), "bit-packetization", stream_bit_packetization, NULL);

                g_object_set(G_OBJECT(g_st_enc), "vbv-size", stream_vbv_size, NULL);
                g_object_set(G_OBJECT(g_st_enc), "temporal-tradeoff", stream_temporal_tradeoff, NULL);

                // g_object_set(G_OBJECT(g_st_enc), "EnableMVBufferMeta", false, NULL);

                g_object_set(G_OBJECT(g_st_enc), "qp-range", stream_qp_range.c_str(), NULL);  // qp;qi;qb ranges

                // g_object_set(G_OBJECT(g_st_enc), "MeasureEncoderLatency", false, NULL);

                g_object_set(G_OBJECT(g_st_enc), "EnableTwopassCBR", stream_EnableTwopassCBR, NULL);
                g_object_set(G_OBJECT(g_st_enc), "preset-level", stream_preset_level, NULL);

                // g_object_set(G_OBJECT(g_st_enc), "EnableStringentBitrate", false, NULL);  // for new kernel
                // g_object_set(G_OBJECT(g_st_enc), "slice-header-spacing", 0, NULL);  // for new kernel
                // g_object_set(G_OBJECT(g_st_enc), "insert-aud", false, NULL);  // for new kernel
                // g_object_set(G_OBJECT(g_st_enc), "insert-vui", false, NULL);  // for new kernel


                if (!t_265)
                {
                    g_object_set(G_OBJECT(g_st_enc), "profile", stream_profile, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "num-B-Frames", stream_b_frames, NULL);
                    g_object_set(G_OBJECT(g_st_enc), "insert-sps-pps", stream_insert_sps_pps, NULL);
                }

                st_enc_cap = element_create (ts_pipeline,"capsfilter");
                if (st_enc_cap == NULL) {ROS_ERROR("Error st_enc_cap is NULL"); return NULL;}

                if (!t_265)
                    strcpy(ac_src_caps, "video/x-h264,");
                else
                    strcpy(ac_src_caps, "video/x-h265,");
                sprintf(ac_temp, "level=(string)%s", stream_level.c_str());
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", stream-format=%s", "byte-stream");
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", width=%d ", stream_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", height=%d", stream_height_override);
                strcat(ac_src_caps, ac_temp);

                ts_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(st_enc_cap), "caps", ts_caps, NULL);
                gst_caps_unref(ts_caps);

                if (!t_265)
                    stream_pkg = element_create(ts_pipeline, "rtph264pay");
                else
                    stream_pkg = element_create(ts_pipeline, "rtph265pay");

                // only this works with wowza
                stream_pkg = element_create(ts_pipeline, stream_muxer.c_str());
                g_object_set(G_OBJECT(stream_pkg), "alignment", stream_alignment, NULL);
                g_object_set(G_OBJECT(stream_pkg), "si-interval", stream_si_interval, NULL);

                if (stream_pkg == NULL) {ROS_ERROR("Error stream_pkg is NULL"); return NULL;}
                stream_sink = element_create(ts_pipeline, "udpsink");
                if (stream_sink == NULL) {ROS_ERROR("Error stream_sink is NULL"); return NULL;}

                ROS_INFO("thr stream_target: %s:%d", stream_target_ip.c_str(), stream_target_port);
                g_object_set(G_OBJECT(stream_sink),"host", stream_target_ip.c_str(),NULL);
                g_object_set(G_OBJECT(stream_sink),"port", stream_target_port,NULL);
                g_object_set(G_OBJECT(stream_sink), "max-bitrate", stream_bitrate, NULL);

                if ( gst_element_link_many (stream_queue, st_convert, st_convert_cap,
                                            s_text_overlay_c[0], s_text_overlay_p[0],
                                            s_text_overlay_c[1], s_text_overlay_p[1],
                                            s_text_overlay_c[2], s_text_overlay_p[2],
                                            s_text_overlay2, g_st_enc, st_enc_cap, stream_pkg, stream_sink, NULL) != TRUE)
                {
                    ROS_ERROR("Error linking stream elements thermal");
                    return (NULL);
                }
                ROS_ERROR("Thermal Streaming Setup Done");
            }  // !t_rec, STREAM ON

            /* example from v0.3 for reference
            //thermal detection enabled
            if (t_det)
            {
                ROS_INFO("Enabled Thermal Detection");

                queue_det = element_create(ts_pipeline, "queue");
                ERR_CHECK(queue_det == NULL, NULL);

                queue_det_pad = gst_element_get_static_pad(queue_det,"sink");

                tee_src_det_pad = gst_element_get_request_pad(ts_tee, "src_%u");

                if (gst_pad_link(tee_src_det_pad, queue_det_pad) != GST_PAD_LINK_OK)
                {
                   ROS_ERROR(could not be linked exiting \n");
                   return NULL;
                }
                gst_object_unref(queue_det_pad);

                det_scale = element_create(ts_pipeline, "videoconvert");
                ERR_CHECK(det_scale == NULL, NULL);

                det_vidconvcap = element_create(ts_pipeline,"capsfilter");
                ERR_CHECK(det_vidconvcap == NULL, NULL);

                strcpy(ac_src_caps, "video/x-raw,");
                sprintf(ac_temp, "format=%s", "BGR");
                strcat(ac_src_caps, ac_temp);
                ts_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(det_vidconvcap), "caps", ts_caps, NULL);
                gst_caps_unref(ts_caps);

                GstAppSinkCallbacks callback ={NULL, NULL, new_sample_thermal};

                det_sink = element_create(ts_pipeline, "appsink");
                ERR_CHECK(det_sink == NULL, NULL);

                g_object_set (G_OBJECT(det_sink), "async", TRUE, NULL);
                g_object_set (G_OBJECT(det_sink), "sync", TRUE, NULL);
                gst_app_sink_set_callbacks (GST_APP_SINK(det_sink), &callback, NULL, NULL);

                if ( gst_element_link_many(queue_det, det_scale, det_vidconvcap, det_sink, NULL) != TRUE)
                {
                  ROS_ERROR("Error linking elements of stream1 pipeline thermal");
                  return NULL ;
                }
            } */

            //thermal detection enabled
            if (t_det) {
                ROS_ERROR("Thermal Detection");
                thr_det_width = detect_width_override;
                thr_det_height = detect_height_override;

                ROS_INFO("Enabled Thermal Detection");

                /* create queue and get queue pad */
                queue_det = element_create(ts_pipeline, "queue");
                if (queue_det == NULL) {ROS_ERROR("Error queue_det is NULL"); return NULL;}
                ERR_CHECK(queue_det == NULL, NULL);
                queue_det_pad = gst_element_get_static_pad(queue_det, "sink");

                /* get pad from tee */
                tee_src_det_pad = gst_element_get_request_pad(ts_tee, "src_%u");
                ERR_CHECK(tee_src_det_pad == NULL, NULL);

                /* connect queue to tee */
                if (gst_pad_link(tee_src_det_pad, queue_det_pad) != GST_PAD_LINK_OK)
                {
                    ROS_INFO(" Error pad_link_Not_ok");
                    return NULL;
                }
                gst_object_unref(queue_det_pad);

                /* build det_scale */
                det_scale = element_create(ts_pipeline, "nvvidconv");
                if (det_scale == NULL) {ROS_ERROR("Error det_scale is NULL"); return NULL;}

                /* build det_scale capsfilter */
                det_scale_cap = element_create(ts_pipeline, "capsfilter");
                if (det_scale_cap == NULL) {ROS_ERROR("Error det_scale_cap is NULL"); return NULL;}

                strcpy(ac_src_caps, "video/x-raw(ANY)");

                sprintf(ac_temp, ", width=%d", detect_width_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", height=%d", detect_height_override);
                strcat(ac_src_caps, ac_temp);
                sprintf(ac_temp, ", format=%s", detect_format_enc.c_str());
                strcat(ac_src_caps, ac_temp);
                // sprintf(ac_temp, "framerate=(fraction)%d/1", detect_framerate);
                // strcat(ac_src_caps, ac_temp);

                ts_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(det_scale_cap), "caps", ts_caps, NULL);
                gst_caps_unref(ts_caps);



                /* copy out of NVMM */
                ROS_ERROR("Thermal DETECT: building Thermal vidconv");
                GstElement *vidconv, *vidconvcap;

                vidconv = element_create(ts_pipeline,"videoconvert");
                ERR_CHECK(vidconv == NULL, NULL);
                vidconvcap = element_create(ts_pipeline,"capsfilter");
                ERR_CHECK(vidconvcap == NULL, NULL);

                ROS_ERROR("Thermal DETECT: building Thermal vidconvcap");
                strcpy(ac_src_caps, "video/x-raw,");
                sprintf(ac_temp, "format=%s", detect_format_dec.c_str());
                strcat(ac_src_caps, ac_temp);
                ts_caps = gst_caps_from_string(ac_src_caps);
                g_object_set(G_OBJECT(vidconvcap), "caps", ts_caps, NULL);
                gst_caps_unref(ts_caps);



                /* setup callback */
                t_det_last_update = ros::Time::now();
                t_det_duration = ros::Duration(1.0/detect_framerate);
                GstAppSinkCallbacks callback = {NULL, NULL, new_sample_thermal};

                det_sink = element_create(ts_pipeline, "appsink");
                if (det_sink == NULL) {ROS_ERROR("Error det_sink is NULL"); return NULL;}

                g_object_set (G_OBJECT(det_sink), "async", TRUE, NULL);
                g_object_set (G_OBJECT(det_sink), "sync", TRUE, NULL);
                g_object_set (G_OBJECT(det_sink), "wait-on-eos", FALSE, NULL);
                //drop frames if delay is more that 16 frames
                g_object_set (G_OBJECT(det_sink), "max-buffers", 16, NULL);
                g_object_set (G_OBJECT(det_sink), "drop", TRUE, NULL);

                gst_app_sink_set_callbacks (GST_APP_SINK(det_sink), &callback, NULL, NULL);

                if ( gst_element_link_many(queue_det, det_scale, det_scale_cap, vidconv, vidconvcap, det_sink, NULL) != TRUE)
                {
                    ROS_ERROR("Error linking elements of stream1 pipeline thermal");
                    return NULL ;
                }
                ROS_ERROR("Thermal Detection Setup Done");
            }

            ROS_ERROR("Thermal Pipeline Final Check");
            /* Set the pipeline to playing  */
            int i_status = gst_element_set_state (ts_pipeline, GST_STATE_PLAYING);
            if (i_status == GST_STATE_CHANGE_FAILURE)
            {
                ROS_ERROR("Error in setting pipeline state to playing thermal");
                return(NULL);
            }

            ROS_INFO("stream_start_thermal success thermal");
            return(ts_stream);
        }
        else {  // t_enable
            return(NULL);
        }
    }

    void stream_stop(void *pv_stream)
    {

        if (modelid == 0)
            ROS_INFO("stream_stop: RGB pipeline cleaned and closed");
        else
            ROS_INFO("stream_stop: THERMAL pipeline cleaned and closed");

        stream_obj_t *s_stream = (stream_obj_t *)pv_stream;

        /* Send an EOS */
        if (s_stream->i_stopped == 0)
        {
            gst_element_send_event (s_stream->s_pipeline, gst_event_new_eos ());
            /* Wait for EOS to be caught */
            while(s_stream->i_stopped == 0)
            {
                ROS_INFO("stream_stop: waiting for EOS to be caught");
                usleep(1000000);
            }
        }

        int i_status = gst_element_set_state (s_stream->s_pipeline, GST_STATE_NULL);
        if (i_status == GST_STATE_CHANGE_FAILURE)
        {
            ROS_ERROR("stream_stop: Error in setting pipeline state to NULL");
        }
        gst_object_unref(s_stream->s_pipeline);
        if (s_stream)
            free(s_stream);
    }

} rgb,thermal;
//End of class


class VideoInfo {

public:

    // Properties from the YAML
    float stream_bitrate_;
    float gain_;
};

std::mutex rgb_info_mutex_;
VideoInfo rgb_info_;
void rgb_info_callback(const std_msgs::Float64MultiArray::ConstPtr _msg) {

   // printf("madhu rgb_info_callback: %l\n", ros::time::now());
    // Set the variables
    std::lock_guard<std::mutex> lock(rgb_info_mutex_);
    ros::Time now_is = ros::Time::now();  // time we received this status
    if (_msg->data[0] > 0) {
        try {
            if (rgb.g_st_enc != NULL) {
                rgb_info_.stream_bitrate_ = _msg->data[0];
               // printf(rgb_info_.stream_bitrate_);
                g_object_set(G_OBJECT(rgb.g_st_enc), "bitrate", int (rgb_info_.stream_bitrate_ * .83333), NULL);
                ROS_INFO("RGB VIDEO CONTROL: setting rgb bitrate to [%d]", int (rgb_info_.stream_bitrate_ * .83333));
            }
            else {
                ROS_ERROR("RGB VIDEO CONTROL: rgb.g_st_enc is NULL");
            }
        }
        catch (...) { ROS_ERROR("RGB VIDEO CONTROL: exception setting rgb bitrate"); }
    }
    if (_msg->data[1] > 0) {
        rgb_info_.gain_ = _msg->data[1];
       // printf(rgb_info_.gain_);
        char cmd[1024];
        sprintf(cmd, "v4l2-ctl -c gain=%d", (int)rgb_info_.gain_);
        int result = system(cmd);
        ROS_ERROR("RGB VIDEO CONTROL: setting rgb gain to [%d] result [%d]", (int)rgb_info_.gain_, result);
    }
}

std::mutex thermal_info_mutex_;
VideoInfo thermal_info_;
void thermal_info_callback(const std_msgs::Float64MultiArray::ConstPtr _msg) {

    // Set the variables
    std::lock_guard<std::mutex> lock(thermal_info_mutex_);
    ros::Time now_is = ros::Time::now();  // time we received this status
    if (_msg->data[0] > 0) {
        try {
            if (thermal.g_st_enc != NULL) {
                thermal_info_.stream_bitrate_ = _msg->data[0];
                g_object_set(G_OBJECT(thermal.g_st_enc), "bitrate", int (thermal_info_.stream_bitrate_ * .83333), NULL);
                ROS_ERROR("THERMAL VIDEO CONTROL: setting thermal bitrate to [%d]", int (thermal_info_.stream_bitrate_ * .83333));
            }
            else {
                ROS_ERROR("THERMAL VIDEO CONTROL: thermal.g_st_enc is NULL");
            }
        }
        catch (...) { ROS_ERROR("THERMAL VIDEO CONTROL: exception setting thermal bitrate"); }
    }
     if (_msg->data[1] > 0) {
    //     thermal_info_.gain_ = _msg->data[1];
    //     char cmd[1024];
    //     sprintf(cmd, "v4l2-ctl -c gain=%d", (int)thermal_info_.gain_);
    //     int result = system(cmd);
    //     ROS_ERROR("THERMAL VIDEO CONTROL: setting thermal bitrate to [%d] result [%d]", (int)thermal_info_.gain_, result);
     }
}




void* call_det_rgb(void * temp)
{
    
   // printf("***********************************************************************IGNORE_THRESH %d",Yolo::LOCATIONS);

    ros::NodeHandle rgb_node;
    ros::Publisher detection_raw_pub = rgb_node.advertise<birdseye_msgs::DetectionResult>("/detection_raw/rgb", 2);
    rgb.frame_id = 0;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    double rgb_det_hz = ros::param::param<double>("/drone/vision/video1/detect/update_rate", 10.0);  // as fast as possible
    ros::Rate rate(rgb_det_hz);
    ROS_DEBUG("starting detection worker thread.");
    cv::Mat rframe_rgb_local;
    //auto t1, t1stop;
    float total;
    while(ros::ok())
    {
        //auto t1 = std::chrono::high_resolution_clock::now();
        if (cnt_rgb == DET_READY)  // ready for processing
        {
            //ros::Time clone_time;
            ROS_DEBUG("*RGB1: detection start");
            pthread_mutex_lock(&rframe_rgb_mut);
            rframe_rgb_local = rframe_rgb.clone();
            //ros::Time prediction_start;
            //double time_start = prediction_start.toSec();  
           // printf(rframe_rgb_local);
            //clone_time = ros::Time::now();
            //printf("clone frame: clone time is %f", clone_time.toSec());
            rframe_rgb.release();
            pthread_mutex_unlock(&rframe_rgb_mut);

            ///////////////////////////////////////////////////////////////////////////////////////
            //                                                                                   //
            //  rgb detection                                                                    //
            //                                                                                   //
            ///////////////////////////////////////////////////////////////////////////////////////
            
            ROS_INFO("RGB DETECTION PIPE: (2) DETECT FRAME");
            // ros::Time prediction_start;
            //ros::Time prediction_end;
            // double time_start = prediction_start.toSec(); 
           
            //ros::Time detect_time;
            rgb.detect_frame(rframe_rgb_local, rgb.net);
            //detect_time = ros::Time::now();
            //printf("detectframe: detect time is %f", detect_time.toSec());

            //double time_end = prediction_end.toSec();
            //printf("++++++++++++++++++++++++++time_start: %f",time_start.toSec());
	        //printf("++++++++++++++++++++++++++time_end: %f", time_end);
            //double time_taken_for_prediction = (time_start - time_end); 
            //printf("++++++++++++++++++++++++++time_taken_for_prediction: %f",time_taken_for_prediction);
            ROS_DEBUG("*RGB1: detection done");
            rframe_rgb_local.release();

            ROS_DEBUG("*RGB1: rframe_rgb release");

            ///////////////////////////////////////////////////////////////////////////////////////
            //                                                                                   //
            //  publish rgb detections                                                           //
            //                                                                                   //
            ///////////////////////////////////////////////////////////////////////////////////////
            {
                std::lock_guard<std::mutex> lock(rgb.detection_raw_syn_mutex);
                ROS_INFO("RGB DETECTION PIPE: (4) PUBLISH DETECTION");
                detection_raw_pub.publish(rgb.detection_raw_syn);
            }

            cnt_rgb = VIS_READY;
            // ROS_DEBUG("*RGB1: publish done");
        }
        //auto t1stop = std::chrono::high_resolution_clock::now();
 	//total = std::chrono::duration<float, milli>(t1stop - t1).count();
	//std::cout<<"\n\n["<<std::this_thread::get_id()<<"] Time taken :"<<total<< "ms\n\n";
        // ros::spinOnce();
        // ROS_DEBUG("*RGB1: spinOnce done");

        // if (gi_stop == 0)
        //     pthread_cond_wait(&rgb_cond, &rframe_rgb_mut);

        if (gi_stop != RUNNING)
        {
            cnt_rgb = CAP_IDLE;
            pthread_exit(NULL);
        }

        rate.sleep();       //<sagarwal> | this together with ros::Rate rate(rgb det hz) will cause the loop to run rgb det hz times in a second
        // ROS_DEBUG("*RGB1: sleep done");

        // uncomment below if we wnat to change rate dynamically
        // double new_hz = ros::param::param<double>("/drone/vision/video1/detect/update_rate", 2.0);
        // if (new_hz != rgb_det_hz) {
        //     rgb_det_hz = new_hz;
        //     rate = ros::Rate(rgb_det_hz);
        // }
        // ROS_DEBUG("*RGB1: rosparam done");
    }

    // sid check if needed
    cnt_rgb = CAP_IDLE;
    // pthread_mutex_unlock(&rframe_rgb_mut);
    pthread_exit(NULL);
}

void* call_det_thr(void * temp)
{
   // printf("call_det_thr: %f\n", ros::Time::now()); 
    ros::NodeHandle thermal_node;
    ros::Publisher detection_raw_pub = thermal_node.advertise<birdseye_msgs::DetectionResult>("/detection_raw/thermal", 2);
    thermal.frame_id = 0;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }


    double thr_det_hz = ros::param::param<double>("/drone/vision/video2/detect/update_rate", 2.0);  // as fast as possible
    ros::Rate rate(thr_det_hz);
    ROS_INFO("starting detection worker thread.");
    cv::Mat rframe_thr_local;
    while(ros::ok())
    {
        if (cnt_thr == DET_READY)
        {

            ROS_DEBUG("THR1: detection start");
            pthread_mutex_lock(&rframe_thr_mut);
            rframe_thr_local = rframe_thr.clone();
            rframe_thr.release();
            pthread_mutex_unlock(&rframe_thr_mut);

            ///////////////////////////////////////////////////////////////////////////////////////
            //                                                                                   //
            //  thermal detection                                                                //
            //                                                                                   //
            ///////////////////////////////////////////////////////////////////////////////////////
            thermal.detect_frame(rframe_thr_local, thermal.net);

            rframe_thr_local.release();

            ROS_DEBUG("THR1: detection end");

            {
                std::lock_guard<std::mutex> lock(thermal.detection_raw_syn_mutex);
                detection_raw_pub.publish(thermal.detection_raw_syn);
            }

            cnt_thr = VIS_READY;
        }
        // ros::spinOnce();

        // if (gi_stop == 0)
        //     pthread_cond_wait(&thr_cond, &rframe_thr_mut);

        if (gi_stop != 0)
        {
            cnt_thr = CAP_IDLE;
            pthread_exit(NULL);
        }
        rate.sleep();

        // uncomment below if we want to dynamically change update rate
        // double new_hz = ros::param::param<double>("/drone/vision/video2/detect/update_rate", 2.0);
        // if (new_hz != thr_det_hz) {
        //     thr_det_hz = new_hz;
        //     rate = ros::Rate(thr_det_hz);
        // }
    }

    //sid check if needed
    cnt_thr = CAP_IDLE;
    // pthread_mutex_unlock(&rframe_thr_mut);
    pthread_exit(NULL);
}


GstFlowReturn new_sample_rgb(GstAppSink *elt, gpointer ptr)
{
    // skip throttle by framerate

    //thread creation
    if (cnt_rgb == CAP_INIT)
    {
        pthread_create(&det_thread_rgb, NULL, call_det_rgb, NULL );
        ROS_INFO("RGB2: RGB Detection thread created");
        cnt_rgb = CAP_READY;
    }

    // ROS_DEBUG("RGB2: frame available");
    if ((ros::Time::now() - r_det_last_update) > r_det_duration)
    {
        r_det_last_update = ros::Time::now();
        
       // printf("madhur_det_last_update:%f", r_det_last_update);
        //frame data extraction
        // ROS_DEBUG("RGB2: new_sample_rgb");
        sample_gst_rgb = gst_app_sink_pull_sample (GST_APP_SINK (elt));
        //get the sample from appsink
        buffer_gst_rgb = gst_sample_get_buffer (sample_gst_rgb);

        if (buffer_gst_rgb)
        {
            try {
                // ROS_DEBUG("new_sample_rgb cv size(%d, %d)", rgb_det_width, rgb_det_height);
                gst_buffer_map (buffer_gst_rgb, &map_rgb, GST_MAP_READ);
                // ROS_DEBUG("done gst_buffer_map");
                cv::Mat frame(cv::Size(rgb_det_width, rgb_det_height), CV_8UC3, (char*)map_rgb.data, cv::Mat::AUTO_STEP);
                // ROS_DEBUG("done frame");
                /*
                cv::Mat rframe;
                // CPU resize, can be optimized
                cv::resize(frame, rframe, cv::Size(rgb_det_width, rgb_det_height));
                ROS_DEBUG("done resize");
                rframe_rgb = rframe.clone();
                */

                if (cnt_rgb == CAP_READY)
                {
                    if (pthread_mutex_trylock(&rframe_rgb_mut) == 0)
                    {
                        //ros::Time clone_time;            
                        ROS_INFO("RGB DETECTION PIPE: (1) CLONE FRAME");
                        rframe_rgb = frame.clone();
                        //clone_time = ros::Time::now();       
                        //ROS_INFO("cloneframe: clone time is %f", clone_time.toSec());
                       // printf("madhurdetduration:%f", r_det_duration);
                       // printf("madhurframe_rgb", rframe_rgb); 
                        // pthread_cond_signal(&rgb_cond);
                        ROS_DEBUG("RGB2: unlock detection thread");
                        pthread_mutex_unlock(&rframe_rgb_mut);
                        cnt_rgb = DET_READY;  // signal processing thread to start
                    }
                    else ROS_DEBUG("RGB2: MAT locked, skip clone frame");
                }
                else ROS_DEBUG("RGB2: detection active, skip clone frame");

                // ROS_DEBUG("done clone");
                gst_buffer_unmap(buffer_gst_rgb, &map_rgb);
                // ROS_DEBUG("done unmap");
                frame.release();
                // rframe.release();
                // ROS_DEBUG("done release");
            }
            catch (...) { ROS_ERROR("exception new_sample_rgb cv"); }
        }
        else {
            // skip processing frame
        }

        //sid check gi_stop
        if (sample_gst_rgb) gst_sample_unref(sample_gst_rgb);
    }

    return GST_FLOW_OK;
}




// exif
static ExifEntry *create_tag(ExifData *exif, ExifIfd ifd, ExifTag tag, size_t len)
{
    void *buf;
    ExifEntry *entry;

    /* Create a memory allocator to manage this ExifEntry */
    ExifMem *mem = exif_mem_new_default();
    assert(mem != NULL); /* catch an out of memory condition */

    /* Create a new ExifEntry using our allocator */
    entry = exif_entry_new_mem (mem);
    assert(entry != NULL);

    /* Allocate memory to use for holding the tag data */
    buf = exif_mem_alloc(mem, len);
    assert(buf != NULL);

    /* Fill in the entry */
    entry->data = (unsigned char*)buf;
    entry->size = len;
    entry->tag = tag;
    entry->components = len;
    entry->format = EXIF_FORMAT_UNDEFINED;

    /* Attach the ExifEntry to an IFD */
    exif_content_add_entry (exif->ifd[ifd], entry);

    /* The ExifMem and ExifEntry are now owned elsewhere */
    exif_mem_unref(mem);
    exif_entry_unref(entry);

    return entry;
}

void add_exif_to_image(const char *mInputFilename, double dlat, double dlon, double dalt)
{
    /* GPS Data */
    // double dlat = 37.667788; // we supposed that your GPS data is Double if its not skip this step
    // double dlon = -122.667788; // we supposed that your GPS data is Double if its not skip this step
    // double dalt = 5.4321;


    ExifEntry *entry;

    //Input JPG
    // char mInputFilename[]="/data/exifyay/build/example.jpg";
    printf("file: %s\n", mInputFilename);

    //Load JPG
    JPEGData * mJpegData = jpeg_data_new_from_file(mInputFilename);
    printf("mJpegData %lu\n", (long)mJpegData);

    //Load Exif data from JPG
    ExifData * mExifData = jpeg_data_get_exif_data(mJpegData);
    printf("mExifData %lu\n", (long)mExifData);

    if (mExifData == NULL)
    {
        mExifData = exif_data_new();
    }


    // python command for example
    // "exiftool -GPSLatitude=\"%s\" -GPSLongitude=\"%s\" -GPSLongitudeRef=\"%s\" -GPSLatitudeRef=\"%s\" -GPSAltitudeRef=\'Above Sea level\' -GPSAltitude=\"%s\" %s*"

    //Set some Exif options
    exif_data_set_option(mExifData, EXIF_DATA_OPTION_FOLLOW_SPECIFICATION);
    exif_data_set_data_type(mExifData, EXIF_DATA_TYPE_COMPRESSED);
    exif_data_set_byte_order(mExifData, FILE_BYTE_ORDER);

    /* example text */
    entry = create_tag(mExifData, EXIF_IFD_EXIF, EXIF_TAG_USER_COMMENT,
            sizeof(ASCII_COMMENT) + sizeof(FILE_COMMENT) - 2);
    /* Write the special header needed for a comment tag */
    memcpy(entry->data, ASCII_COMMENT, sizeof(ASCII_COMMENT)-1);
    /* Write the actual comment text, without the trailing NUL character */
    memcpy(entry->data+8, FILE_COMMENT, sizeof(FILE_COMMENT)-1);
    /* create_tag() happens to set the format and components correctly for
     * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */

    /* Create a EXIF_TAG_SUBJECT_AREA tag */
    entry = create_tag(mExifData, EXIF_IFD_EXIF, EXIF_TAG_SUBJECT_AREA,
               4 * exif_format_get_size(EXIF_FORMAT_SHORT));
    entry->format = EXIF_FORMAT_SHORT;
    entry->components = 4;

    /* GPS Lat */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LATITUDE, (size_t)24);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 3;

        // convert double to unsigned long array
        double coord = fabs(dlat);
        double sec = (coord * 3600);
        int deg = sec / 3600;
        sec = sec - 3600.0*((int)sec/3600);
        int min = sec / 60;
        sec = sec - 60.0*((int)sec/60);
        // printf("debug sec %f\n", sec);

        ExifRational secr;
        secr.numerator = (unsigned)(sec * 1000000.0);
        secr.denominator = (unsigned)1000000 ;

        ExifRational degr;
        degr.numerator = (unsigned)(deg * 1000000.0);
        degr.denominator = (unsigned)1000000 ;

        ExifRational minr;
        minr.numerator = (unsigned)(min * 1000000.0);
        minr.denominator = (unsigned)1000000 ;

        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, degr);
        exif_set_rational(entry->data+sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, minr);
        exif_set_rational(entry->data+2*sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, secr);
    }

    /* GPS Lon */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LONGITUDE, 24);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 3;
        // convert double to unsigned long array
        double coord = fabs(dlon);
        double sec = (coord * 3600);
        int deg = sec / 3600;
        sec = sec - 3600.0*((int)sec/3600);
        int min = sec / 60;
        sec = sec - 60.0*((int)sec/60);
        // printf("debug sec %f\n", sec);

        ExifRational secr;
        secr.numerator = (unsigned)(sec * 1000000.0);
        secr.denominator = (unsigned)1000000 ;

        ExifRational degr;
        degr.numerator = (unsigned)(deg * 1000000.0);
        degr.denominator = (unsigned)1000000 ;

        ExifRational minr;
        minr.numerator = (unsigned)(min * 1000000.0);
        minr.denominator = (unsigned)1000000 ;

        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, degr);
        exif_set_rational(entry->data+sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, minr);
        exif_set_rational(entry->data+2*sizeof(ExifRational), EXIF_BYTE_ORDER_INTEL, secr);
    }

    /* GPS Alt */
    {
        // create our latitude tag, the whole  field is 24 bytes long
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_ALTITUDE, 8);

        // Set the field's format and number of components, this is very important!
        entry->format = EXIF_FORMAT_RATIONAL;
        entry->components = 1;

        ExifRational altr;
        altr.numerator = (unsigned)(dalt * 1000000);
        altr.denominator = (unsigned)1000000 ;
        exif_set_rational(entry->data, EXIF_BYTE_ORDER_INTEL, altr);
    }

    /* GPS Lon Ref */
    {
        char value[2] = "W";
        if (dlon > 0) memcpy(value, "E", sizeof(value)-1);

        printf("lon ref %lu\n", sizeof(value)-1);
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LONGITUDE_REF, sizeof(value)-1);
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, value, sizeof(value)-1);
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_ASCII;
    }

    /* GPS Lat Ref */
    {
        char value[2] = "S";
        if (dlat > 0) memcpy(value, "N", sizeof(value)-1);

        printf("lat ref %lu\n", sizeof(value)-1);
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_LATITUDE_REF, sizeof(value)-1);
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, value, sizeof(value)-1);
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_ASCII;
    }

    /* GPS Alt Ref */
    {
        // https://github.com/avsej/exif_geo_tag/blob/master/ext/exif_geo_tag.c
        int value = 0;  // above sea

        printf("Alt ref %lu\n", sizeof(value));
        entry = create_tag(mExifData, EXIF_IFD_GPS, (ExifTag)EXIF_TAG_GPS_ALTITUDE_REF, sizeof(value));
        /* Write the actual comment text, without the trailing NUL character */
        memcpy(entry->data, &value, sizeof(value));
        /* create_tag() happens to set the format and components correctly for
         * EXIF_TAG_USER_COMMENT, so there is nothing more to do. */
        entry->format = EXIF_FORMAT_BYTE;
        entry->components = 1;
    }

    //Write back exif data
    jpeg_data_set_exif_data(mJpegData,mExifData);

    //Save to JPG
    jpeg_data_save_file(mJpegData, mInputFilename);
}


void* call_snapshot_rgb(void * temp)
{
   // printf("madhu call_snapshot_rgb: %f\n", ros::Time::now());
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    pthread_mutex_lock(&snapshot_rgb_mut);

    double loop_rate = ros::param::param<double>("/drone/vision/video1/snapshot/loop_rate", 3.0);
    ros::Rate rate(loop_rate);
    double snapshot_interval = ros::param::param<double>("/drone/vision/video1/snapshot/snapshot_interval_1", 0.0);

    ros::Time now_is = ros::Time::now();  // time we received this status
    ros::Time last_write = now_is;

    ROS_INFO("RGB SNAPSHOT: starting snapshot worker thread.");
    while(ros::ok())
    {
        ROS_INFO("RGB SNAPSHOT: loop snapshot_rframe_rgb");
        // snapshot_interval = ros::param::param<double>("/drone/vision/video1/snapshot/snapshot_interval_1", 0.0);
        now_is = ros::Time::now();
        DroneState local_drone_state;
        if ((now_is - last_write).toSec() > snapshot_interval)
        {
            last_write = now_is;

            if (snapshot_cnt_rgb == 1)  // ready for processing
            {

                ROS_INFO("RGB SNAPSHOT: get bird telem for snapshot");
                {
                    // copy DroneState drone_state_;
                    std::lock_guard<std::mutex> lock(bird_status_mutex_);
                    local_drone_state = drone_state_;
                }
                ROS_INFO("RGB SNAPSHOT: snapshot_rframe_rgb exif gathering gps (%f, %f) alt (%f) hdg (%f) gimbal rpy (%f, %f, %f)",
                    local_drone_state.vehicle_lat, local_drone_state.vehicle_lon,
                    local_drone_state.vehicle_alt_asl, local_drone_state.vehicle_hdg,
                    local_drone_state.gimbal_rol, local_drone_state.gimbal_pit, local_drone_state.gimbal_yaw
                    );

                ROS_INFO("RGB SNAPSHOT: write snapshot_rframe_rgb");
                char filename_tmp[1024];
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);
                sprintf(filename_tmp, "/data/photos/rgb_launch_%s_time_%s.jpg", local_drone_state.launch_id.c_str(), iso_time_str.c_str());

                vector<int> compression_params;
                // compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                // compression_params.push_back(0);
                compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
                compression_params.push_back(100);

                try {
                    imwrite(filename_tmp, snapshot_rframe_rgb, compression_params);
                }
                catch (runtime_error& ex) {
                    ROS_ERROR("RGB SNAPSHOT: snapshot error: %s", ex.what());
                }
                try {
                    // add exif data
                    add_exif_to_image(filename_tmp,
                      local_drone_state.vehicle_lat,
                      local_drone_state.vehicle_lon,
                      local_drone_state.vehicle_alt_asl);
                }
                catch (runtime_error& ex) {
                    ROS_ERROR("RGB SNAPSHOT: snapshot exif error: %s", ex.what());
                }
            }
            else
            {
                ROS_INFO("RGB SNAPSHOT: skip snapshot_rframe_rgb");
            }

            snapshot_rframe_rgb.release();
            snapshot_cnt_rgb = 0;
        }
        ros::spinOnce();

        if (gi_stop == 0)
            pthread_cond_wait(&snapshot_rgb_cond, &snapshot_rgb_mut);

        if (gi_stop != 0)
        {
            snapshot_cnt_rgb = 20;

            pthread_mutex_unlock(&snapshot_rgb_mut);
            pthread_exit(NULL);
        }
        rate.sleep();
    }

    // sid check if needed
    snapshot_cnt_rgb = 20;
    pthread_mutex_unlock(&snapshot_rgb_mut);
    pthread_exit(NULL);
}
GstFlowReturn new_snapshot_rgb(GstAppSink *elt, gpointer ptr)
{
    snapshot_gst_rgb = gst_app_sink_pull_sample (GST_APP_SINK (elt));

    // throttle
    double snapshot_interval = ros::param::param<double>("/drone/vision/video1/snapshot/snapshot_interval_2", 5.0);
    ros::Time now_is = ros::Time::now();  // time we received this status
    if ((now_is - last_snapshot).toSec() > snapshot_interval)
    {
        last_snapshot = now_is;
       // printf("madhulastsnapshot", last_snapshot);

        //thread creation
        if (snapshot_cnt_rgb == 10)
        {
            pthread_create(&snapshot_thread_rgb, NULL, call_snapshot_rgb, NULL );
            ROS_INFO("RGB SNAPSHOT: RGB snapshot thread created");
            snapshot_cnt_rgb = 0;
        }

        //frame data extraction
        if (snapshot_cnt_rgb == 0)
        {
            if (pthread_mutex_trylock(&snapshot_rgb_mut) == 0)
            {
                //get the sample from appsink

                buffer_snapshot_rgb = gst_sample_get_buffer (snapshot_gst_rgb);
                if (buffer_snapshot_rgb)
                {
                    gst_buffer_map (buffer_snapshot_rgb, &snapshot_map_rgb, GST_MAP_READ);

                    // incoming is yuv i420
                    cv::Mat yuv_frame(rgb_snapshot_height+rgb_snapshot_height/2, rgb_snapshot_width,
                                      CV_8UC1, (char*)snapshot_map_rgb.data, cv::Mat::AUTO_STEP);

                    bool use_rgb = ros::param::param<bool>("/drone/vision/video1/snapshot/use_rgb", true);
                    if (use_rgb)
                    {
                        // using rgb
                        cv::Mat rgb_frame(cv::Size(rgb_snapshot_width, rgb_snapshot_height), CV_8UC4);
                        // cv::cvtColor(yuv_frame, rgb_frame, CV_YCrCb2RGB);  // this segfaults
                        // cv::cvtColor(yuv_frame, rgb_frame, CV_YUV2RGB_YV12);
                        cv::cvtColor(yuv_frame, rgb_frame, CV_YUV420p2RGB);
                        snapshot_rframe_rgb = rgb_frame.clone();
                        rgb_frame.release();
                    }
                    else
                    {
                        // using rgba
                        cv::Mat rgba_frame(cv::Size(rgb_snapshot_width, rgb_snapshot_height), CV_8UC4);
                        cv::cvtColor(yuv_frame, rgba_frame, CV_YUV2RGBA_NV21);
                        snapshot_rframe_rgb = rgba_frame.clone();
                        rgba_frame.release();
                    }

                    // assuming buffer is already rgb
                    // cv::Mat rgb_frame(cv::Size(rgb_snapshot_width, rgb_snapshot_height),
                    //                   CV_8UC3, (char*)snapshot_map_rgb.data, cv::Mat::AUTO_STEP);

                    yuv_frame.release();

                    gst_buffer_unmap(buffer_snapshot_rgb, &snapshot_map_rgb);
                    snapshot_cnt_rgb = 1;  // signal processing thread to start
                }

                pthread_cond_signal(&snapshot_rgb_cond);
                pthread_mutex_unlock(&snapshot_rgb_mut);
            }
        }

    }

    // sid check gi_stop
    if (snapshot_gst_rgb )
        gst_sample_unref(snapshot_gst_rgb);

    return GST_FLOW_OK;
}

GstFlowReturn new_sample_thermal(GstAppSink *elt, gpointer ptr)
{
    // skip throttle by framerate

    //thread creation
    if (cnt_thr == CAP_INIT)
    {
        pthread_create(&det_thread_thr, NULL, call_det_thr, NULL );
        ROS_INFO("THR2: Thermal Detection thread created");
        cnt_thr = CAP_READY;
    }

    // ROS_DEBUG("THR2: frame available");
    if ((ros::Time::now() - t_det_last_update) > t_det_duration)
    {
        t_det_last_update = ros::Time::now();
        
       // printf("madhu tdetlastupdate%f", t_det_last_update);
        //frame data extraction
        // ROS_DEBUG("THR2: new_sample_thermal");
        sample_gst_thr = gst_app_sink_pull_sample (GST_APP_SINK (elt));
        //get the sample from appsink
        buffer_gst_thr = gst_sample_get_buffer (sample_gst_thr);

        if (buffer_gst_thr)
        {
            try {
                gst_buffer_map (buffer_gst_thr, &map_thr, GST_MAP_READ);
                cv::Mat frame(cv::Size(thr_det_width, thr_det_height), CV_8UC3, (char*)map_thr.data, cv::Mat::AUTO_STEP);
                /*cv::Mat rframe;
                // CPU resize, can be optimized
                cv::resize(frame, rframe, cv::Size(thr_det_width, thr_det_height));
                rframe_thr = rframe.clone(); */

                if (cnt_thr == CAP_READY)
                {
                    if (pthread_mutex_trylock(&rframe_thr_mut) == 0)
                    {
                        ROS_DEBUG("THR2: clone frame");
                        rframe_thr = frame.clone();
                       // ROS_INFO("madhu", rframe_thr);
                        // pthread_cond_signal(&thr_cond);
                        pthread_mutex_unlock(&rframe_thr_mut);
                        // rframe.release();
                        cnt_thr = DET_READY;  // signal processing thread to start
                    }
                    else ROS_DEBUG("THR2: MAT locked, skip clone frame");
                }
                else ROS_DEBUG("THR2: detection active, skip clone frame");

                gst_buffer_unmap(buffer_gst_thr, &map_thr);
                frame.release();
            }
            catch (...) { ROS_ERROR("THR2: exception new_sample_thermal cv"); }
        }

        if (sample_gst_thr) gst_sample_unref(sample_gst_thr);
    }

    return GST_FLOW_OK;
}

void *run_detector_rgb(void *param)
{
   
    void *pv_stream_obj_rgb = NULL;
    
    ros::NodeHandle rgb_node;
    ros::Subscriber fast_status_sub = rgb_node.subscribe<std_msgs::Float64MultiArray>("/bird_status_fast", 2, bird_status_callback_fast);
    ros::Subscriber dcs_status_sub = rgb_node.subscribe<birdseye_msgs::BirdStatus>("/bird_status_dcs", 2, bird_status_callback_dcs);

    ros::Subscriber rgb_info_sub = rgb_node.subscribe<std_msgs::Float64MultiArray>("/video/rgb/control", 3, rgb_info_callback);

    float total;
    //auto t2 = std::chrono::high_resolution_clock::now();
    while (r_enable && !rgb_start_success && gi_stop == 0) {

        if (cnt_rgb == CAP_IDLE)
            cnt_rgb = CAP_INIT;

        if (snapshot_cnt_rgb == 20)
            snapshot_cnt_rgb = 10;

        rgb.fname = std::string((char*)param);
        ROS_INFO("RGB0: Streaming fname: %s", (char*)param);

        rgb.modelid = 0;

        // RGB pipeline Initialization
        {
            //auto t3 = std::chrono::high_resolution_clock::now();
            // Initialize the CNN
            if (r_det && !RGB_model_loaded)
            {
                // camera properties, used by detect_frame to compute angle_x and angle_y of a target
                rgb.hfov = ros::param::param<float>("/drone/detection/rgb/hfov", 2.0*0.8255407362);
                rgb.vfov = ros::param::param<float>("/drone/detection/rgb/vfov", 2.0*0.547430511);
                rgb.tmargin = ros::param::param<float>("/drone/detection/rgb/tmargin", 0.0);

                // detection weights
                std::string home_dir = ros::param::param<std::string>("/drone/vision/video1/detect/home_dir", "/home/nsd_user");
                rgb.railcar = ros::param::param<bool>("/drone/vision/video1/detect/railcar", false);
                if (rgb.railcar) {
                    std::string rgb_rail = ros::param::param<std::string>("/drone/vision/video1/detect/rgb_railcar_model",
                        home_dir + "/code/catkin_birdseye/install/share/ngale_ros/models/yolov3_rail_fp16.engine");
                    string engineName(rgb_rail.c_str());
                    rgb.net.reset(new trtNet(engineName));
                }
                else {
                    std::string rgb_vp = ros::param::param<std::string>("/drone/vision/video1/detect/rgb_person_vehicle_model",
                        home_dir + "/code/catkin_birdseye/install/share/ngale_ros/models/yolov3_rgb_fp16.engine");
                    string engineName(rgb_vp.c_str());
                    rgb.net.reset(new trtNet(engineName));
                }
                RGB_model_loaded = 1;
                rgb.frame_id = 0;
            }
            //auto t3stop = std::chrono::high_resolution_clock::now();
            //total = std::chrono::duration<float, milli>(t3stop - t3).count();
            //std::cout<<"\n["<<std::this_thread::get_id()<<"] | TIME for RGB pipeline intialization | : "<<total<<" ms.\n\n";

            // Start RGB pipeline
            printf("\n[%lu] | RGB Pipeline start\n", std::this_thread::get_id());
            pv_stream_obj_rgb = rgb.stream_start_rgb();
            if (pv_stream_obj_rgb == NULL)
            {
                ROS_INFO("RGB0: Error in starting rgb stream");
                // exit(1);
            }
            else {
                rgb_start_success = true;
            }
        }

        usleep(3000000);        //<sagarwal> sleep for 3 sec: why?
        if (r_enable && !rgb_start_success) {
            ROS_ERROR("RGB0: Error in starting rgb stream, stopping before restart");
            rgb.stream_stop(pv_stream_obj_rgb);
            usleep(1000000);
        }
    }
    //auto t2stop = std::chrono::high_resolution_clock::now();
    //total = std::chrono::duration<float, milli>(t2stop - t2).count();
    //std::cout<<"\n["<<std::this_thread::get_id()<<"] | TIME : "<<total<<" ms.\n\n";
    ROS_INFO("RGB0: Streaming started");
    //printf("[%lu]\n", std::this_thread::get_id()); 
    // Sleep for 10 milliseconds till we get a break
    double overlay_update_rate  = ros::param::param<double>("/drone/vision/video1/stream/overlay/update_rate"   , 3.0);
    std::vector<double> overlay_x_c;
    std::vector<double> overlay_y_c;
    std::vector<double> overlay_x_p;
    std::vector<double> overlay_y_p;
   /* modified to read param values overlay x and y
      once for each iteration instead of calling twice
      this saves some time (~100ms)
   */
    double x,y; 	//<sagarwal>
    int num_markers = 3;
	//auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < num_markers; ++i) {
        x = ros::param::param<double>("/drone/vision/video1/stream/overlay/x"         , 0.5);
	    y = ros::param::param<double>("/drone/vision/video1/stream/overlay/y"         , 0.5);
        //overlay_x_p.push_back(ros::param::param<double>("/drone/vision/video1/stream/overlay/x"         , 0.5));	//testing to reduce time <sagarwal>
	    //overlay_y_p.push_back(ros::param::param<double>("/drone/vision/video1/stream/overlay/y"         , 0.5));	//testing to reduce time <sagarwal>
	    overlay_x_c.push_back(x);
	    overlay_y_c.push_back(y);
	    overlay_x_p.push_back(x);
	    overlay_y_p.push_back(y);
    }	
	//auto t1_stop = std::chrono::high_resolution_clock::now();
	//float total = std::chrono::duration<float, milli>(t1_stop - t1).count();
	//std::cout<< "TIme taken for for loop in overlay: "<< total<<" ms.\n\n";
    bool overlay_target = ros::param::param<bool>("/drone/vision/video1/stream/overlay/target"      , true);

    ros::Rate rate(overlay_update_rate);
    ros::Time last_rgb_det_timestamp = ros::Time::now();
    // ROS_INFO(ros::Time last_rgb_det_timestamp);
    birdseye_msgs::DetectionResult detection_raw_local;
    auto t1 = std::chrono::high_resolution_clock::now();
    while(gi_stop == 0)
    {
        //printf("[%lu]",std::this_thread::get_id());
        /* update text overlay */
        if (cnt_rgb == VIS_READY && r_enable && !r_rec && rgb.stream_test_textoverlay)
        {

            if (overlay_target) {

                // show target
                detection_raw_local.objects.clear();
                // ROS_DEBUG("RGB0: copy targets");
                {
                    ROS_INFO("RGB DETECTION PIPE: (5) COPY DETECTIONS for VISUALS");
                    // ROS_DEBUG("RGB5: clone detection_raw_viz");
                    std::lock_guard<std::mutex> lock(rgb.detection_raw_viz_mutex);
                    detection_raw_local.width = rgb.detection_raw_viz.width;
                    detection_raw_local.height = rgb.detection_raw_viz.height;
		    //ROS_INFO("detection_raw_viz size: ",rgb.detection_raw_viz.objects.size());
                    for (int i = 0; i < rgb.detection_raw_viz.objects.size(); ++i) {
                        detection_raw_local.objects.push_back(rgb.detection_raw_viz.objects[i]);
                    }
                    // ROS_DEBUG("RGB5: clone detection_raw_viz done");
                    cnt_rgb = CAP_READY;
                }

                // hide markers
                for (int i = 0; i < num_markers; ++i) {
                    overlay_x_c[i] = -0.5;
                    overlay_y_c[i] = -0.5;
                    overlay_x_p[i] = -0.5;
                    overlay_y_p[i] = -0.5;
                }

                // NOTE: we've prefilled all objects in the detection_raw_viz array with the same timestampe, so only need to check 1
                if (!detection_raw_local.objects.empty() && detection_raw_local.objects[0].header.stamp != last_rgb_det_timestamp) {

                    ROS_DEBUG("RGB0: got new targets, update text overlays");

                    last_rgb_det_timestamp = detection_raw_local.objects[0].header.stamp;
                   // printf(last_rgb_det_timestamp);

                    int index_c = 0;
                    int index_p = 0;
                    for (int i = 0; i < detection_raw_local.objects.size(); ++i) {
                        if (detection_raw_local.objects[i].class_id == 1) {
                            // car
                            if (index_c < num_markers) {
                                overlay_x_c[index_c] = detection_raw_local.objects[i].bbox_x / rgb_det_width;
                                overlay_y_c[index_c] = detection_raw_local.objects[i].bbox_y / rgb_det_height;
                                ++index_c;
                            }
                        }
                        else if (detection_raw_local.objects[i].class_id == 0) {
                            // ped
                            if (index_p < num_markers) {
                                overlay_x_p[index_p] = detection_raw_local.objects[i].bbox_x / rgb_det_width;
                                overlay_y_p[index_p] = detection_raw_local.objects[i].bbox_y / rgb_det_height;
                                ++index_p;
                            }
                        }
                    }
                    ROS_DEBUG("RGB0: car (%f, %f)(%f, %f)(%f, %f)", overlay_x_c[0], overlay_y_c[0],
                                                                    overlay_x_c[1], overlay_y_c[1],
                                                                    overlay_x_c[2], overlay_y_c[2]);
                    ROS_DEBUG("RGB0: ped (%f, %f)(%f, %f)(%f, %f)", overlay_x_p[0], overlay_y_p[0],
                                                                    overlay_x_p[1], overlay_y_p[1],
                                                                    overlay_x_p[2], overlay_y_p[2]);
                }
            }
            for (int i = 0; i < num_markers; ++i) {
                g_object_set(G_OBJECT(rgb.s_text_overlay_c[i]), "x-absolute", overlay_x_c[i], NULL);
                g_object_set(G_OBJECT(rgb.s_text_overlay_c[i]), "y-absolute", overlay_y_c[i], NULL);
                g_object_set(G_OBJECT(rgb.s_text_overlay_p[i]), "x-absolute", overlay_x_p[i], NULL);
                g_object_set(G_OBJECT(rgb.s_text_overlay_p[i]), "y-absolute", overlay_y_p[i], NULL);
            }

            if (rgb.s_text_overlay2 != NULL)
            {
                // show time
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

                std::string test1_str = "";  // "ok\nthis is a sentence\n";
                std::string whole_str = test1_str + iso_time_str;

                g_object_set(G_OBJECT(rgb.s_text_overlay2), "text", whole_str.c_str(), NULL);
            }
        }

        {   // no visuals, but still release capture state chnage for next frame
            // ROS_DEBUG("RGB0: copy targets");
            ROS_INFO("RGB DETECTION PIPE: (5) COPY DETECTIONS for no-VISUALS");
            // ROS_DEBUG("RGB5: clone detection_raw_viz");
            std::lock_guard<std::mutex> lock(rgb.detection_raw_viz_mutex);
            // ROS_DEBUG("RGB5: clone detection_raw_viz done");
            cnt_rgb = CAP_READY;
        }

        if (rgb.write_test_textoverlay)
        {
            boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
            std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

            std::string test1_str = "";  // "ok\nthis is a sentence\n";
            std::string whole_str = test1_str + iso_time_str;

            g_object_set(G_OBJECT(rgb.r_text_overlay),"text", whole_str.c_str(), NULL);
        }

        // if (rgb_info_.stream_bitrate_ > 0) {
        //     std::string vision_prefix = "/drone/vision/video1";
        //     int stream_bitrate = ros::param::param<int>(vision_prefix+ "/stream/bitrate_override", rgb_info_.stream_bitrate_);
        //     g_object_set(G_OBJECT(rgb.g_st_enc), "bitrate", int (stream_bitrate * .83333), NULL);
        // }

        // ros::spinOnce();
        rate.sleep();
    }
	//auto t1stop = std::chrono::high_resolution_clock::now();
	//total = std::chrono::duration<float, milli>(t1stop - t1).count();
	//std::cout<<"\n["<<std::this_thread::get_id()<<"] |  Time taken for text overlay is :"<<total<< "ms\n\n";

    usleep(10000);

    ROS_INFO("Calling RGB Stream Stop");

    //Stop RGB pipeline
    rgb_start_success = false;
    rgb.stream_stop(pv_stream_obj_rgb);
    ROS_INFO("Completed RGB Pipeline");

    ROS_INFO("Sending signal for RGB conditional wait in stop");
    pthread_cond_signal(&rgb_full_cond);

}

void *run_detector_thermal(void *param)
{
    void *pv_stream_obj_thermal = NULL;

    ros::NodeHandle thermal_node;
    // ros::Subscriber fast_status_sub = thermal_node.subscribe<std_msgs::Float64MultiArray>("/bird_status_fast", 3, bird_status_callback_fast);
    // ros::Subscriber dcs_status_sub = thermal_node.subscribe<birdseye_msgs::BirdStatus>("/bird_status_dcs", 3, bird_status_callback_dcs);

    ros::Subscriber thermal_info_sub = thermal_node.subscribe<std_msgs::Float64MultiArray>("/video/thermal/control", 3, thermal_info_callback);
    while (t_enable && !thr_start_success && gi_stop == 0) {

        if (cnt_thr == CAP_IDLE)
            cnt_thr = CAP_INIT;

        thermal.fname = std::string((char*)param);
        ROS_INFO("THR0: Thermal Streaming fname: %s", (char*)param);

        thermal.modelid = 1;

        //Thermal pipeline Initialization
        {
            // Initialize the CNN
            if (t_det && !THERMAL_model_loaded)
            {
                // camera properties, used by detect_frame to compute angle_x and angle_y of a target
                thermal.hfov = ros::param::param<float>("/drone/detection/thermal/hfov", 2.0*0.41887902047);
                thermal.vfov = ros::param::param<float>("/drone/detection/thermal/vfov", 2.0*0.33510321638);
                thermal.tmargin = ros::param::param<float>("/drone/detection/thermal/tmargin", 0.0);

                // detection weights
                std::string home_dir = ros::param::param<std::string>("/drone/vision/video2/detect/home_dir", "/home/nsd_user");
                std::string thr_vp = ros::param::param<std::string>("/drone/vision/video2/detect/thermal_person_vehicle_model",
                    home_dir + "/code/catkin_birdseye/install/share/ngale_ros/models/yolov3_thermal_fp16.engine");
                ROS_INFO("THR0: thermal detection engine [%s]\n", thr_vp.c_str());
                string engineName(thr_vp.c_str());
                thermal.net.reset(new trtNet(engineName));
                THERMAL_model_loaded = 1;
                thermal.frame_id = 0;
            }

            // Start Thermal pipeline
            pv_stream_obj_thermal = thermal.stream_start_thermal();
            if (pv_stream_obj_thermal == NULL)
            {
                ROS_ERROR("Error in starting thermal stream");
                // exit(1);
            }
            else {
                thr_start_success = true;
            }
        }

        usleep(3000000);
        if (t_enable && !thr_start_success) {
            ROS_ERROR("Error in starting thermal stream, stopping before restart");
            thermal.stream_stop(pv_stream_obj_thermal);
            usleep(1000000);
        }
    }
    ROS_INFO("Thermal Streaming started");

    // Sleep for 10 milliseconds till we get a break
    double overlay_update_rate               = ros::param::param<double>("/drone/vision/video2/stream/overlay/update_rate"   , 3.0);
    std::vector<double> overlay_x_c;
    std::vector<double> overlay_y_c;
    std::vector<double> overlay_x_p;
    std::vector<double> overlay_y_p;
    int num_markers = 3;
    for (int i = 0; i < num_markers; ++i) {
        overlay_x_c.push_back(ros::param::param<double>("/drone/vision/video2/stream/overlay/x"         , 0.5));
        overlay_y_c.push_back(ros::param::param<double>("/drone/vision/video2/stream/overlay/y"         , 0.5));
        overlay_x_p.push_back(ros::param::param<double>("/drone/vision/video2/stream/overlay/x"         , 0.5));
        overlay_y_p.push_back(ros::param::param<double>("/drone/vision/video2/stream/overlay/y"         , 0.5));
    }
    bool overlay_target           = ros::param::param<bool>("/drone/vision/video2/stream/overlay/target"      , true);

    ros::Rate rate(overlay_update_rate);
    ros::Time last_thr_det_timestamp = ros::Time::now();
    birdseye_msgs::DetectionResult detection_raw_local;
    while(gi_stop == 0)
    {
        // ROS_INFO("Thermal Streaming running");

        /* update text overlay */
        if (cnt_thr = VIS_READY && t_enable && !t_rec && thermal.stream_test_textoverlay)
        {
            if (overlay_target) {

                // show target
                detection_raw_local.objects.clear();
                {
                    std::lock_guard<std::mutex> lock(thermal.detection_raw_viz_mutex);
                    detection_raw_local.width = thermal.detection_raw_viz.width;
                    detection_raw_local.height = thermal.detection_raw_viz.height;
                    for (int i = 0; i < thermal.detection_raw_viz.objects.size(); ++i) {
                        detection_raw_local.objects.push_back(thermal.detection_raw_viz.objects[i]);
                    }
                    cnt_thr = CAP_READY;
                }

                // hide markers
                for (int i = 0; i < num_markers; ++i) {
                    overlay_x_c[i] = -0.5;
                    overlay_y_c[i] = -0.5;
                    overlay_x_p[i] = -0.5;
                    overlay_y_p[i] = -0.5;
                }

                if (!detection_raw_local.objects.empty() && detection_raw_local.objects[0].header.stamp != last_thr_det_timestamp) {

                    ROS_DEBUG("THR0: got new targets, update text overlays");
                    last_thr_det_timestamp = detection_raw_local.objects[0].header.stamp;

                    int index_c = 0;
                    int index_p = 0;
                    for (int i = 0; i < detection_raw_local.objects.size(); ++i) {
                        if (detection_raw_local.objects[i].class_id == 1) {
                            // car
                            if (index_c < num_markers) {
                                overlay_x_c[index_c] = detection_raw_local.objects[i].bbox_x / thr_det_width;
                                overlay_y_c[index_c] = detection_raw_local.objects[i].bbox_y / thr_det_height;
                                ++index_c;
                            }
                        }
                        else if (detection_raw_local.objects[i].class_id == 0) {
                            // ped
                            if (index_p < num_markers) {
                                overlay_x_p[index_p] = detection_raw_local.objects[i].bbox_x / thr_det_width;
                                overlay_y_p[index_p] = detection_raw_local.objects[i].bbox_y / thr_det_height;
                                ++index_p;
                            }
                        }
                    }
                    ROS_DEBUG("THR0: car (%f, %f)(%f, %f)(%f, %f)", overlay_x_c[0], overlay_y_c[0],
                                                                    overlay_x_c[1], overlay_y_c[1],
                                                                    overlay_x_c[2], overlay_y_c[2]);
                    ROS_DEBUG("THR0: ped (%f, %f)(%f, %f)(%f, %f)", overlay_x_p[0], overlay_y_p[0],
                                                                    overlay_x_p[1], overlay_y_p[1],
                                                                    overlay_x_p[2], overlay_y_p[2]);
                }
            }
            for (int i = 0; i < num_markers; ++i) {
                g_object_set(G_OBJECT(thermal.s_text_overlay_c[i]), "x-absolute", overlay_x_c[i], NULL);
                g_object_set(G_OBJECT(thermal.s_text_overlay_c[i]), "y-absolute", overlay_y_c[i], NULL);
                g_object_set(G_OBJECT(thermal.s_text_overlay_p[i]), "x-absolute", overlay_x_p[i], NULL);
                g_object_set(G_OBJECT(thermal.s_text_overlay_p[i]), "y-absolute", overlay_y_p[i], NULL);
            }

            if (thermal.s_text_overlay2 != NULL)
            {
                // show time
                boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
                std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

                std::string test1_str = "";  // "ok\nthis is a sentence\n";
                std::string whole_str = test1_str + iso_time_str;

                g_object_set(G_OBJECT(thermal.s_text_overlay2),"text", whole_str.c_str(), NULL);
            }
        }

        {   // no visuals, but still release capture state chnage for next frame
            ROS_INFO("THR DETECTION PIPE: (5) COPY DETECTIONS for no-VISUALS");
            std::lock_guard<std::mutex> lock(thermal.detection_raw_viz_mutex);
            cnt_thr = CAP_READY;
        }

        if (t_enable && thermal.write_test_textoverlay)
        {
            // skip_count = 0;
            boost::posix_time::ptime my_posix_time = ros::Time::now().toBoost();
            std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);

            std::string test1_str = "";  // "ok\nthis is a sentence\n";
            std::string whole_str = test1_str + iso_time_str;

            g_object_set(G_OBJECT(thermal.r_text_overlay),"text", whole_str.c_str(), NULL);
        }

        // if (thermal_info_.stream_bitrate_ > 0) {
        //     std::string vision_prefix = "/drone/vision/video2";
        //     int stream_bitrate = ros::param::param<int>(vision_prefix+ "/stream/bitrate_override", thermal_info_.stream_bitrate_);
        //     g_object_set(G_OBJECT(thermal.g_st_enc), "bitrate", int (stream_bitrate * .83333), NULL);
        // }

        // ros::spinOnce();
        rate.sleep();
    }

    usleep(10000);
    
    ROS_INFO("Calling thermal Stream Stop");
    // Stop Thermal pipeline
    thr_start_success = false;
    thermal.stream_stop(pv_stream_obj_thermal);
    ROS_INFO("Completed Thermal Pipeline");

    ROS_INFO("Sending signal for THERMAL conditional wait in stop");
    pthread_cond_signal(&thr_full_cond);
}
