#include "yaml-cpp/yaml.h"
#include <iostream>
#include "interface.h"
#include <string.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>

int railcar;
int r_framerate, r_textoverlay;
int s_framerate, s_textoverlay;
std::string rgb_vp;
std::string thr_vp;
std::string rgb_rail;

void parse_config(stream_config_t *ps_stream_config, const std::string &conf_file, const std::string &cam_type)
{
    std::string vision_prefix = "/drone/vision/video1";
    std::string stream_target_ip = "192.168.168.4";
    int stream_target_port = 13001;

    memset(ps_stream_config, 0,sizeof(stream_config_t));

    YAML::Node node = YAML::LoadFile(conf_file);

    YAML::Node drone = node["drone"];
    for(YAML::const_iterator it=drone.begin();it != drone.end();++it) 
    {
        std::string key = it->first.as<std::string>();

        const YAML::Node& pair = it->second;

        if(pair["video1"] || pair["video2"])
        {
            YAML::Node video;
            if(cam_type == "rgb")
            {
                std::cout << "In RGB......................." << std::endl;
                vision_prefix = "/drone/vision/video1";
                video = pair["video1"];
            }
            else
            {
                std::cout << "In Thermal......................." << std::endl;
                vision_prefix = "/drone/vision/video2";
                video = pair["video2"];
            }

            for(YAML::const_iterator v1=video.begin();v1 != video.end();++v1)
            {

               
               if(v1->first.as<std::string>() == "file_input")
                   ps_stream_config->file_input = ros::param::param<int>(vision_prefix + "/file_input", v1->second.as<int>());

               if(v1->first.as<std::string>() == "filename")
                   ps_stream_config->filename = ros::param::param<std::string>(vision_prefix + "/filename", v1->first.as<std::string>().c_str()); 

               if(v1->first.as<std::string>() == "railcar")
                   railcar = ros::param::param<int>(vision_prefix + "/railcar", v1->second.as<int>());
                
               if(v1->first.as<std::string>() == "rgb_person_vehicle_model")
                   rgb_vp = ros::param::param<std::string>(vision_prefix + "/rgb_person_vehicle_model", v1->second.as<std::string>().c_str());

               if(v1->first.as<std::string>() == "rgb_railcar_model")
                    rgb_rail = ros::param::param<std::string>(vision_prefix + "/rgb_railcar_model", v1->second.as<std::string>().c_str());

               if(v1->first.as<std::string>() == "thermal_person_vehicle_model")
                    thr_vp = ros::param::param<std::string>(vision_prefix + "/thermal_person_vehicle_model", v1->second.as<std::string>().c_str());

               if(v1->first.as<std::string>() == "sensor_id")
                    ps_stream_config->i_sensor_id = ros::param::param<int>(vision_prefix + "/sensor_id", v1->second.as<int>());

               if(v1->first.as<std::string>() == "min_fps")
                    ps_stream_config->i_min_fps = ros::param::param<int>(vision_prefix + "/min_fps", v1->second.as<int>());

               if(v1->first.as<std::string>() == "max_fps")
                    ps_stream_config->camera_fps_override = ros::param::param<int>(vision_prefix + "/camera_fps_override", v1->second.as<int>());

               if(v1->first.as<std::string>() == "src_width")
                    ps_stream_config->i_src_width = ros::param::param<int>(vision_prefix + "/src_width", v1->second.as<int>());

               if(v1->first.as<std::string>() == "src_height")
                    ps_stream_config->i_src_height = ros::param::param<int>(vision_prefix + "/src_height", v1->second.as<int>());

               if(v1->first.as<std::string>() == "split_duration")
                    ps_stream_config->i_split_dur = ros::param::param<int>(vision_prefix + "/split_duration", v1->second.as<int>());

               if(v1->first.as<std::string>() == "log_file")
                    ps_stream_config->ac_log_file = ros::param::param<std::string>(vision_prefix + "/log_file", v1->second.as<std::string>().c_str());

               if(v1->first.as<std::string>() == "log_level"){
                   std::string log_level = ros::param::param<std::string>(vision_prefix + "/log_level", v1->second.as<std::string>().c_str());
               	   if(log_level == "LOG_ERROR")
                       ps_stream_config->i_log_level = LOG_ERROR;
               }

               if(v1->first.as<std::string>() == "h265"){
                    std::string h265 = ros::param::param<std::string>(vision_prefix + "/h265", v1->second.as<std::string>().c_str());
                   if(h265 == "true")
                       ps_stream_config->i_enc_var = H265;
                   else
                       ps_stream_config->i_enc_var = H264;
               }

               // Write
               if(v1->first.as<std::string>() == "write")
               {
                  const YAML::Node& write = v1->second;

                  r_framerate = ros::param::param<int>(vision_prefix + "/write/framerate", write["framerate"].as<int>());
                  r_textoverlay = ros::param::param<int>(vision_prefix + "/write/textoverlay", write["textoverlay"].as<int>());
                  ps_stream_config->as_enc[0].bitrate = ros::param::param<int>(vision_prefix + "/write/bitrate_override", write["bitrate_override"].as<int>());
                  ps_stream_config->as_enc[0].width_override = ros::param::param<int>(vision_prefix + "/write/width_override", write["width_override"].as<int>());
                  ps_stream_config->as_enc[0].height_override = ros::param::param<int>(vision_prefix + "/write/height_override",write["height_override"].as<int>());
                  ps_stream_config->as_enc[0].preset_level = ros::param::param<int>(vision_prefix + "/write/preset_level", write["preset_level"].as<int>());
                  ps_stream_config->as_enc[0].profile = ros::param::param<int>(vision_prefix + "/write/profile", write["profile"].as<int>());
                  ps_stream_config->as_enc[0].iframeinterval = ros::param::param<int>(vision_prefix + "/write/iframeinterval", write["iframeinterval"].as<int>());
                  ps_stream_config->as_enc[0].b_frames = ros::param::param<int>(vision_prefix + "/write/b_frames", write["b_frames"].as<int>());
                  ps_stream_config->as_enc[0].rate_control = ros::param::param<int>(vision_prefix + "/write/rate_control", write["rate_control"].as<int>());
                  ps_stream_config->as_enc[0].qp_range = ros::param::param<std::string>(vision_prefix + "/write/qp_range", write["qp_range"].as<std::string>().c_str());
                  ps_stream_config->as_enc[0].level = ros::param::param<std::string>(vision_prefix + "/write/level", write["level"].as<std::string>().c_str());
                  ps_stream_config->as_enc[0].muxer = ros::param::param<std::string>(vision_prefix + "/write/muxer", write["muxer"].as<std::string>().c_str());
                  ps_stream_config->as_enc[0].EnableTwopassCBR = ros::param::param<int>(vision_prefix + "/write/EnableTwopassCBR", write["EnableTwopassCBR"].as<int>()); 
                  ps_stream_config->as_enc[0].vbv_size = ros::param::param<int>(vision_prefix + "/write/vbv_size", write["vbv_size"].as<int>());
                  ps_stream_config->as_enc[0].temporal_tradeoff = ros::param::param<int>(vision_prefix + "/write/temporal_tradeoff", write["temporal_tradeoff"].as<int>());
                  ps_stream_config->as_enc[0].insert_sps_pps = ros::param::param<int>(vision_prefix + "/write/insert_sps_pps", write["insert_sps_pps"].as<int>());
                  ps_stream_config->as_enc[0].output_file = ros::param::param<std::string>(vision_prefix + "/write/output_file", write["output_file"].as<std::string>().c_str());

                  if (write["enc_type"].as<std::string>() == "ENC_HW")
                      ps_stream_config->as_enc[0].e_enc_type  = ENC_HW;
                  else
                      ps_stream_config->as_enc[0].e_enc_type  = ENC_SW;
                  
               }

               // Stream
               if(v1->first.as<std::string>() == "stream")
               {
                  const YAML::Node& stream = v1->second;

                  s_framerate = ros::param::param<int>(vision_prefix + "/stream/framerate", stream["framerate"].as<int>());
                  s_textoverlay = ros::param::param<int>(vision_prefix + "/stream/textoverlay", stream["textoverlay"].as<int>());
                  ps_stream_config->as_enc[1].bitrate = ros::param::param<int>(vision_prefix + "/stream/bitrate_override", stream["bitrate_override"].as<int>());
                  ps_stream_config->as_enc[1].width_override = ros::param::param<int>(vision_prefix + "/stream/width_override", stream["width_override"].as<int>());
                  ps_stream_config->as_enc[1].height_override = ros::param::param<int>(vision_prefix + "/stream/height_override", stream["height_override"].as<int>());
                  ps_stream_config->as_enc[1].preset_level = ros::param::param<int>(vision_prefix + "/stream/preset_level", stream["preset_level"].as<int>());
                  ps_stream_config->as_enc[1].profile = ros::param::param<int>(vision_prefix + "/stream/profile", stream["profile"].as<int>());
                  ps_stream_config->as_enc[1].iframeinterval = ros::param::param<int>(vision_prefix + "/stream/iframeinterval", stream["iframeinterval"].as<int>());
                  ps_stream_config->as_enc[1].b_frames = ros::param::param<int>(vision_prefix + "/stream/b_frames", stream["b_frames"].as<int>());
                  ps_stream_config->as_enc[1].rate_control = ros::param::param<int>(vision_prefix + "/stream/rate_control", stream["rate_control"].as<int>());
                  ps_stream_config->as_enc[1].qp_range = ros::param::param<std::string>(vision_prefix + "/stream/qp_range", stream["qp_range"].as<std::string>().c_str());
                  ps_stream_config->as_enc[1].level = ros::param::param<std::string>(vision_prefix + "/stream/level", stream["level"].as<std::string>().c_str());
                  ps_stream_config->as_enc[1].muxer = ros::param::param<std::string>(vision_prefix + "/stream/muxer", stream["muxer"].as<std::string>().c_str());
                  ps_stream_config->as_enc[1].EnableTwopassCBR = ros::param::param<int>(vision_prefix + "/stream/EnableTwopassCBR", stream["EnableTwopassCBR"].as<int>());
                  ps_stream_config->as_enc[1].vbv_size = ros::param::param<int>(vision_prefix + "/stream/vbv_size", stream["vbv_size"].as<int>());
                  ps_stream_config->as_enc[1].temporal_tradeoff = ros::param::param<int>(vision_prefix + "/stream/temporal_tradeoff", stream["temporal_tradeoff"].as<int>());
                  ps_stream_config->as_enc[1].insert_sps_pps = ros::param::param<int>(vision_prefix + "/stream/insert_sps_pps", stream["insert_sps_pps"].as<int>());
                  ps_stream_config->as_enc[1].target_ip = ros::param::param<std::string>(vision_prefix + "/stream_target_ip", stream["target_ip"].as<std::string>().c_str());
                  ps_stream_config->as_enc[1].target_port = ros::param::param<int>(vision_prefix + "/stream_target_port", stream["target_port"].as<int>()); 
                  ps_stream_config->as_enc[1].alignment = ros::param::param<int>(vision_prefix + "/stream/alignment", stream["alignment"].as<int>());
                  ps_stream_config->as_enc[1].si_interval = ros::param::param<int>(vision_prefix + "/stream/si_interval", stream["si_interval"].as<int>());                  

                  if (stream["enc_type"].as<std::string>() == "ENC_HW")
                        ps_stream_config->as_enc[1].e_enc_type  = ENC_HW;
                  else
                        ps_stream_config->as_enc[1].e_enc_type  = ENC_SW;
               }

               // Detect
               if(v1->first.as<std::string>() == "detect")
               {

                  const YAML::Node& detect = v1->second;

                  ps_stream_config->as_enc[2].bitrate = ros::param::param<int>(vision_prefix + "/detect/bitrate_override", detect["bitrate_override"].as<int>());
                  ps_stream_config->as_enc[2].width_override = ros::param::param<int>(vision_prefix + "/detect/width_override", detect["width_override"].as<int>());
                  ps_stream_config->as_enc[2].height_override = ros::param::param<int>(vision_prefix + "/detect/height_override", detect["height_override"].as<int>());
                  ps_stream_config->as_enc[2].preset_level = ros::param::param<int>(vision_prefix + "/detect/preset_level", detect["preset_level"].as<int>());
                  ps_stream_config->as_enc[2].profile = ros::param::param<int>(vision_prefix + "/detect/profile", detect["profile"].as<int>());
                  ps_stream_config->as_enc[2].iframeinterval = ros::param::param<int>(vision_prefix + "/detect/iframeinterval", detect["iframeinterval"].as<int>());
                  ps_stream_config->as_enc[2].b_frames = ros::param::param<int>(vision_prefix + "/detect/b_frames", detect["b_frames"].as<int>());
                  ps_stream_config->as_enc[2].rate_control = ros::param::param<int>(vision_prefix + "/detect/rate_control", detect["rate_control"].as<int>());
                  ps_stream_config->as_enc[2].qp_range = ros::param::param<std::string>(vision_prefix + "/detect/qp_range", detect["qp_range"].as<std::string>().c_str());
                  ps_stream_config->as_enc[2].level = ros::param::param<std::string>(vision_prefix + "/detect/level", detect["level"].as<std::string>().c_str());
                  ps_stream_config->as_enc[2].muxer = ros::param::param<std::string>(vision_prefix + "/detect/muxer", detect["muxer"].as<std::string>().c_str());
                  ps_stream_config->as_enc[2].EnableTwopassCBR = ros::param::param<int>(vision_prefix + "/detect/EnableTwopassCBR", detect["EnableTwopassCBR"].as<int>());
                  ps_stream_config->as_enc[2].vbv_size = ros::param::param<int>(vision_prefix + "/detect/vbv_size", detect["vbv_size"].as<int>());
                  ps_stream_config->as_enc[2].temporal_tradeoff = ros::param::param<int>(vision_prefix + "/detect/temporal_tradeoff", detect["temporal_tradeoff"].as<int>());
                  ps_stream_config->as_enc[2].insert_sps_pps = ros::param::param<int>(vision_prefix + "/detect/insert_sps_pps", detect["insert_sps_pps"].as<int>());
                  ps_stream_config->as_enc[2].target_ip = ros::param::param<std::string>(vision_prefix + "/detect/target_ip", detect["target_ip"].as<std::string>().c_str());
                  ps_stream_config->as_enc[2].target_port = ros::param::param<int>(vision_prefix + "/detect/target_port", detect["target_port"].as<int>());
                  ps_stream_config->as_enc[2].format = ros::param::param<std::string>(vision_prefix + "/detect/format", detect["format"].as<std::string>().c_str());

                  if (detect["enc_type"].as<std::string>() == "ENC_HW")
                    ps_stream_config->as_enc[2].e_enc_type  = ENC_HW;
                  else
                    ps_stream_config->as_enc[2].e_enc_type  = ENC_SW;
               }
               
            }
        }

    }

}
