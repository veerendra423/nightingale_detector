/* This is proprietery code and no permission is granted to copy or redistribute this
 */

#ifndef _STREAM_H_
#define _STREAM_H_
#include <stdbool.h>
#include <stdio.h>
#define SIG_FATAL_ERROR -100
#include <string>

typedef enum _LOG_LEVEL_E_
{
    LOG_ERROR,
    LOG_WARNING,
    LOG_INFO,
    LOG_DEBUG
} LOG_LEVEL_E;

typedef enum _ENC_TYPE_E_
{
    ENC_HW,
    ENC_SW
} ENC_TYPE_E;

typedef enum _ENC_VARIANT_E_
{
    H264,
    H265
} ENC_VARIANT_E;

typedef struct _enc_config_t_
{
    ENC_TYPE_E e_enc_type;
    int  bitrate;
    int  width_override;
    int  height_override;
    int  preset_level;
    int  profile;
    int  iframeinterval;
    int  b_frames;
    std::string qp_range;
    int  rate_control;
    std::string level;
    std::string muxer;
    int EnableTwopassCBR;
    int vbv_size;
    int temporal_tradeoff;
    int insert_sps_pps;
    int alignment;
    int si_interval;
    std::string format;

    /* Output settings */
    bool b_output_stream;
    std::string target_ip;
    int  target_port;
    std::string output_file;

} enc_config_t;


typedef struct _stream_config_t_
{
    /* Src settings */
    int i_sensor_id;
    int i_min_fps;
    int camera_fps_override;
    int i_src_width;
    int i_src_height;
    ENC_VARIANT_E i_enc_var;
    int i_split_dur;
    int file_input;
    std::string filename;
    int railcar;
    int r_framerate, r_textoverlay;
    int s_framerate, s_textoverlay;
    std::string rgb_vp;
    std::string thr_vp;
    std::string rgb_rail;

    /* Encode settings */
    enc_config_t as_enc[3];

    /* Logging */
    std::string ac_log_file;
    int  i_log_level;

} stream_config_t;

void log_write(int i_level, const char *ac_format, ...);
void signal_setup();
void parse_config(stream_config_t *ps_stream_config, const std::string &conf_file, const std::string &cam_type);
int stream_init(stream_config_t *ps_stream);
void* stream_start(stream_config_t *ps_stream);
void stream_stop(void *pv_stream);

#endif /* _STREAM_H_ */
