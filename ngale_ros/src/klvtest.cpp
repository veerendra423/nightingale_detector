#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <gst/gst.h>
#include <gst/base/gstbasetransform.h>
#include <gst/base/gstbytewriter.h>
//#include "libklv/include/Klv.h"
//#include <gst/app/gstappsrc.h>

static GMainLoop *loop;
static GstElement *pipeline, *appsrc, *src, vidrate, *tee, *encoder, *muxer, *filesink, *videoconvert_record, *videoconvert_display, *videosink, *queue, *queue_record, *queue_klv, *queue_display, *caps_filter, *capsfilter_klv, *capsfilter_video;
static GstBus *bus;

/* KLV data from Day_Flight.mpg */
static const guint8 rtp_KLV_frame_data[] = {
  0x06, 0x0e, 0x2b, 0x34, 0x02, 0x0b, 0x01, 0x01,
  0x0e, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00,
  0x81, 0x91, 0x02, 0x08, 0x00, 0x04, 0x6c, 0x8e,
  0x20, 0x03, 0x83, 0x85, 0x41, 0x01, 0x01, 0x05,
  0x02, 0x3d, 0x3b, 0x06, 0x02, 0x15, 0x80, 0x07,
  0x02, 0x01, 0x52, 0x0b, 0x03, 0x45, 0x4f, 0x4e,
  0x0c, 0x0e, 0x47, 0x65, 0x6f, 0x64, 0x65, 0x74,
  0x69, 0x63, 0x20, 0x57, 0x47, 0x53, 0x38, 0x34,
  0x0d, 0x04, 0x4d, 0xc4, 0xdc, 0xbb, 0x0e, 0x04,
  0xb1, 0xa8, 0x6c, 0xfe, 0x0f, 0x02, 0x1f, 0x4a,
  0x10, 0x02, 0x00, 0x85, 0x11, 0x02, 0x00, 0x4b,
  0x12, 0x04, 0x20, 0xc8, 0xd2, 0x7d, 0x13, 0x04,
  0xfc, 0xdd, 0x02, 0xd8, 0x14, 0x04, 0xfe, 0xb8,
  0xcb, 0x61, 0x15, 0x04, 0x00, 0x8f, 0x3e, 0x61,
  0x16, 0x04, 0x00, 0x00, 0x01, 0xc9, 0x17, 0x04,
  0x4d, 0xdd, 0x8c, 0x2a, 0x18, 0x04, 0xb1, 0xbe,
  0x9e, 0xf4, 0x19, 0x02, 0x0b, 0x85, 0x28, 0x04,
  0x4d, 0xdd, 0x8c, 0x2a, 0x29, 0x04, 0xb1, 0xbe,
  0x9e, 0xf4, 0x2a, 0x02, 0x0b, 0x85, 0x38, 0x01,
  0x2e, 0x39, 0x04, 0x00, 0x8d, 0xd4, 0x29, 0x01,
  0x02, 0x1c, 0x5f
};

/*
static const guint8 rtp_KLV_frame_data[] = {
  0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01, 
  0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 
  0x81, 0x90, 0x02, 0x08, 0x00, 0x04, 0x6C, 0xAE, 
  0x70, 0xF9, 0x80, 0xCF, 0x41, 0x01, 0x01, 0x05, 
  0x02, 0xE1, 0x91, 0x06, 0x02, 0x06, 0x0D, 0x07, 
  0x02, 0x0A, 0xE1, 0x0B, 0x02, 0x49, 0x52, 0x0C, 
  0x0E, 0x47, 0x65, 0x6F, 0x64, 0x65, 0x74, 0x69, 
  0x63, 0x20, 0x57, 0x47, 0x53, 0x38, 0x34, 0x0D, 
  0x04, 0x4D, 0xCC, 0x41, 0x90, 0x0E, 0x04, 0xB1, 
  0xD0, 0x3D, 0x96, 0x0F, 0x02, 0x1B, 0x2E, 0x10, 
  0x02, 0x00, 0x84, 0x11, 0x02, 0x00, 0x4A, 0x12, 
  0x04, 0xE7, 0x23, 0x0B, 0x61, 0x13, 0x04, 0xFD, 
  0xE8, 0x63, 0x8E, 0x14, 0x04, 0x03, 0x0B, 0xC7, 
  0x1C, 0x15, 0x04, 0x00, 0x9F, 0xB9, 0x38, 0x16, 
  0x04, 0x00, 0x00, 0x01, 0xF8, 0x17, 0x04, 0x4D, 
  0xEC, 0xDA, 0xF4, 0x18, 0x04, 0xB1, 0xBC, 0x81, 
  0x74, 0x19, 0x02, 0x0B, 0x8A, 0x28, 0x04, 0x4D, 
  0xEC, 0xDA, 0xF4, 0x29, 0x04, 0xB1, 0xBC, 0x81, 
  0x74, 0x2A, 0x02, 0x0B, 0x8A, 0x38, 0x01, 0x31, 
  0x39, 0x04, 0x00, 0x9F, 0x85, 0x4D, 0x01, 0x02, 
  0xB7, 0xEB };
*/

int vid_frame_counter = 1;
int klv_frame_counter = 0;
GstClockTime timestamp = 0;

static gboolean message_cb (GstBus * bus, GstMessage * message, gpointer user_data)
{
  
  gchar *name = NULL;
  name = gst_object_get_path_string (message->src);
  printf("received message: %d from source: %s\n", GST_MESSAGE_TYPE (message), name);

  switch (GST_MESSAGE_TYPE (message)) {
    case GST_MESSAGE_ERROR:{
      g_main_loop_quit (loop);

      GError *err = NULL;
      gchar *name, *debug = NULL;
      name = gst_object_get_path_string (message->src);
      gst_message_parse_error (message, &err, &debug);

      std::string err_message = err->message;
      if(err_message == "Output window was closed"){
        std::cout << std::endl << err_message << std::endl;
      }
      else{
        g_printerr ("\nERROR: from element %s: %s\n", name, err->message);
        if (debug != NULL)
          g_printerr ("Additional debug info:\n%s\n", debug);
      }

      g_error_free (err);
      g_free (debug);
      g_free (name);
      break;
    }
    case GST_MESSAGE_WARNING:{
      GError *err = NULL;
      gchar *name, *debug = NULL;

      name = gst_object_get_path_string (message->src);
      gst_message_parse_warning (message, &err, &debug);

      g_printerr ("ERROR: from element %s: %s\n", name, err->message);
      if (debug != NULL)
        g_printerr ("Additional debug info:\n%s\n", debug);

      g_error_free (err);
      g_free (debug);
      g_free (name);
      break;
    }
    case GST_MESSAGE_EOS: {
      g_print ("Got EOS\n");
      g_main_loop_quit(loop);
      break;
    }
    default:
    break;
  }

  return TRUE;
}

// This will need clean-up using LDS/KLV muxer library
static GstStaticCaps unix_reference = GST_STATIC_CAPS ("timestamp/x-unix");

gsize inject_klv_meta_data (GstBuffer *buf)
{
/* Add KLV meta for testing, here: Motion Imagery Standards Board (MISB)
 * Engineering Guideline MISB std 601
 */
  const guint8 klv_header[16] = { 0x06, 0x0e, 0x2b, 0x34, 0x02, 0x0b, 0x01,
    0x01, 0x0e, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00
  };
  GstByteWriter *bw;
  /* NOTE: MISB defines MISP time, which is NOT UTC, but use UTC for now */
  guint64 utc_us = -1;
  guint8 metasize = 59; // fixed size for now UAS LDS Key + lenght (1 byte) + value (42 bytes) 

#if GST_CHECK_VERSION(1,14,0)
  GstReferenceTimestampMeta *time_meta;
  time_meta =
      gst_buffer_get_reference_timestamp_meta (buf,gst_static_caps_get (&unix_reference));
  if (time_meta) {
    utc_us = time_meta->timestamp / 1000;
  }
#endif

  if (utc_us == -1) {
    GDateTime *dt = g_date_time_new_now_utc ();
    utc_us = g_date_time_to_unix (dt) * 1000000;        /* microseconds */
    utc_us += g_date_time_get_microsecond (dt);
    g_date_time_unref (dt);
  }

  //gst_byte_writer_init (&bw);
  //bw = gst_byte_writer_new ();
  /* KLV header */
  gst_byte_writer_put_data (bw, klv_header, 16);

  /* Total length in BER long form (verbatim as long as <= 127) */
  gst_byte_writer_put_uint8 (bw,
      (1 + 1 + 8) + (1 + 1 + 14) + (1 + 1 + 4) + (1 + 1 + 4) + (1 + 1 + 2));

  /* Tag 1: unix timestamp */
  gst_byte_writer_put_uint8 (bw, 2);   /* local tag: unix timestamp    */
  gst_byte_writer_put_uint8 (bw, 8);   /* data length (BER short form) */
  gst_byte_writer_put_uint64_be (bw, utc_us);

  /* Tag 2: Image Coordinate System */
  gst_byte_writer_put_uint8 (bw, 12);  /* local tag: unix timestamp    */
  gst_byte_writer_put_uint8 (bw, 14);  /* data length (BER short form) */
  gst_byte_writer_put_data (bw, (guint8 *) "Geodetic WGS84", 14);

  /* Tag 3: Sensor Latitude */
  gst_byte_writer_put_uint8 (bw, 13);  /* local tag: latitude */
  gst_byte_writer_put_uint8 (bw, 4);   /* data length (BER short form) */
  {
    /* Map -(2^31-1)..(2^31-1) to +/-90 with 0x80000000 = error */
    gdouble latitude = 51.449825;
    gint32 val = (gint) ((latitude / 90.0) * 2147483647.0);
    gst_byte_writer_put_int32_be (bw, val);
  }

  /* Tag 4: Sensor Longitude */
  gst_byte_writer_put_uint8 (bw, 14);  /* local tag: longitude */
  gst_byte_writer_put_uint8 (bw, 4);   /* data length (BER short form) */
  {
    /* Map -(2^31-1)..(2^31-1) to +/-180 with 0x80000000 = error */
    gdouble longitude = -2.600439;
    gint32 val = (gint) ((longitude / 180.0) * 2147483647.0);
    gst_byte_writer_put_int32_be (bw, val);
  }

  /* Tag 5: Sensor True Altitude (MSL) (elevation) */
  gst_byte_writer_put_uint8 (bw, 15);  /* local tag: elevation */
  gst_byte_writer_put_uint8 (bw, 2);   /* data length (BER short form) */
  {
    /* Map 0..(2^16-1) to -900..19000 meters, so resolution 0.303654536m */
    gdouble elevation = 10.0;
    guint16 val = ((elevation + 900.0 + 0.151827268) / 19900.0) * 65535.0;
    gst_byte_writer_put_int16_be (bw, val);
  }

  {
    gsize klv_size = gst_byte_writer_get_size (bw);
    //guint8 *klv_data = gst_byte_writer_free_and_get_data (bw);
    metasize = klv_size;
    //metasize = gst_buffer_fill(buf, 0, &klv_data, klv_size);
    //gst_buffer_add_klv_meta_take_data (buf, klv_data, klv_size);
  }

  return metasize;
}

void need_data_cb(GstElement* element, guint arg0, gpointer userData){
  //std::cout << "need_data" << std::endl;

  // Needed for appsrc->push_buffer data in Big-Endian.
  //static GstClockTime timestamp = 0;
  //GstMapInfo map;
  //GstClock *clock = NULL;
  //GstClockTime base_time, now;
  //GstBuffer *buffer = gst_buffer_new();

  GstBuffer *buffer =  gst_buffer_new_allocate(NULL, 163, NULL);
  gsize buf_size=0, klv_size = (gsize)sizeof(rtp_KLV_frame_data);
  //printf("rtp_KLV_frame_data_size:%d\n", rtp_KLV_frame_data_size);

  //gst_buffer_map(buffer, &map, GST_MAP_WRITE);
  //map.data = (guint8*)g_memdup(rtp_KLV_frame_data, rtp_KLV_frame_data_size);
  //map.size = rtp_KLV_frame_data_size;
  //gst_buffer_unmap (buffer, &map);
  
  buf_size = gst_buffer_fill(buffer, 0, (guint8 *)&rtp_KLV_frame_data, klv_size);
  //buf_size = inject_klv_meta_data (buffer);

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 30);


  GstFlowReturn ret;
  g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
  //g_signal_emit_by_name (appsrc, "end-of-stream", &ret);

  klv_frame_counter += 1;
  timestamp += GST_BUFFER_DURATION (buffer);

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    printf("KLV_frame_insertion_failed:%d\n", klv_frame_counter);
    gst_buffer_unref (buffer);
    g_main_loop_quit (loop);
  }
  printf("KLV_frame_insertion_successful:%d  -%" GST_TIME_FORMAT "\n", klv_frame_counter, GST_TIME_ARGS(timestamp));
  //if (klv_frame_counter > 30 ) {
  //   g_signal_emit_by_name (appsrc, "end-of-stream", &ret);
  //}
}


static void
need_frame (GstElement *appsrc,
          guint       unused_size,
          gpointer    user_data)
{
  static gboolean white = FALSE;
  GstBuffer *buffer;
  guint size;
  GstFlowReturn ret;

  printf("pushing video frame:%d\n", vid_frame_counter);
  size = 388 * 285 * 2;

  buffer = gst_buffer_new_allocate (NULL, size, NULL);

  /* this makes the image black/white */
  gst_buffer_memset (buffer, 0, white ? 0xff : 0x0, size);

  white = !white;

  GST_BUFFER_PTS (buffer) = timestamp;
  GST_BUFFER_DTS (buffer) = timestamp;
  GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 1);

  g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);

  if (ret != GST_FLOW_OK) {
    /* something wrong, stop pushing */
    printf("video_frame_insertion_failed:%d\n", vid_frame_counter);
    gst_buffer_unref (buffer);
    g_main_loop_quit (loop);
  }
  vid_frame_counter++;
  timestamp += GST_BUFFER_DURATION (buffer);
  
  if (vid_frame_counter > 30) {
     g_signal_emit_by_name (src, "end-of-stream", &ret);
     g_signal_emit_by_name (appsrc, "end-of-stream", &ret);

  }

}


void sigintHandler(int unused)
{
  g_print("You ctrl-c-ed!");
  gst_element_send_event(pipeline, gst_event_new_eos());
}
int main(int argc, char *argv[])
{
  signal(SIGINT, sigintHandler);
  gst_init (&argc, &argv);

  guint major, minor, micro, nano;
  gst_version (&major, &minor, &micro, &nano);
  std::string nano_str = nano==1 ? "(CVS)" : (nano==2 ? "(Prerelease)" : "");
  std::cout << "This program is linked against GStreamer " << major << "." << minor<< "." << micro << " " << nano_str << std::endl;


  pipeline = gst_pipeline_new(NULL);
  appsrc = gst_element_factory_make ("appsrc", NULL);
  //src = gst_element_factory_make("appsrc", NULL);
  src = gst_element_factory_make("v4l2src", NULL);
  //vidrate = gst_element_factory_make("videorate", NULL);
  tee = gst_element_factory_make("tee", "tee");
  encoder = gst_element_factory_make("x264enc", NULL);
  muxer = gst_element_factory_make("mpegtsmux", NULL);
  filesink = gst_element_factory_make("filesink", NULL);
  videoconvert_record = gst_element_factory_make("videoconvert", NULL);
  videoconvert_display = gst_element_factory_make("videoconvert", NULL);
  videosink = gst_element_factory_make("autovideosink", NULL);
  queue = gst_element_factory_make("queue", "NULL");
  queue_display = gst_element_factory_make("queue", "queue_display");
  queue_record = gst_element_factory_make("queue", "queue_record");
  queue_klv = gst_element_factory_make("queue", "queue_klv");
  caps_filter = gst_element_factory_make("capsfilter", NULL);
  capsfilter_klv = gst_element_factory_make("capsfilter", NULL);
  capsfilter_video = gst_element_factory_make("capsfilter", NULL);

  if (!pipeline || !appsrc || !src || !tee || !encoder || !muxer || !filesink || !videoconvert_record|| !videoconvert_display || !videosink || !queue || !queue_record || !queue_display || !queue_klv || !caps_filter || !capsfilter_klv || !capsfilter_video) {
    std::cerr << "Failed to create elements:"
    << (!pipeline? " pipeline " : "")
    << (!appsrc? " appsrc " : "")
    << (!src? " v4l2src " : "")
    << (!tee? " tee " : "")
    << (!encoder? " x264enc " : "")
    << (!muxer? " mpegtsmux " : "")
    << (!filesink? " filesink " : "")
    << (!videoconvert_record? " videoconvert " : "")
    << (!videoconvert_display? " videoconvert " : "")
    << (!videosink? " autovideosink " : "")
    << (!queue? " queue " : "")
    << (!queue_display? " queue_display " : "")
    << (!queue_record? " queue_record " : "")
    << (!caps_filter? " capsfilter " : "")
    << (!capsfilter_klv? " capsfilter " : "")
    << (!capsfilter_video? " capsfilter " : "")
    << std::endl;
    return -1;
  }
  
  // Create caps for video streaming thru v4l2src
  GstCaps *video_cap = gst_caps_new_simple ("video/x-raw",
                    "width", G_TYPE_INT, 640,
                    "height", G_TYPE_INT, 480,
                    "framerate", GST_TYPE_FRACTION, 30/1, 1, 
                    "is_live", G_TYPE_BOOLEAN, TRUE, 
                    "do-timestamp", G_TYPE_BOOLEAN, TRUE,
                    NULL);


  // Create caps for video appsrc frame by frame
/*  GstCaps *video_cap = gst_caps_new_simple ("video/x-raw",
                     "format", G_TYPE_STRING, "RGB16",
                     "width", G_TYPE_INT, 388,
                     "height", G_TYPE_INT, 285,
                     "framerate", GST_TYPE_FRACTION, 1/1, 1,
                     NULL);
*/

  // Create caps for video streaming thru v4l2src
  GstCaps *klv_caps = gst_caps_new_simple("meta/x-klv", 
                         "parsed", G_TYPE_BOOLEAN, TRUE, 
                         "is_live", G_TYPE_BOOLEAN, TRUE, 
                         nullptr);

  // Create caps for video appsrc frame by frame
/*  GstCaps *klv_caps = gst_caps_new_simple("meta/x-klv", 
   		         "parsed", G_TYPE_BOOLEAN, TRUE, 
   			 "sparse", G_TYPE_BOOLEAN, TRUE, 
   			 nullptr);
*/
  GstCaps *caps = gst_caps_new_full (
                     gst_structure_new ("video/x-raw",
                             "width", G_TYPE_INT, 640,
                             "height", G_TYPE_INT, 480,
                             "framerate", GST_TYPE_FRACTION, 30/1, 1,
                              NULL),
                     gst_structure_new ("meta/x-klv",
                             "parsed", G_TYPE_BOOLEAN, TRUE,
                             "sparse", G_TYPE_BOOLEAN, TRUE,
                              NULL),
                      NULL);

  g_object_set(appsrc, "caps", klv_caps, nullptr);
  g_object_set(appsrc, "format", GST_FORMAT_TIME, nullptr);
  g_object_set(appsrc, "stream-type", 0, nullptr);
  g_object_set(appsrc, "do-timestamp", 1, nullptr);
  //g_object_set(appsrc, "max-bytes", 163, nullptr);

  /* setup for video v4l2src */
  g_object_set (src, "device", "/dev/video1", NULL);

  /* setup for video appsrc */
  //g_object_set (G_OBJECT (src), "format", GST_FORMAT_TIME, NULL);
  //g_object_set(G_OBJECT(src), "stream-type", 0, nullptr);

  g_object_set (encoder, "noise-reduction", 1000, NULL);
  g_object_set (encoder, "bitrate", 90000, NULL);
  g_object_set (encoder, "threads", 4, NULL);
  g_object_set (encoder, "byte-stream", TRUE, NULL);
  g_object_set(filesink, "location", "klvtest.ts", NULL);
  g_object_set(filesink, "async", 0, NULL);
  g_object_set(caps_filter, "caps", caps, NULL);
  g_object_set(capsfilter_klv, "caps", klv_caps, NULL);
  g_object_set(capsfilter_video, "caps", video_cap, NULL);
  gst_caps_unref(caps);
  gst_caps_unref(video_cap);
  gst_caps_unref(klv_caps);

  // add all gst elements to bin
  gst_bin_add_many(GST_BIN(pipeline), src, caps_filter, capsfilter_klv, capsfilter_video, tee, queue_record, encoder, muxer, filesink, queue, queue_klv, queue_display, videoconvert_record, videoconvert_display, videosink, appsrc, NULL);

  //create the pipeline
  if (!gst_element_link_many(src, capsfilter_video, queue, tee, NULL) || 
      !gst_element_link_many(tee, videoconvert_record, queue_record, encoder, muxer, filesink, NULL) || 
      !gst_element_link_many( appsrc, queue_klv, muxer, NULL) ||
      !gst_element_link_many(tee, videoconvert_display, queue_display, videosink, NULL) ) {
    g_error("Failed to link elements");
    return -2;
  }


  //register app callbacks
  bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));
  gst_bus_add_signal_watch(bus);
  g_signal_connect(G_OBJECT(bus), "message", G_CALLBACK(message_cb), NULL);
  gst_object_unref(GST_OBJECT(bus));

  g_signal_connect(appsrc, "need-data", G_CALLBACK(need_data_cb), NULL);
  
  /* setup for video appsrc */
  //g_signal_connect (src, "need-data", G_CALLBACK (need_frame), NULL);

  //start pipeline
  gst_element_set_state(pipeline, GST_STATE_PLAYING);
  if (gst_element_get_state (pipeline, NULL, NULL, -1) == GST_STATE_CHANGE_FAILURE) {
    g_error("KLVTEST Failed to go into PLAYING state");
  }


  loop = g_main_loop_new(NULL, FALSE);
  g_print("Starting loop");
  g_main_loop_run(loop);
  
  // stop pipeline 
  gst_element_set_state (pipeline, GST_STATE_NULL);
  //
  // Free resources
  gst_object_unref (bus);
  gst_object_unref (pipeline);

  return 0;
}
