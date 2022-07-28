gst-launch-1.0 -e nvcamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1' ! \
  tee name=tee1 tee1. ! \
    queue ! nvvidconv ! \
    'video/x-raw(memory:NVMM), width=(int)3840, height=(int)2160, format=(string)I420, framerate=(fraction)30/1' ! \
    omxh264enc qp-range="-1,-1:-1,-1:-1,-1" control-rate=1 bitrate=4500000 \
               preset-level=0 EnableTwopassCBR=0 vbv-size=10 \
               temporal-tradeoff=0 iframeinterval=0 \
               profile=1 num-B-Frames=0 insert-sps-pps=1 ! \
    'video/x-h264, level=(string)5.2, stream-format=byte-stream, framerate=(fraction)30/1' ! \
    mpegtsmux  ! filesink location="/data/videos/rgb_video.mp4" \
  tee1. ! \
    queue ! nvvidconv ! 'video/x-raw, framerate=(fraction)30/1' ! videoscale ! \
    'video/x-raw, width=1280, height=720, format=I420' ! \
    omxh264enc control-rate=1 bitrate=350000 peak-bitrate=400000 preset-level=0 \
               qp-range=-1,-1:-1,-1:-1,-1 EnableTwopassCBR=0 \
               vbv-size=10 temporal-tradeoff=0 iframeinterval=30 \
               profile=2 num-B-Frames=0 insert-sps-pps=1 ! \
    'video/x-h264, level=(string)5.2, stream-format=byte-stream' ! \
    mpegtsmux si-interval=1000 alignment=7 ! \
    udpsink host=192.88.88.5 port=13001 max-bitrate=400000

gst-launch-1.0 v4l2src device=/dev/video1 ! \
  'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1' ! \
    queue ! videoconvert ! \
    'video/x-raw(ANY), width=(int)640, height=(int)512, format=(string)I420, framerate=(fraction)30/1' ! \
    queue ! nvvidconv flip-method=2 ! \
    'video/x-raw(memory:NVMM)' ! \
  tee name=tee1 tee1. ! \
    queue ! nvvidconv ! \
    'video/x-raw(memory:NVMM)' ! \
    omxh265enc qp-range="-1,-1:-1,-1:-1,-1" control-rate=1 bitrate=4500000 \
               preset-level=0 EnableTwopassCBR=0 vbv-size=10 \
               temporal-tradeoff=0 iframeinterval=0 \
               profile=1 num-B-Frames=0 insert-sps-pps=1 ! \
    'video/x-h264, level=(string)5.2, stream-format=byte-stream, framerate=(fraction)30/1' ! \
    mpegtsmux  ! filesink location="/data/videos/rgb_video.mp4" \
  tee1. ! \
    queue ! nvvidconv ! 'video/x-raw(ANY)' ! \
    omxh265enc control-rate=1 bitrate=180000 peak-bitrate=200000 preset-level=0 \
               qp-range=-1,-1:-1,-1:-1,-1 EnableTwopassCBR=0 \
               vbv-size=10 temporal-tradeoff=0 iframeinterval=30 \
               SliceIntraRefreshEnable=1 SliceIntraRefreshInterval=60 \
               bit-packetization=1 \
               profile=2 num-B-Frames=0 insert-sps-pps=1 ! \
    'video/x-h264, level=(string)5.2, stream-format=byte-stream' ! \
    mpegtsmux si-interval=1000 alignment=7 ! \
    udpsink host=192.88.88.5 port=14001 max-bitrate=200000

