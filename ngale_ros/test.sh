function test-gstreamer-v4l2() {
    # this one works
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR'! videoconvert ! filesink location="/tmp/a.mp4"

    # below also works, testing BGR to BGRx, RGBA, I420
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)60/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=BGRx,framerate=(fraction)60/1' ! videoconvert ! filesink location="/tmp/a.mp4"
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)60/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=RGBA,framerate=(fraction)60/1' ! nvvidconv ! filesink location="/tmp/a.mp4"
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)60/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=I420,framerate=(fraction)60/1' ! nvvidconv ! filesink location="/tmp/a.mp4"

    # downsample to 30 fps
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=NV12,framerate=(fraction)30/1' ! nvvidconv ! filesink location="/tmp/a.mp4"

    # rudimentary playbackable file
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=NV12,framerate=(fraction)30/1' ! nvvidconv ! omxh265enc ! mpegtsmux ! filesink location="/tmp/a.mp4"

    # with flip
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=NV12,framerate=(fraction)30/1' ! nvvidconv ! 'video/x-raw(memory:NVMM),flip-method=2' ! omxh265enc ! mpegtsmux ! filesink location="/tmp/a.mp4"

    # more detailed simulation
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=I420,framerate=(fraction)30/1' ! nvvidconv ! 'video/x-raw(memory:NVMM), flip-method=2' ! tee ! queue ! nvvidconv ! 'video/x-raw(memory:NVMM)' ! omxh265enc ! 'video/x-h265' ! mpegtsmux ! filesink location="/tmp/a.mp4"

    # resize works
    gst-launch-1.0 v4l2src device=/dev/video1 ! 'video/x-raw,width=640,height=512,format=BGR,framerate=(fraction)30/1'! videoconvert ! 'video/x-raw(ANY),width=640,height=512,format=I420,framerate=(fraction)30/1' ! nvvidconv ! 'video/x-raw(memory:NVMM), flip-method=2' ! tee ! queue ! nvvidconv ! 'video/x-raw(memory:NVMM),width=320,height=256' ! omxh265enc ! 'video/x-h265' ! mpegtsmux ! filesink location="/tmp/a.mp4"

   # for rgb detection stream simulation
   gst-launch-1.0 nvcamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=3840,height=2160,format=I420'! nvvidconv ! 'video/x-raw(memory:NVMM), flip-method=2' ! tee ! queue ! nvvidconv ! 'video/x-raw(memory:NVMM),width=864,height=480,format=I420' ! omxh265enc ! 'video/x-h265' ! mpegtsmux ! filesink location="/tmp/a.mp4"                   

}


