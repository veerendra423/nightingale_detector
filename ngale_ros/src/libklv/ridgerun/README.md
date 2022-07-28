# Previous instructions to build are no longer being used rather we've built the required libraries from ridgerun source code for now
# these libraries may not be needed in future once we move to complete appsrc instead of using metasrc for inseting KLV metadata packets

# Copy prebuilt libraries for Tx1 to  

sudo cp libs/libgstmeta.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstmeta.so

sudo cp libs/libgstbase-1.0.so.0.803.0 /usr/lib/aarch64-linux-gnu/libgstbase-1.0.so.0.803.0
sudo cp libs/libgstcodecparsers-1.0.so.0.803.0 /usr/lib/aarch64-linux-gnu/libgstcodecparsers-1.0.so.0.803.0
sudo cp libs/libgstmpegts-1.0.so.0.803.0 /usr/lib/aarch64-linux-gnu/libgstmpegts-1.0.so.0.803.0

sudo cp libs/libgstmpegpsdemux.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstmpegpsdemux.so
sudo cp libs/libgstmpegtsmux.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstmpegtsmux.so
sudo cp libs/libgstmpegtsdemux.so /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstmpegtsdemux.so
 

## License

This source code is pulled from ridgerun github. please go thru ridgerun LICENSES.txt
