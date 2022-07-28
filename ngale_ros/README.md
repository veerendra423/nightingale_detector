# SOW2 Milestone 2

## **Note**
Integrated SOW1 and SOW2 packages will be delivered in the next drop. A base framework to support RGB and THERMAL camera is currently provided, but for** SOW-2 usage is only focused on RGB**. Please ignore the THERMAL aspect of the till functionally integrated packages are provided. 
## **Prerequisites**

**ROS** must be installed and setup in the TX1 board. Catkin\_ws (catkin workspace) should be accessible. For installation of ROS follow [these instructions](https://www.jetsonhacks.com/2018/04/27/robot-operating-system-ros-on-nvidia-jetson-tx-development-kits/) . clone this [repo](https://github.com/jetsonhacks/installROSTX1) and follow the instructions as per the article.

## **ROS Node Design**

 The application consists of 4 ROS Nodes.

1. start\_client      : Start pipeline with options to enable rgb/thermal and detection.
2. stop\_client      : Stop pipeline .
3. listen\_rgb        : Receive and print RGB detection information .
4. ngale\_exe      : Publishes detection information form RGB and THERMAL

pipeline, provide service to start and stop pipeline.

## **How to Run the Application?**
**Step 1** : Download and unZip SOW2_MS2.ZIP on TX1 board.

**Step 2** : Copy the folder into " **< Your catkin\_ws path >/src/** ".

**Step 3** : From your terminal run " **source devel/setup.bash** ".

**Step 4** : run " **export ROS\_HOSTNAME=localhost** ".

**Step 5** : Open 5 other terminal  (one for each node) and repeat step 3 through step 4.

**Step 6** : In one of the terminals move into your catkin\_ws folder and run " **catkin\_make** ".

**Step 7** : Run " **roscore** " to execute ros engine.

**Step 8** : Update the **IP address** , **Port number** in " <Your catkin\_ws path\>/src/ngale\_ros/cfg/ " , config\_perf\_rgb\_h264.txt , config\_perf\_rgb\_h265.txt .

**Step 9 :** Run ROS node based on **'ROS NODES'** section below.


**Note :**

- Make sure OpenCV has Gstreamer enabled.


## **ROS Nodes**

### Ngale\_exe
  - **Command : rosrun ngale\_ros ngale\_exe**
  - Ngale\_exe should be the first ros Node you should run as it controls our entire application. Waits for calls from **start\_client and stop\_client.** Publish detection info to **listen\_rgb and listen\_thermal.**
  
### **Start\_client**
  - **Command : rosrun ngale\_ros start\_client <A1> <A2> <A3> <A4>**
    - **A1 -** Control RGB store and stream.
      	-  0 - Disable
      	-  264 - Encode using H264.
      	-  265 - Encode using H265

    - **A2 -** Control RGB Detection.
    		- 0 - Disable detection.
    		- 1 - Enable detection

    - **A3 -** Control THERMAL store and stream.
      		   -  0 - Disable

    - **A4 -** Control THERMAL Detection.
         		 - 0 - Disable detection.
         
- Start pipeline on ngale\_exe node.

### Stop\_client
  - **Command : rosrun ngale\_ros stop\_client**
  - Stop pipeline running on ngale\_exe node.

### Listen\_rgb
  - **Command : rosrun ngale\_ros listen\_rgb**
  - Print RGB detection

### Roscore
  - **Command : roscore**
  - It should be the first thing you. roscore should always run while running ros nodes and it starts the ros engine.

## **Order of calling ROS node**
1. roscore
2. rosrun ngale\_ros ngale\_exe
3. rosrun ngale\_ros start\_client < 0/264/265 > < 0/1 > < 0 > < 0 >
4. rosrun ngale\_ros listen\_rgb
5. rosrun ngale\_ros stop\_client

## **New feature controles**
1. Dumps Json file for detections:  ** write_json : 0/1 ** 
2. Enable Video input instead of camera input: **file_input : 0/1 ** 
3. Path to video input: **filename : < Path to H264 encoded Mp4 video >**;
4. Set the FPS for recording pipeline: **Record_framerate : 0/fps ;**
5. Enable Text overlay on recorded video: **Record_textoverlay : 0/1;**
6. Set the FPS for Stream pipeline: **Stream_framerate : 0/fps**
7. Enable Text overlay on video stream: **Stream_textoverlay : 0/1**

## **VIEW STREAM ON CLIENT**

- **X264 (Run on the client system)**

		gst-launch-1.0 -e udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay  ! h264parse ! queue ! avdec\_h264 !  fpsdisplaysink video-sink=xvimagesink sync=false async=false -e ``


- **X265 (Run on the client system)**

		gst-launch-1.0 -e udpsrc port=5000 ! application/x-rtp,encoding-name=H265,payload=96 ! rtph265depay ! h265parse ! queue ! avdec\_h265 ! fpsdisplaysink video-sink=xvimagesink sync=false async=false -e```


**Note** : Make sure the config files have updated values of features you want to enable especially port number and IP.
