#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "YoloLayer.h"

using namespace std;
using namespace Yolo;

struct Bbox
{
    int classId;
    int left;
    int right;
    int top;
    int bot;
    float score;
};

vector<float> prepareImage(cv::Mat& img, int c, int h, int w);
void DoNms(vector<Detection>& detections,int classes ,float nmsThresh);
vector<Bbox> postProcessImg(cv::Mat& img,vector<Detection>& detections,int classes, int h, int w);
vector<string> split(const string& str, char delim);

