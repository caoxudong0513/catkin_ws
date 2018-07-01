#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);////判断点在图像内

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);//去除未跟踪到的点
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    vector<cv::Point2f> undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;//11.6cadd LK跟踪前后两帧，forw当前帧，cur上一帧，prev上一次发布的帧
    vector<cv::Point2f> n_pts;//! 除了KLT得到的Freatures之外，新提取的Features以满足最大特征点数目要求
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;//! 和图像image同理,cur_pts：上一帧特征点，forw_pts是当前帧特征点
    vector<int> ids;
    vector<int> track_cnt; //代表当前cur_ptrs被追踪的时间次数
    camodocal::CameraPtr m_camera;

    static int n_id;
};
