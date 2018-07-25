/* angle_detection
 * North is 0 degree
 * The x-axis of camera is heading toward west
 * The y-axis of camera is heading toward South
 * The z-axis of camera is heading toward Sky
 */
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <unistd.h>
#include "opencv2/core/utility.hpp"
#include <string>

using namespace cv;

double dist_p2l(Vec4f &pt1, CvPoint &pt2);

double angle_detection(Mat &img);
