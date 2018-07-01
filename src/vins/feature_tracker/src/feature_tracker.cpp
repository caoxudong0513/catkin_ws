#include "feature_tracker.h"

int FeatureTracker::n_id = 0;

/**
 * @breif 判断特征点是非在范围内
 * * [inBorder 判断该Feature是否在图像上]
 * @param  pt [Feature坐标]
 * @return    [description]
*/
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * @breif 去除无法追踪的特征
 * * [reduceVector 根据status的状态，剔除tracking失败的点]
 * @param v      [description]
 * @param status [description]
*/
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

/**
 * @breif 去除无法追踪的特征
 *  * [reduceVector 根据status的状态，剔除tracking失败的点]
 * @param v      [description]
 * @param status [description]
*/
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
}


/**
 * @brief 对图像使用光流法进行特征点跟踪
 *
 * 按照被追踪到的次数排序，然后加上mask去掉部分点，主要是针对鱼眼相机
 *  * [FeatureTracker::setMask 将Features1根据跟踪次数大小排序，并且设置后续提取强角点的Region]
*/
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    

    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    //! 因为刚开始的时候track_cnt的大小和Max_Features是相等的
    for (unsigned int i = 0; i < forw_pts.size(); i++)//对检测到的特征点按追踪到的次数排序
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    //! 根据跟踪效果对Features排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

      //! 清除当前帧的所有Features，id及跟踪次数
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
//在mask图像中将追踪到点的地方设置为0，否则为255，为了下面做特征点检测时可选择没有特征点的区域进行检测。同一区域内，追踪到次数最多的点保留其他的点会被删除
        //! 将mask中以Features为圆心的，MIN_DIST为半径的区域全部置0，在后面就不在这些区域中选取强角点了。
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

/**
 * @brief 添加新的特征点
 *  * [FeatureTracker::addPoints 将当前图像中选取的强角点加入到tracking的Features中，以满足最大Features的数目]
*/
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);
        track_cnt.push_back(1);
    }
}

/**
 * @brief 对新来的图像使用光流法进行特征点跟踪
 *
 * optional: 使用createCLAHE对图像进行自适应直方图均衡化
 * calcOpticalFlowPyrLK() LK金字塔光流法
 * 设置mask，在setMask()函数中
 * goodFeaturesToTrack()添加一些特征点，确保每帧都有足够的特征点
 * addPoints()添加新的追踪点
*/
void FeatureTracker::readImage(const cv::Mat &_img)
{
    cv::Mat img;
    TicToc t_r;

    //! Step1：对原始图像做增强
    if (EQUALIZE)//直方图均衡化
    {
      //使用createCLAHE对图像进行自适应直方图均衡化//! 利用自适应直方图均衡算法对图像做增强处理
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        TicToc t_c;
        clahe->apply(_img, img);
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

     //! Step2: 读入增强后的图像
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img;
    }

    forw_pts.clear();

     //! Step3: 利用LKT算法对Features进行tracking
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status;
        vector<float> err;

        //! Step3.1:进行LKT跟踪
        //calcOpticalFlowPyrLK() LK金字塔光流法
        //ref: https://docs.opencv.org/3.1.0/d7/d8b/tutorial_py_lucas_kanade.html
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
        //track cur_pts~forw_pts,根据status,剔除跟踪失败的点prev, cur, forw, ids, track_cnt,
        //清理vector中无法追踪到的特征点
        //! Step3.2: 剔除跟踪后不在图像之内的Feature的状态标志位
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))//剔除跟踪到图像边缘的点inBorder
                status[i] = 0;
         //! Step3.3:剔除跟踪失败的Features
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
             //！保证了track_cnt的大小和forw_pts是是一致的
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }
//如果不需要发布特征点,则到这步就完了,把当前帧forw赋给上一帧cur, 然后退出
    if (PUB_THIS_FRAME)//发布特征点
    {
       //! Step4: 计算F矩阵，剔除外点
        rejectWithF();//prev_pts和forw_pts做ransac剔除outlier,(实际就是调用了findFundamentalMat函数)
//完了以后, 剩下的点track_cnt都加1.
        //在光流追踪成功就记被追踪+1，数值代表被追踪的次数，数值越大，说明被追踪的就越久
        //! 将满足经过KLT跟踪、在图像内及没有被ransac剔除三个条件的Features的跟踪次数加1
        for (auto &n : track_cnt)
            n++;

        ROS_DEBUG("set mask begins");
        TicToc t_m;
        //为下面的goodFeaturesToTrack保证相邻的特征点之间要相隔30个像素,设置mask image
        //！Step5: 设置要提取强角点的区域
        //
        setMask();
//跟踪点forw_pts按跟踪次数降排序, 然后依次选点, 选一个点, 在mask中将该点周围一定半径的区域设为0, 后面不再选取该区域内的点. 有点类似与non-max suppression, 但区别是这里保留track_cnt最高的点.
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        ROS_DEBUG("detect feature begins");
        TicToc t_t;
         //! Step6：若跟踪的Features未达到最大值，则另外选取一些强角点
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            //! 在上一幅图像中选取MAX_CNT - forw_pts.size()质量最高的点，以达到tracking过程中最大角点个数的要求
            //上面通过光流法找到一些对应点，这里是为了确保每个帧有足够点，然后调用addPoint添加点
//在mask中不为0的区域,调用goodFeaturesToTrack提取新的角点n_pts, 通过addPoints()push到forw_pts中, id初始化-1, track_cnt初始化为1
//完了以后,在外面主回调函数中调用updateID()来更新那些新特征点的ID
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.1, MIN_DIST, mask);
        }
        else
            n_pts.clear();
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        ROS_DEBUG("add feature begins");
        TicToc t_a;
        //！Step6：将选取的强角点加入到tracking的Featues中
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());

        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;
}


/**
 * @breif 通过前后两帧的追踪计算F矩阵，通过F矩阵去除Outliers
*/
void FeatureTracker::rejectWithF()//通过F矩阵去除外点
{
    if (forw_pts.size() >= 8)
    {
        //! Step4.1: 将特征点经过畸变矫正后再投影到图像平面上
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        //! Step4.2: 通过计算F矩阵，得到内外点
        vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
          //! Step4.3: 将Features中的外点剔除
        int size_a = prev_pts.size();
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

/**
 * @breif 更新ID
 *  * @param  i [description]
 * @return   [description]
*/
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
      //! 新提取的Feature
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

/**
 * @breif Visualize undistortedPoints Points
 * [FeatureTracker::showUndistortion 显示未畸变的图像]
 * @param name [description]
*/
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

/**
 * @breif undistortedPoints Points
 * 将图像坐标转到归一化平面上，并进行畸变校正
*/
vector<cv::Point2f> FeatureTracker::undistortedPoints()//将所有特征点转换到一个归一化平面并且进行畸变
{
    vector<cv::Point2f> un_pts;
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }

    return un_pts;
}
