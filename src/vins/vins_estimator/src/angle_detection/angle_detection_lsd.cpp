#include </home/caoxudong/catkin_ws/src/VINS-Mono/vins_estimator/src/angle_detection/angle_detection.h>

using namespace std;
using namespace cv;

double dist_p2l(Vec4f &pt1, CvPoint &pt2)
{
    //根据中心点与直线的距离 排除干扰直线
    //点(x0,y0)到直线Ax+By+C=0的距离为d = (A*x0+B*y0+C)/sqrt(A^2+B^2)
    double A,B,C,dist;
    //化简两点式为一般式
    //两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
    //化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
    //A = y2 - y1
    //B = x1 - x2
    //C = x2y1 - x1y2
    A = pt1[3] - pt1[1];
    B = pt1[0] - pt1[2];
    C = pt1[2] * pt1[1] - pt1[0] * pt1[3];
    //距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
    dist = abs(A * pt2.x + B * pt2.y + C) / sqrt(A * A + B * B);
    return dist;
}


/*(Hough Transform)*/
double angle_detection(Mat &img)
{
    Mat input,dst,grayImage;
    vector<Vec4f> lines_std;
    CvPoint corep;
    CvPoint line0,line1,linex,liney;
    vector<int> theta_H;
    vector<double>length_H;
    double heading;
    corep.x=0.5*img.cols;
    corep.y=0.5*img.rows;
theta_H.clear();
        theta_H.resize(180);//theta
        length_H.clear();
        length_H.resize(180);//length

input = img.clone();
        //cvtColor(input, grayImage, CV_BGR2GRAY);
        //dst = cvCreateImage( cvGetSize( input ), IPL_DEPTH_8U, 1 );
        blur(input, dst, Size(3, 3));
        Canny(dst,dst,20,60,3);
Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
        ls->detect(dst,lines_std);
ls->drawSegments(input, lines_std);
        for( int i = 0; i < lines_std.size(); i++ )  //lines存储的是直线
        {
            //CvPoint line[0] = (lines_std[0][0],

            //CvPoint* line = ( CvPoint* )cvGetSeqElem( lines_std, i );
            //  CvPoint* line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
            if(dist_p2l(lines_std[i], corep)>=100)//判断是否经过图像中心
                //            //cvSeqRemove(lines_std,i);
                    cout << "num     " << lines_std.size() << endl;
                //          else
            {


                line0.x = lines_std[i][0];
                line0.y = lines_std[i][1];
                line1.x = lines_std[i][2];
                line1.y = lines_std[i][3];

                double theta=180*atan2(line0.y-line1.y,line0.x-line1.x)/M_PI;
                double length=sqrt((line0.y-line1.y)*(line0.y-line1.y)+(line0.x-line1.x)*(line0.x-line1.x));
                if(length>50)
                {
                    line(input,line0,line1,Scalar(0,255,0),1,LINE_8);
                    if(theta<0)
                        theta=theta+180;
                    theta_H[round(theta)]++;
                    length_H[round(theta)]+=length;
                    //CvPoint linex=[];
                            //IplImage* color_dst = cvCreateImage( cvGetSize(img), 8, 3 );
              // cvCvtColor( dst, color_dst, CV_GRAY2BGR );
                    linex.x=cvRound(line0.x);
                    linex.y=cvRound(line0.y);
                    liney.x=cvRound(line1.x);
                    liney.y=cvRound(line1.y);
                    // const  CvArr* s=(CvArr*)&img;
//cv::inRange(img,cv::Scalar(20,100,100),cv::Scalar(30,255,255),redspace);
//                    cvLine( &img, linex, liney, CV_RGB(255,0,0), 3, 8 );

                }
            }
        }
        int no_first=0;
        int no_second=0;
        int th_first=0;
        int th_second=0;
        float r_first=0;
        float r_second=0;
        for(int i=0;i<180;i++)
        {
            if(no_first<theta_H[i])
            {
                no_second=no_first;
                r_second=r_first;
                th_second=th_first;
                no_first=theta_H[i];
                r_first=length_H[i];
                th_first=i;
            }
        }

        if(abs(th_first-th_second)>85&&abs(th_first-th_second)<95)
        {
            if(th_first>th_second)
            {
                th_first=th_first-90;
            }
            else{
                th_second=th_second-90;
            }
            heading=(r_first*th_first+r_second*th_second)/(r_first+r_second);
        }
        else
        {
            if(th_first>=90)
            {
                th_first=th_first-90;
                th_second=th_first;
                r_second=length_H[th_second];
            }
            else
            {
                th_first=th_first+90;
                th_second=th_first;
                r_second=length_H[th_second];
            }
            heading=(r_first*th_first+r_second*th_second)/(r_first+r_second);
        }

        //    cout<<"first_theta :"<<th_first<<", second_theta :"<<th_second<<endl;
        //    cout<<"first_length :"<<r_first<<", second_length :"<<r_second<<endl;

        cout << "  heading   " << heading << endl;
        imshow("Standard refinement",input);
        //imshow("last",img);
        waitKey(1);



    return heading;
}
