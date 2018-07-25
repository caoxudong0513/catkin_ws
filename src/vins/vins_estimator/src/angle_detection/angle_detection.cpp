#include "angle_detection.h"

using namespace std;
using namespace cv;

double dist_p2l(CvPoint &pt1, CvPoint &pt2, CvPoint &pt3){
  //根据中心点与直线的距离 排除干扰直线
  //点(x0,y0)到直线Ax+By+C=0的距离为d = (A*x0+B*y0+C)/sqrt(A^2+B^2)
  double A,B,C,dist;
  //化简两点式为一般式
  //两点式公式为(y - y1)/(x - x1) = (y2 - y1)/ (x2 - x1)
  //化简为一般式为(y2 - y1)x + (x1 - x2)y + (x2y1 - x1y2) = 0
  //A = y2 - y1
  //B = x1 - x2
  //C = x2y1 - x1y2
  A = pt2.y - pt1.y;
  B = pt1.x - pt2.x;
  C = pt2.x * pt1.y - pt1.x * pt2.y;
  //距离公式为d = |A*x0 + B*y0 + C|/√(A^2 + B^2)
  dist = abs(A * pt3.x + B * pt3.y + C) / sqrt(A * A + B * B);
  return dist;
}
/*(Hough Transform)*/
double angle_detection(Mat &img)
{
    IplImage* src=0;
    IplImage input;
    CvSeq* lines = 0;
    CvMemStorage* storage = cvCreateMemStorage(0);
    IplImage* dst=0;
    CvPoint corep;
    vector<int> theta_H;
    vector<double>length_H;
    double heading;
    corep.x=0.5*img.cols;
    corep.y=0.5*img.rows;

    theta_H.clear();
    theta_H.resize(180);//theta
    length_H.clear();
    length_H.resize(180);//length

    input=IplImage(img);
    src=cvCloneImage(&input);
    dst = cvCreateImage( cvGetSize( src ), IPL_DEPTH_8U, 1 );

    cvCanny( src, dst, 20, 60, 3 );  //首先运行边缘检测，结果以灰度图显示（只有边缘）
    lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 30, 10 ); //直接得到直线序列

    //循环直线序列
    for( int i = 0; i < lines ->total; i++ )  //lines存储的是直线
    {
        CvPoint* line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
        if(dist_p2l(line[0], line[1], corep)<100)//判断是否经过图像中心
            cvSeqRemove(lines,i);
        else
        {
            double theta=180*atan2(line[0].y-line[1].y,line[0].x-line[1].x)/M_PI;
            double length=sqrt((line[0].y-line[1].y)*(line[0].y-line[1].y)+(line[0].x-line[1].x)*(line[0].x-line[1].x));
            if(theta<0)
                theta=theta+180;
            theta_H[round(theta)]++;
            length_H[round(theta)]+=length;
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

    for( int i = 0; i < lines ->total; i++ )  //lines存储的是直线
    {
        CvPoint* line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
        cvLine( src, line[0], line[1], CV_RGB( 0, 255, 0 ) );  //将找到的直线标记为绿色
    }
    cvNamedWindow( "Hough", 1 );
    cvShowImage( "Hough", src );
    waitKey(1);

    cvReleaseImage(&src);
    cvReleaseImage(&dst);
    src=0;
    dst=0;

    return heading;
}
/**/
/*   **********lsd*
double HeadingDetector(Mat &img)
{
  //    IplImage* src=0;
  //    IplImage input;
  //    CvSeq* lines = 0;
  //    CvMemStorage* storage = cvCreateMemStorage(0);
  //    IplImage* dst=0;
  Mat img,src,dst,grayImage;
  vector<Vec4f> lines_std;
  CvPoint corep;
  vector<int> theta_H;
  vector<double>length_H;
  double heading;
  corep.x=0.5*img.cols;
  corep.y=0.5*img.rows;

  theta_H.clear();
  theta_H.resize(180);//theta
  length_H.clear();
  length_H.resize(180);//length

  //    input=IplImage(img);
  //    src=cvCloneImage(&input);
  //    dst = cvCreateImage( cvGetSize( src ), IPL_DEPTH_8U, 1 );
  src = img.clone();
  cvtColor(src, grayImage, CV_BGR2GRAY);
  blur(grayImage, dst, Size(3, 3));
  Canny(dst,dst,50,200,3);

  Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_NONE);
  ls->detect(dst,lines_std);
//  cvCanny( src, dst, 50, 100, 3 );  //首先运行边缘检测，结果以灰度图显示（只有边缘）
//  lines = cvHoughLines2( dst, storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 80, 30, 10 ); //直接得到直线序列
  ls->drawSegments(src, lines_std);
  //循环直线序列
  for( int i = 0; i < lines_std.size(); i++ )  //lines存储的是直线
  {
    //line[0] = lines_std[0][]

     CvPoint* line = ( CvPoint* )cvGetSeqElem( lines_std, i );
  //  CvPoint* line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
    if(dist_p2l(line[0], line[1], corep)<100)//判断是否经过图像中心
      cvSeqRemove(lines,i);
    else
    {
      double theta=180*atan2(line[0].y-line[1].y,line[0].x-line[1].x)/M_PI;
      double length=sqrt((line[0].y-line[1].y)*(line[0].y-line[1].y)+(line[0].x-line[1].x)*(line[0].x-line[1].x));
      if(theta<0)
        theta=theta+180;
      theta_H[round(theta)]++;
      length_H[round(theta)]+=length;
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

  //    for( int i = 0; i < lines ->total; i++ )  //lines存储的是直线
  //    {
  //        CvPoint* line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
  //        cvLine( src, line[0], line[1], CV_RGB( 0, 255, 0 ) );  //将找到的直线标记为绿色
  //    }
  imshow("Standard refinement", src);
  waitKey(1);
  return heading;
}
*/
