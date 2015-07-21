
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>
#include <iostream>
#include <time.h>

using namespace std;
using namespace cv;

const int FRAME_WIDTH = 320;
const int FRAME_HEIGHT = 240;

static double angle(Point pt1, Point pt2, Point pt0)
{
  double dx1 = pt1.x - pt0.x;
  double dy1 = pt1.y - pt0.y;
  double dx2 = pt2.x - pt0.x;
  double dy2 = pt2.y - pt0.y;
  return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

template<typename T>
bool CheckCrossParams(vector<T> cross)
{
  for (int j = 0; j < cross.size() - 3; j++)
  {
    double ang1 = angle(cross[j], cross[j + 1], cross[j + 2]);
    double ang2 = angle(cross[j + 1], cross[j + 2], cross[j + 3]);
    if (ang1 > 0.7)
    {
      if (!(ang1 > 0.7 && ang2 > 0.3))
      {
        return false;
      }
    }
  }

  // проверка соотношений ширины/длины сторон креста
  int length_top = (abs(cross[0].x - cross[2].x) + abs(cross[0].y - cross[2].y))/2;
  int length_bot = (abs(cross[2].x - cross[3].x) + abs(cross[2].y - cross[3].y)) / 2;
  int length_left = (abs(cross[1].x - cross[2].x) + abs(cross[1].y - cross[2].y)) / 2;
  int length_right = (abs(cross[2].x - cross[4].x) + abs(cross[2].y - cross[4].y)) / 2;
  if (length_top == 0 || length_bot == 0 || length_left == 0 || length_right == 0) return false;
  printf("top: %d\t bot: %d\t left: %d\t right:%d \n", length_top, length_bot, length_left, length_right);

  double eps = 0.3;
  if ((length_top / length_bot) - 1  > eps) return false;
  if ((length_left / length_right) - 1 > eps) return false;

  double ratio1 = ((abs(length_top - length_bot) / length_top + abs(length_top - length_bot) / length_bot))/2;
  double ratio2 = ((abs(length_left - length_right) / length_left + abs(length_left - length_right) / length_right))/2;

  

  if (abs(ratio1 + ratio2) / 2  > 0.23)
  {
    return false;
  }
  return true;
}

int averageColor(Mat src, int x1, int x2, int y1, int y2){
  LineIterator it(src, Point(x1, y1), Point(x2, y2), 8);
  vector<Vec3b> buf(it.count);
  vector<Point> points(it.count);

  int avg_sum = 0;
  for (int i = 0; i < it.count; i++, ++it)
  {
    points[i] = it.pos();
    Vec3b colour = src.at<Vec3b>(points[i]);
    avg_sum += (colour.val[0] + colour.val[1] + colour.val[2]) / 3;
  }
  avg_sum = avg_sum / it.count;
  return avg_sum;
}

Point intersection(Point p1, Point p2, Point p3, Point p4) {

  double x1 = p1.x, x2 = p2.x, x3 = p3.x, x4 = p4.x;
  double y1 = p1.y, y2 = p2.y, y3 = p3.y, y4 = p4.y;

  double d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  // If d is zero, there is no intersection
  if (d == 0) return NULL;

  // Get the x and y
  double pre = (x1*y2 - y1*x2), post = (x3*y4 - y3*x4);
  double x = (pre * (x3 - x4) - (x1 - x2) * post) / d;
  double y = (pre * (y3 - y4) - (y1 - y2) * post) / d;

  // Check if the x and y coordinates are within both lines
  if (x < min(x1, x2) || x > max(x1, x2) ||
    x < min(x3, x4) || x > max(x3, x4)) return NULL;
  if (y < min(y1, y2) || y > max(y1, y2) ||
    y < min(y3, y4) || y > max(y3, y4)) return NULL;

  // Return the point of intersection
  Point ret;
  ret.x = x;
  ret.y = y;
  return ret;
}

RotatedRect rRect;
int aPDcoeff_int = 20;
double tresholdmin = 0.6;
int tresholdmin_int = 6;
int tresholdmax_int = 6;
int tresholdCannyMin = 1100;
int tresholdCannyMax = 1500;

int main()
{
  Mat src = imread("img1.jpg");
  if (src.empty())
    return -1;

  namedWindow("Control", CV_WINDOW_AUTOSIZE);
  createTrackbar("tresholdCannyMin", "Control", &tresholdCannyMin, 2000);
  createTrackbar("tresholdCannyMax", "Control", &tresholdCannyMax, 2000);
  createTrackbar("aPDcoeff", "Control", &aPDcoeff_int, 1000);

  VideoCapture capture;
  capture.open(0);

  capture.set(CV_CAP_PROP_FRAME_WIDTH, FRAME_WIDTH);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT);

  Mat display;
  vector<Mat> contours;
  vector<Point> approx;
  vector<Point>hull;
  vector<Point>cross (5);
  while (true){

    bool iscross = true;

    double aPDcoeff = (double)aPDcoeff_int / 1000;
    if (aPDcoeff <= 0) aPDcoeff = 0.01;
    capture.read(src);

    display = src.clone();

    line(src, Point(0, 0), Point(FRAME_WIDTH, 0), CV_RGB(255, 0, 0), 3);
    line(src, Point(FRAME_WIDTH, 0), Point(FRAME_WIDTH, FRAME_HEIGHT), CV_RGB(255, 0, 0), 3);
    line(src, Point(FRAME_WIDTH, FRAME_HEIGHT), Point(0, FRAME_HEIGHT), CV_RGB(255, 0, 0), 3);
    line(src, Point(0, FRAME_HEIGHT), Point(0, 0), CV_RGB(255, 0, 0), 3);

    Mat gray;
    Mat bw;
    Mat blurr;
    cvtColor(src, gray, CV_BGR2GRAY);

    blur(gray, gray, Point(3, 3));

    //threshold(gray, bw, tresholdCannyMin, tresholdCannyMax, THRESH_OTSU);

    Canny(gray, bw, tresholdCannyMin, tresholdCannyMax, 5);

    findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
      approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*aPDcoeff, true);

      if (fabs(contourArea(contours[i])) < 200 || isContourConvex(approx))
        continue;

      convexHull(Mat(contours[i]), hull, false);

      
      double ColoredPercentage = double(contourArea(contours[i])) / double(contourArea(hull));
      //printf("%f \n ", ColoredPercentage);
      if (ColoredPercentage > 0.35 || ColoredPercentage < 0.2) continue;

      

      //нахождение углов креста
      int max_x = 0;
      int max_y = 0;
      int min_x = FRAME_WIDTH + 1;
      int min_y = FRAME_HEIGHT + 1;
      int i1, i2, i3, i4;
      for (int i = 0; i < hull.size(); i++){
        if (hull[i].x > max_x) { max_x = hull[i].x; i1 = i; }
        if (hull[i].y > max_y) { max_y = hull[i].y; i2 = i; }
        if (hull[i].x < min_x) { min_x = hull[i].x; i3 = i; }
        if (hull[i].y < min_y) { min_y = hull[i].y; i4 = i; }
      }

      Point inter = intersection(Point(min_x, hull[i3].y), Point(max_x, hull[i1].y), Point(hull[i4].x, min_y), Point(hull[i2].x, max_y));

      cross[0] = Point(hull[i2].x, max_y);
      cross[1] = Point(min_x, hull[i3].y);
      cross[2] = inter;
      cross[3] = Point(hull[i4].x, min_y);
      cross[4] = Point(max_x, hull[i1].y);

      if (!CheckCrossParams(cross)) continue;

      line(display, Point(min_x, hull[i3].y), Point(max_x, hull[i1].y), Scalar(255, 0, 0));
      line(display, Point(hull[i4].x, min_y), Point(hull[i2].x, max_y), Scalar(255, 0, 0));

      circle(display, inter, 3, Scalar(0, 0, 255), -1, 8, 0);

      int k = 0;
      for (k = 0; k < hull.size() - 1; k++){
        line(display, hull[k], hull[k + 1], CV_RGB(0, 255, 0), 1);
        string s = to_string(k);
      }
      line(display, hull[k], hull[0], CV_RGB(0, 255, 0), 1);

    }
    


    imshow("display", display);
    imshow("bw", bw);

    waitKey(1);
  }
  getchar();
  return 0;
}