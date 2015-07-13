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


RotatedRect rRect;
int aPDcoeff_int = 20;
double tresholdmin = 0.6;
int tresholdmin_int = 6;
int tresholdmax_int = 6;
int tresholdCannyMin = 1400;
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
	bool isfinish = true;
	while (isfinish){

		bool iscross = true;

		double aPDcoeff = (double)aPDcoeff_int / 1000;
		if (aPDcoeff <= 0) aPDcoeff = 0.01;
		capture.read(src);

		display= src.clone();

		Mat gray;
		cvtColor(src, gray, CV_BGR2GRAY);

		Mat bw;
		Canny(gray, bw, tresholdCannyMin, tresholdCannyMax, 5);


		line(bw, Point(0, 0), Point(FRAME_WIDTH, 0), CV_RGB(255, 0, 0), 2);
		line(bw, Point(FRAME_WIDTH, 0), Point(FRAME_WIDTH - 1, FRAME_HEIGHT - 1), CV_RGB(255, 0, 0), 2);
		line(bw, Point(FRAME_WIDTH, FRAME_HEIGHT), Point(0, FRAME_HEIGHT - 1), CV_RGB(255, 0, 0), 2);
		line(bw, Point(0, FRAME_HEIGHT), Point(0, 0), CV_RGB(255, 0, 0), 2);

		findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);


		
		//vector<vector<Point> >hull(contours.size());
		aPDcoeff = 0.01;
		for (int i = 0; i < contours.size(); i++)
		{
			//arcLength(Mat(contours[i]), true)*aPDcoeff
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*aPDcoeff, true);
			//!(approx.size() == 8)
			if (fabs(contourArea(contours[i])) < 50|| isContourConvex(approx))
				continue;

			

			convexHull(Mat(contours[i]), hull, false);
			
			
			//cont_avgs[i] = 5;

			rRect = minAreaRect(contours[i]);



			Point2f vertices[4];
			rRect.points(vertices);
			for (int i = 0; i < 4; i++){
				//line(display, vertices[i], vertices[(i + 1) % 4], Scalar(255, 255, 0));

			}

			LineIterator it(src, vertices[0], vertices[2], 8);
			vector<Vec3b> buf(it.count);
			vector<Point> points(it.count);
			//line(display, vertices[0], vertices[2], Scalar(255, 255, 0));
			int avg_sum = 0;
			for (int i = 0; i < it.count; i++, ++it)
			{
				points[i] = it.pos();
				Vec3b colour = src.at<Vec3b>(points[i]);
				avg_sum += (colour.val[0] + colour.val[1] + colour.val[2])/3;
				if (it.count - i - 1 == 0)  avg_sum = avg_sum / i;
			}

			LineIterator it2(src, vertices[1], vertices[3], 8);
			vector<Vec3b> buf2(it2.count);
			vector<Point> points2(it2.count);
			//line(display, vertices[1], vertices[3], Scalar(255, 255, 0));
			int avg_sum2 = 0;
			for (int i = 0; i < it2.count; i++, ++it2)
			{
				points2[i] = it2.pos();
				Vec3b colour = src.at<Vec3b>(points2[i]);
				avg_sum2 += (colour.val[0] + colour.val[1] + colour.val[2]) / 3;
				if (it2.count - i - 1 == 0) avg_sum2 = avg_sum2 / i;
			}
			//printf("%d \t %d \n", avg_sum, avg_sum2);
			//if (((avg_sum + avg_sum2) / 2) > 40) continue;
			
			
			//int IntercectX = ((vertices[i].x + vertices[i + 2].x) / 2 + (vertices[i+1].x + vertices[i  +3].x) / 2)/2;
			//int IntercectY = ((vertices[i].y + vertices[i + 2].y) / 2 + (vertices[i + 1].y + vertices[i + 3].y) / 2) / 2;

			//circle(src, Point(IntercectX, IntercectY), 2, Scalar(0, 0, 255), 1, 8, 0);


			
			int k = 0;
			for (k = 0; k < hull.size() - 1; k++){
				line(display, hull[k], hull[k + 1], CV_RGB(0, 255, 0), 1);
				string s = to_string(k);
				//putText(display, s, Point(hull[k].x, hull[k].y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 0), 0.5);
			}
			line(display, hull[k], hull[0], CV_RGB(0, 255, 0), 1);
			
			


			/*


			int eps = 4;
			double eps2 = 0.3;

			double x0 = approx[0].x;
			double x1 = approx[1].x;
			double x2 = approx[2].x;
			double x3 = approx[3].x;
			double x4 = approx[4].x;
			double x5 = approx[5].x;
			double x6 = approx[6].x;
			double x7 = approx[7].x;

			double y0 = approx[0].y;
			double y1 = approx[1].y;
			double y2 = approx[2].y;
			double y3 = approx[3].y;
			double y4 = approx[4].y;
			double y5 = approx[5].y;
			double y6 = approx[6].y;
			double y7 = approx[7].y;


			//соотнощение малой и большой стороны креста ~0.5
			double length_top = (((abs(x0 - x1) + abs(x0 - x7)) / 2) + ((abs(y0 - y1) + abs(y0 - y7)) / 2)) / 2;
			double length_bot = (((abs(x3 - x4) + abs(x4 - x5)) / 2) + ((abs(y3 - y4) + abs(y4 - y5)) / 2)) / 2;
			double ratio1 = ((((length_top + length_bot) / length_top - 0.5) + ((length_top + length_bot) / length_bot - 0.5))) / 2 - 0.5;


			double length_left = (((abs(x2 - x1) + abs(x2 - x3)) / 2) + ((abs(y2 - y1) + abs(y2 - y3)) / 2)) / 2;
			double length_right = (((abs(x6 - x7) + abs(x6 - x5)) / 2) + ((abs(y6 - y7) + abs(y6 - y5)) / 2)) / 2;
			double ratio2 = ((((length_left + length_right) / length_left - 0.5) + ((length_left + length_right) / length_right - 0.5))) / 2 - 0.5;

			//printf("ratio1: %f \t,ratio2: %f \t sr: %f\n", ratio1, ratio2, (ratio1 + ratio2) / 2);

			if (abs((ratio1 + ratio2) / 2 - 1) > 0.1) {
				iscross = false; continue;
			};
			for (int j = 0; j < approx.size() - 6; j++){
				double ang1 = angle(approx[j], approx[j + 1], approx[j + 2]);
				double ang2 = angle(approx[j + 1], approx[j + 2], approx[j + 3]);
				printf("ang1: %f\t, ang2: %f \n", ang1, ang2);
				if (ang1>0.7){
					if (!(ang1>0.7 && ang2 < 0.3)) { iscross = false; continue; }
				}
			}



			if (iscross) putText(display, "Cross detected", Point(10, 200), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 1.0);



			*/
			/*
			k = 0;
			for (k = 0; k < approx.size() - 1; k++){
				line(display, approx[k], approx[k + 1], CV_RGB(255, 0, 0), 1);
			}
			line(display, approx[k], approx[0], CV_RGB(255, 0, 0),	1);
			*/
			//for (int j = 0; j < approx.size(); j++){
			//string s = to_string(approx.size());
				//putText(display, s, Point(approx[j].x, approx[j].y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 0), 0.5);
			//}
			//putText(display, s, Point(approx[1].x, approx[1].y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 0.5);

			
			}
		
		
		
		imshow("src", display);
		imshow("bw", bw);
		
		waitKey(1);

		//isfinish = false;
	}
	getchar();
	return 0;
}