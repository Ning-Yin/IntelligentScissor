#include "UserInterface.h"

using namespace cv;
using namespace std;

MouseEvent mouse_event = NONE;
int x_, y_;
Point point_select;
void onMouse(int event, int x, int y, int flag, void*) {
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		point_select.x = x;
		point_select.y = y;
		mouse_event = LEFTBUTTONDOWN;
		break;
	case EVENT_LBUTTONUP:
		point_select.x = x;
		point_select.y = y;
		mouse_event = LEFTBUTTONUP;
		break;
	case EVENT_MOUSEMOVE:
		if (mouse_event != NONE&&mouse_event != WAITING) {
			point_select.x = x;
			point_select.y = y;
			mouse_event = MOUSEMOVE;
		}
		break;
	case EVENT_RBUTTONDOWN:
		if (mouse_event != WAITING) {
			mouse_event = RIGHTBUTTONDOWNONCE;
		}
		else {
			point_select.x = x;
			point_select.y = y;
			mouse_event = RIGHTBUTTONDOWNTWICE;
		}
		break;
	case EVENT_MBUTTONDOWN:
		mouse_event = MIDBUTTONDOWN;
		break;
	default:
		break;
	}
}
void drawpoint(Mat& img, Point point) {
	assert(img.type() == CV_8UC3);
	int nrow = img.rows;
	int ncol = img.cols;
	int ncha = img.channels();
	int step = static_cast<int>(img.step);
	uchar* data = img.data;
	int x = point.x, y = point.y;
	data += step*(y != 0 ? y - 1 : 0);
	for (int j = y != 0 ? y - 1 : 0; j <= y + 1 && j < nrow; j++) {
		*(data + x*ncha + 0) = 0;
		*(data + x*ncha + 1) = 0;
		*(data + x*ncha + 2) = 255;
		data += step;
	}
	data = img.data;
	data += step*y;
	for (int i = x != 0 ? x - 1 : 0; i <= x + 1 && i < ncol; i++) {
		*(data + i*ncha + 0) = 0;
		*(data + i*ncha + 1) = 0;
		*(data + i*ncha + 2) = 255;
	}
}
void drawpath(Mat& img, const Mat& path, Point point) {
	int x_parent = point.x, y_parent = point.y, x_child, y_child;
	x_child = path.at<Vec2s>(y_parent, x_parent)[0];
	y_child = path.at<Vec2s>(y_parent, x_parent)[1];
	while (x_child != -1 && y_child != -1) {
		img.at<Vec3b>(y_child, x_child)[0] = 255;
		img.at<Vec3b>(y_child, x_child)[1] = 0;
		img.at<Vec3b>(y_child, x_child)[2] = 0;
		x_parent = x_child;
		y_parent = y_child;
		x_child = path.at<Vec2s>(y_parent, x_parent)[0];
		y_child = path.at<Vec2s>(y_parent, x_parent)[1];
	}

//record the times
//	static int times_drawpath = 0;
//	cout << "path has been drawn for " << times_drawpath++ << " times." << endl;
}
void recordcontour(Mat& contour, const Mat& path, Point point) {
	static int flag = 0;
	flag++;
	if (flag == 1) {
		return;
	}
	int x_parent = point.x, y_parent = point.y, x_child, y_child;
	x_child = path.at<Vec2s>(y_parent, x_parent)[0];
	y_child = path.at<Vec2s>(y_parent, x_parent)[1];
	while (x_child != -1 && y_child != -1) {
		contour.at<uchar>(y_child, x_child) = 1;
		x_parent = x_child;
		y_parent = y_child;
		x_child = path.at<Vec2s>(y_parent, x_parent)[0];
		y_child = path.at<Vec2s>(y_parent, x_parent)[1];
	}
}