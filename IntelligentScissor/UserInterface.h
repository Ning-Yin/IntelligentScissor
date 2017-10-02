#pragma once

#include <iostream>
#include <opencv2/highgui.hpp>

enum MouseEvent { NONE = 0, LEFTBUTTONDOWN, LEFTBUTTONUP, MOUSEMOVE, RIGHTBUTTONDOWNONCE, RIGHTBUTTONDOWNTWICE, MIDBUTTONDOWN, WAITING };
extern MouseEvent mouse_event;
extern cv::Point point_select;

void onMouse(int event, int x, int y, int flag, void*);
void drawpoint(cv::Mat& img, cv::Point point);
void drawpath(cv::Mat& img, const cv::Mat& path, cv::Point point);
void recordcontour(cv::Mat& contour, const cv::Mat& path, cv::Point point);
