#pragma once
#define _USE_MATH_DEFINES
#define WEIGHT_C 0.43f
#define WEIGHT_Z 0.43f
#define WEIGHT_G 0.14f
#define WEIGHT_D 0.43f
#include "ImageProcessing.h"
#include <iostream>

void Dijkstra(cv::Point point_root, cv::Mat& img, cv::Mat& path, cv::Mat& log, cv::Mat& edge_direction);
void Dijkstra(cv::Point point_root, cv::Point point_goal, cv::Mat& img, cv::Mat& path, cv::Mat& log, cv::Mat& edge_direction);
void UpdateNeighborCost(cv::Mat& cost, cv::Mat& log, cv::Mat& path, cv::Mat& mask, cv::Mat& edge_direction, cv::Point point_parent);
float CalcDirectionCost(const cv::Mat& direction, const int x_parent, const int y_parent, const int x_child, const int y_child);