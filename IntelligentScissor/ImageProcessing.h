#pragma once

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


const cv::Mat LoGFilter[2] = { (cv::Mat_<char>(3, 3) <<
	0, 1, 0,
	1, -4, 1,
	0, 1, 0),
	(cv::Mat_<char>(5, 5) <<
		0, 0, 1, 0, 0,
		0, 1, 2, 1, 0,
		1, 2, -16, 2, 1,
		0, 1, 2, 1, 0,
		0, 0, 1, 0, 0) };
void padding(const cv::Mat& src, cv::Mat& dst, int paddingwidth);
void CalcLoG(const cv::Mat& src, cv::Mat& dst, int filtersize);//may be actualized in the end
//void CalcAbsolute(const cv::Mat& srt, cv::Mat& dst);
void CalcGradient(const cv::Mat& src, cv::Mat& dst);
void CalcMagnitudeGradient(const cv::Mat& src, cv::Mat& dst);
void CalcEdgeDirection(const cv::Mat& gradient, const cv::Mat& magnitude_gradient, cv::Mat& dst);