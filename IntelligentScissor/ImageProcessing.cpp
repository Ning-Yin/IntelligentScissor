#include "ImageProcessing.h"

using namespace cv;
using namespace std;

void padding(const Mat& src, Mat& dst, int paddingwidth) {
	int nr = src.rows;
	int nc = src.cols;
	int tp = src.type();
	dst.create(nr + paddingwidth * 2, nc + paddingwidth * 2, tp);
	for (int y = 0; y < nr; y++) {
		for (int x = 0; x < nc; x++) {
			dst.at<float>(y + paddingwidth, x + paddingwidth) = src.at<float>(y, x);
		}
	}
	//pading the left and right
	for (int y = 0; y < nr; y++) {
		for (int i = 0; i < paddingwidth; i++) {
			dst.at<float>(y + paddingwidth, paddingwidth - i - 1) = dst.at<float>(y + paddingwidth, paddingwidth - i);
			dst.at<float>(y + paddingwidth, nc + paddingwidth + i) = dst.at<float>(y + paddingwidth, nc + paddingwidth + i - 1);
		}
	}
	//padding the up and down
	for (int x = 0; x < nc; x++) {
		for (int j = 0; j < paddingwidth; j++) {
			dst.at<float>(paddingwidth - j - 1, x + paddingwidth) = dst.at<float>(paddingwidth - j, x + paddingwidth);
			dst.at<float>(nr + paddingwidth + j, x + paddingwidth) = dst.at<float>(nr + paddingwidth + j - 1, x + paddingwidth);
		}
	}
	//padding the corners
	for (int j = 0; j < paddingwidth; j++) {
		for (int i = 0; i < paddingwidth; i++) {
			dst.at<float>(paddingwidth - j - 1, paddingwidth - i - 1) = (dst.at<float>(paddingwidth - j - 1, paddingwidth - i) + dst.at<float>(paddingwidth - j, paddingwidth - i - 1)) / 2;
			dst.at<float>(paddingwidth - j - 1, nc + paddingwidth + i) = (dst.at<float>(paddingwidth - j - 1, nc + paddingwidth + i - 1) + dst.at<float>(paddingwidth - j, nc + paddingwidth + i)) / 2;
			dst.at<float>(nr + paddingwidth + j, paddingwidth - i - 1) = (dst.at<float>(nr + paddingwidth + j, paddingwidth - i) + dst.at<float>(nr + paddingwidth + j - 1, paddingwidth - i - 1)) / 2;
			dst.at<float>(nr + paddingwidth + j, nc + paddingwidth + i) = (dst.at<float>(nr + paddingwidth + j, nc + paddingwidth + i - 1) + dst.at<float>(nr + paddingwidth + j - 1, nc + paddingwidth + i)) / 2;
		}
	}
}
void CalcLoG(const Mat& src, Mat& dst, int filtersize) {
	int nr = src.rows, nc = src.cols;
	Mat srtpadding;
	padding(src, srtpadding, (filtersize - 1) / 2);
	dst = Mat::zeros(nr, nc, src.type());

	for (int y = 0; y < nr; y++) {
		for (int x = 0; x < nc; x++) {
			for (int j = 0; j < filtersize; j++) {
				for (int i = 0; i < filtersize; i++) {
					dst.at<float>(y, x) += srtpadding.at<float>(y + j, x + i)*LoGFilter[(filtersize - 3) / 2].at<char>(j, i);
				}
			}
		}
	}
}
void CalcGradient(const Mat& src, Mat& dst) {
	int nr = src.rows, nc = src.cols;
	Mat padding_src;
	padding(src, padding_src, 1);
	vector<Mat> gradient(2);
	gradient[0].create(nr, nc, src.type());
	gradient[1].create(nr, nc, src.type());
	for (int y = 0; y < nr; y++) {
		for (int x = 0; x < nc; x++) {
			gradient[0].at<float>(y, x) = (padding_src.at<float>(y + 1, x + 2) - padding_src.at<float>(y + 1, x)) / 2;
			gradient[1].at<float>(y, x) = (padding_src.at<float>(y + 2, x + 1) - padding_src.at<float>(y, x + 1)) / 2;
		}
	}
	merge(gradient, dst);
}
void CalcMagnitudeGradient(const Mat& src, Mat& dst) {
	int nr = src.rows, nc = src.cols;
	dst.create(src.size(), CV_32FC1);
	for (int y = 0; y < nr; y++) {
		for (int x = 0; x < nc; x++) {
			dst.at<float>(y, x) = powf(powf(src.at<Vec2f>(y, x)[0], 2) + powf(src.at<Vec2f>(y, x)[1], 2), 0.5);
		}
	}
}
void CalcEdgeDirection(const Mat& gradient, const Mat& magnitude_gradient, Mat& dst) {
	int nr = gradient.rows, nc = gradient.cols;
	dst.create(gradient.size(), gradient.type());
	for (int y = 0; y < nr; y++) {
		for (int x = 0; x < nc; x++) {
			if (magnitude_gradient.at<float>(y, x) != 0) {
				dst.at<Vec2f>(y, x)[0] = -gradient.at<Vec2f>(y, x)[1] / magnitude_gradient.at<float>(y, x);
				dst.at<Vec2f>(y, x)[1] = gradient.at<Vec2f>(y, x)[0] / magnitude_gradient.at<float>(y, x);
			}
			else {
				dst.at<Vec2f>(y, x)[0] = 0;
				dst.at<Vec2f>(y, x)[1] = 0;
			}
		}
	}
}