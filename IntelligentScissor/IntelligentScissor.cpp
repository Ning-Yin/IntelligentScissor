#include <string>
#include "ImageProcessing.h"
#include "Dijkstra.h"
#include "UserInterface.h"

#define PIXEL_NUMBER_THRESHOLD 100000

using namespace std;
using namespace cv;

const string s = "Lena.png";

int main(int argc, char *argv[]) {
//read the image
	string ss;
	if (argc == 1) {
		ss = s;
	}
	else {
		ss = argv[1];
	}
	Mat img_original;
	img_original = imread(ss);
	if (img_original.empty()) {
		cerr << "can't read the image." << endl;
		system("pause");
		return -1;
	}
	else {
		cout << "image has been read" << endl;
	}
	//img_resize = img_original.clone();
	//resize(img_original, img_resize, Size(100, 100));
	//imshow("original", img_original);

	int nr = img_original.rows, nc = img_original.cols, size = nr*nc;
	
//convert the image to the gray float
	Mat grayc;
	cvtColor(img_original, grayc, CV_BGR2GRAY);
	Mat colorf;
	img_original.convertTo(colorf, CV_32FC3, 1.0f / 255);
	Mat grayf;
	cvtColor(colorf, grayf, CV_BGR2GRAY);
	cout << "image has been converted to gray" << endl;

//calculate the edge 
	Mat cannyc, cannyf;
	Canny(grayc, cannyc, 150, 100);
	cannyc.convertTo(cannyf, CV_32FC1, 1.0f / 255);
	cannyf = 1 - cannyf;
	cout << "canny has been calculated." << endl;
	Mat log, abs_log, norm_abs_log, cost_log;
	CalcLoG(grayf, log, 5);
	cout << "log has been calculated." << endl;
	abs_log = abs(log);
	cout << "abs_log has been calculated." << endl;
	normalize(abs_log, norm_abs_log, 1.0, 0.0, NORM_MINMAX);
	norm_abs_log = 1 - norm_abs_log;
	cout << "norm_abs_log has been calculated." << endl;

//calculate the gradient, magnitude of the gradient, and direction
	Mat gradient, magnitude_gradient, norm_magnitude_gradient, edge_direction;
	CalcGradient(grayf, gradient);
	cout << "gradient has been calculated." << endl;
	CalcMagnitudeGradient(gradient, magnitude_gradient);
	cout << "magnitude_gradient has been calculated." << endl;
	normalize(magnitude_gradient, norm_magnitude_gradient, 1.0, 0.0, NORM_MINMAX);
	norm_magnitude_gradient = 1 - norm_magnitude_gradient;
	cout << "norm_magnitude_gradient has been calculated." << endl;
	CalcEdgeDirection(gradient, magnitude_gradient, edge_direction);
	cout << "edge_direction has been calculated." << endl;

//calculate the cost of edge and gradient magnitude
	Mat cost_edge;
	addWeighted(cannyf, WEIGHT_C, norm_magnitude_gradient, WEIGHT_G, 0, cost_edge);
	cout << "cost_edge has been calculated." << endl;

//initialize the necessary data
	Mat path;
	Point point_root;
	vector<Point> vector_point_root;

//ui
	Mat  img_save, img_show;
	img_save = img_original.clone();
	img_show = img_save.clone();
	namedWindow("Intelligent Scissor");
	setMouseCallback("Intelligent Scissor", onMouse);
	cout << "user interface is ready." << endl << endl;
//show the selected image
	Mat area_select = Mat::zeros(nr, nc, CV_8UC1), img_result;
	if (size < PIXEL_NUMBER_THRESHOLD) {
		while (waitKey(10) != 27) {
			switch (mouse_event)
			{
			case NONE:
				imshow("Intelligent Scissor", img_show);
				break;
			case LEFTBUTTONDOWN:
				recordcontour(area_select, path, point_select);
				point_root = point_select;
				vector_point_root.push_back(point_root);
				drawpoint(img_show, point_root);
				Dijkstra(point_root, grayf, path, cost_edge, edge_direction);
				img_save = img_show.clone();
				mouse_event = LEFTBUTTONUP;
				break;
			case LEFTBUTTONUP:
				break;
			case MOUSEMOVE:
				img_show = img_save.clone();
				drawpath(img_show, path, point_select);
				imshow("Intelligent Scissor", img_show);
				break;
			case RIGHTBUTTONDOWNONCE:
				img_show = img_save.clone();
				point_select = vector_point_root.front();
				recordcontour(area_select, path, point_select);
				drawpath(img_show, path, point_select);
				imshow("Intelligent Scissor", img_show);
				img_save = img_show.clone();
				vector_point_root.clear();
				mouse_event = WAITING;
				break;
			case RIGHTBUTTONDOWNTWICE:
				floodFill(area_select, point_select, 1);
				img_original.copyTo(img_result, area_select);
				imshow("Result", img_result);
				imwrite("result.tif", img_result);
				cout << "result has been write" << endl;
				//initialize ui
				mouse_event = NONE;
				break;
			case MIDBUTTONDOWN:
				return 0;
			default:
				break;
			}
		}
	}
	else {
		while (waitKey(10) != 27) {
			switch (mouse_event)
			{
			case NONE:
				imshow("Intelligent Scissor", img_show);
				break;
			case LEFTBUTTONDOWN:
				recordcontour(area_select, path, point_select);
				point_root = point_select;
				vector_point_root.push_back(point_root);
				drawpoint(img_show, point_root);
				img_save = img_show.clone();
				mouse_event = LEFTBUTTONUP;
				break;
			case LEFTBUTTONUP:
				break;
			case MOUSEMOVE:
				img_show = img_save.clone();
				Dijkstra(point_root, point_select, grayf, path, cost_edge, edge_direction);
				drawpath(img_show, path, point_select);
				imshow("Intelligent Scissor", img_show);
				break;
			case RIGHTBUTTONDOWNONCE:
				img_show = img_save.clone();
				point_select = vector_point_root.front();
				Dijkstra(point_root, point_select, grayf, path, cost_edge, edge_direction);
				recordcontour(area_select, path, point_select);
				drawpath(img_show, path, point_select);
				imshow("Intelligent Scissor", img_show);
				img_save = img_show.clone();
				vector_point_root.clear();
				mouse_event = WAITING;
				break;
			case RIGHTBUTTONDOWNTWICE:
				floodFill(area_select, point_select, 1);
				img_original.copyTo(img_result, area_select);
				imshow("Result", img_result);
				imwrite("result.tif", img_result);
				cout << "result has been write" << endl;
				//initialize ui
				mouse_event = NONE;
				break;
			case MIDBUTTONDOWN:
				return 0;
			default:
				break;
			}
		}
	}
	return 0;
}

