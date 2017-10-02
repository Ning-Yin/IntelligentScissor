#include "Dijkstra.h"

using namespace cv;
using namespace std;

void Dijkstra(Point point_root, Mat& img, Mat& outputpath, Mat &cost_edge, Mat& edge_direction) {
	cout << "Waiting..." << endl << "Dijkstra is being calculated." << endl;
	/*
	1. intialize the cost image, e.i. all the pixels of cost are infinite. intialize the path image
	2. let the root pixel of cost be 0, let the root pixel be certained, and update the neighbors cost value
	3. enter into the loop:
		while(the goal point is uncertained)
			i)	select the pixel uncertained of the smallest cost value,
			ii)	let it be certained
			iii)update the neighbors uncertained cost value and their parent point
		end the loop
	*/
	const int nr = img.rows, nc = img.cols, np = nr*nc;

	//1.
	//the first channel of path denotes the x coordinate of the parent point. -1 represents the point has no parent point
	//the second channel of path denotes the y coordinate of the parent point. -1 represents the point has no parent point
	//the mask denotes whether the point is certained. 1 represents the point is uncertained. 0 represents the point is certained.
	Mat cost_final(nr, nc, CV_32FC1, Scalar(3.2e38f));
	Mat path(nr, nc, CV_16SC2, Scalar(-1, -1));
	Mat mask(nr, nc, CV_8UC1, Scalar(1));
		
	//2.
	cost_final.at<float>(point_root) = 0;
	
	mask.at<uchar>(point_root) = 0;
	UpdateNeighborCost(cost_final, cost_edge, path, mask, edge_direction, point_root);
	
	//3.
	int x_update, y_update;
	Point point_update;
	while (countNonZero(mask) != 0) {
		//i)
		minMaxLoc(cost_final, NULL, NULL, &point_update, NULL, mask);
		x_update = point_update.x;
		y_update = point_update.y;
		//ii)
		mask.at<uchar>(y_update, x_update) = 0;
		//iii)
		UpdateNeighborCost(cost_final, cost_edge, path, mask, edge_direction, point_update);
	}

	outputpath = path.clone();

//record the times
	static int times_Dijkstra = 1;
	cout << "Dijkstra has been calculated for " << times_Dijkstra++ << (times_Dijkstra == 1 ? " time." : " times.") << endl << endl;
}
void Dijkstra(Point point_root, Point point_goal, Mat& img, Mat& outputpath, Mat &cost_edge, Mat& edge_direction) {
	/*
	1. intialize the cost image, e.i. all the pixels of cost are infinite. intialize the path image
	2. let the root pixel of cost be 0, let the root pixel be certained, and update the neighbors cost value
	3. enter into the loop:
	while(the goal point is uncertained)
	i)	select the pixel uncertained of the smallest cost value,
	ii)	let it be certained
	iii)update the neighbors uncertained cost value and their parent point
	end the loop
	*/
	const int nr = img.rows, nc = img.cols, np = nr*nc;

	//1.
	//the first channel of path denotes the x coordinate of the parent point. -1 represents the point has no parent point
	//the second channel of path denotes the y coordinate of the parent point. -1 represents the point has no parent point
	//the mask denotes whether the point is certained. 1 represents the point is uncertained. 0 represents the point is certained.
	static Point point_root_save = point_root;
	static Mat* ptr_cost_final = new Mat(nr, nc, CV_32FC1, Scalar(3.2e38f));
	static Mat* ptr_path = new Mat(nr, nc, CV_16SC2, Scalar(-1, -1));
	static Mat* ptr_mask = new Mat(nr, nc, CV_8UC1, Scalar(1));
	if (point_root != point_root_save) {
		delete ptr_cost_final;
		delete ptr_path;
		delete ptr_mask;
		point_root_save = point_root;
		ptr_cost_final = new Mat(nr, nc, CV_32FC1, Scalar(3.2e38f));
		ptr_path = new Mat(nr, nc, CV_16SC2, Scalar(-1, -1));
		ptr_mask = new Mat(nr, nc, CV_8UC1, Scalar(1));
	}
	Mat cost_final = *ptr_cost_final;
	Mat path = *ptr_path;
	Mat mask = *ptr_mask;

	//2.
	cost_final.at<float>(point_root) = 0;

	mask.at<uchar>(point_root) = 0;
	UpdateNeighborCost(cost_final, cost_edge, path, mask, edge_direction, point_root);

	//3.
	int x_update, y_update;
	Point point_update;
	while (mask.at<uchar>(point_goal) == 1) {
		//i)
		minMaxLoc(cost_final, NULL, NULL, &point_update, NULL, mask);
		x_update = point_update.x;
		y_update = point_update.y;
		//ii)
		mask.at<uchar>(y_update, x_update) = 0;
		//iii)
		UpdateNeighborCost(cost_final, cost_edge, path, mask, edge_direction, point_update);
	}

	outputpath = path.clone();
}
void UpdateNeighborCost(Mat& cost_final, Mat& cost_edge, Mat& path, Mat& mask, Mat& edge_direction, Point point_parent) {
	int x_parent = point_parent.x, y_parent = point_parent.y;
	float cost_child;
	for (int y_child = y_parent - 1; y_child <= y_parent + 1; y_child++) {
		for (int x_child = x_parent - 1; x_child <= x_parent + 1; x_child++) {
			if (y_child < 0 || x_child < 0) {
				continue;
			}
			if (y_child >= cost_final.rows || x_child >= cost_final.cols) {
				continue;
			}
			if (mask.at<uchar>(y_child, x_child) == 0) {
				continue;
			}
			if (x_child == x_parent&&y_child == y_parent) {
				continue;
			}
			cost_child = cost_final.at<float>(y_parent, x_parent);
			cost_child += cost_edge.at<float>(y_child, x_child);
			cost_child += CalcDirectionCost(edge_direction, x_parent, y_parent, x_child, y_child);
			if (cost_child < cost_final.at<float>(y_child, x_child)) {
				cost_final.at<float>(y_child, x_child) = cost_child;
				path.at<Vec2s>(y_child, x_child)[0] = x_parent;
				path.at<Vec2s>(y_child, x_child)[1] = y_parent;
			}
		}
	}
}
float CalcDirectionCost(const Mat& edge_direction, const int x_parent, const int y_parent, const int x_child, const int y_child) {
	float direction_parent[2], direction_child[2], cost_link[2];

	direction_parent[0] = edge_direction.at<Vec2f>(y_parent, x_parent)[0];
	direction_parent[1] = edge_direction.at<Vec2f>(y_parent, x_parent)[1];
	direction_child[0] = edge_direction.at<Vec2f>(y_child, x_child)[0];
	direction_child[1] = edge_direction.at<Vec2f>(y_child, x_child)[1];
	cost_link[0] = float(x_child - x_parent);
	cost_link[1] = float(y_child - y_parent);
	if (cost_link[0] != 0.0f && cost_link[1] != 0.0f) {
		cost_link[0] *= 0.707f;
		cost_link[1] *= 0.707f;
	}
	if (cost_link[0] * direction_parent[0] + cost_link[1] * direction_parent[1] < 0) {
		cost_link[0] = -cost_link[0];
		cost_link[1] = -cost_link[1];
	}

	float cost_direction_parent, cost_direction_child, cost_direction_final;
	cost_direction_parent = cost_link[0] * direction_parent[0] + cost_link[1] * direction_parent[1];
	cost_direction_child = cost_link[0] * direction_child[0] + cost_link[1] * direction_child[1];
	cost_direction_final = float((acos(cost_direction_parent) + acos(cost_direction_child)) / M_PI);
	cost_direction_final *= WEIGHT_D;
	return cost_direction_final;
}