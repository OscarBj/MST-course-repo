#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <stdlib.h>
#include <vector>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem::v1;

using namespace cv;

// Read next video timestamp to make sure the video is synchronized
double read_next_video_timestamp(std::ifstream &file) {
	std::string str;
	std::getline(file, str);

	std::string data = str.substr(str.find(':') + 2, str.find(',') - (str.find(':') + 2));
	double hr = std::stof(data.substr(0, data.find(':')));
	data.erase(0, data.find(':') + 1);
	double min = std::stof(data.substr(0, data.find(':')));
	data.erase(0, data.find(':') + 1);
	double sec = std::stof(data);

	sec = (hr * 60 + min) * 60 + sec;

	return sec;
};

//read points into point_array until the given timestamp is passed (minimum 1 line)
int read_until(std::ifstream &file, std::vector<cv::Point3d> &point_array, double timestamp, std::vector<std::string> &lidar_str) {
	std::string str;
	double new_timestamp = 0.0;
	double lo = timestamp - new_timestamp;
	int ctr = 0;
	while (lo > 0.0)
	{
		std::getline(file, str);
		//const char* str = istr.c_str();
		new_timestamp = std::stod(str.substr(1, str.find(':')));

		std::string data = str.substr(str.find(':') + 1, str.find(']') - 1);
		while (data.length() > 1) {

			std::string entry = data.substr(0, data.find(';'));

			double d = std::stof(entry.substr(0, entry.find('|')));
			entry.erase(0, entry.find('|') + 1);
			double h = std::stof(entry.substr(0, entry.find('|')));
			entry.erase(0, entry.find('|') + 1);
			double v = std::stof(entry.substr(0, entry.find('|')));

			data.erase(0, data.find(';') + 1);

			double diag = d * cos(v);

			point_array.push_back(Point3d(diag * cos(h), diag * sin(h), d * -sin(v)));

			lidar_str.push_back(str);
		}
		ctr++;
		lo = timestamp - new_timestamp;
	}
	return ctr;
};

//read objects into an array until the timestamp is passed (minimum 1 line)
int read_obj_until(std::ifstream &file, std::vector<cv::Vec4d> &object_array, double timestamp, std::vector<cv::Point3d> &extra_params) {
	std::string str;
	double new_timestamp = 0.0;
	double lo = timestamp - new_timestamp;
	int ctr = 0;
	while (lo > 0.0)
	{
		if (file.eof()) { return -1; }
		std::getline(file, str);
		new_timestamp = std::stod(str.substr(1, str.find(':')));

		std::string data = str.substr(str.find(':') + 1, str.find(']') - 1);
		while (data.length() > 1) {

			std::string entry = data.substr(0, data.find(';'));

			double center_x = std::stof(entry.substr(1, data.find(',')));
			entry.erase(0, entry.find(',') + 2);
			double center_y = std::stof(entry.substr(0, entry.find(')')));
			entry.erase(0, entry.find(')') + 2);

			//first 2 extra parameters: velocity x and velocity y      
			double vel_x = std::stof(entry.substr(1, data.find(',')));
			entry.erase(0, entry.find(',') + 2);
			double vel_y = std::stof(entry.substr(0, entry.find(')')));
			entry.erase(0, entry.find(')') + 2);

			//remove second velocity parameter
			entry.erase(0, entry.find(')') + 2);

			//read width and height
			double width = std::stof(entry.substr(1, data.find(',')));
			entry.erase(0, entry.find(',') + 2);
			double height = std::stof(entry.substr(0, entry.find(')')));
			entry.erase(0, entry.find(')') + 2);

			int age = std::stoi(entry.substr(0, entry.find('|')));

			data.erase(0, data.find(';') + 1);

			object_array.push_back(cv::Vec4d(center_x - width / 2.0, center_y - height / 2.0, center_x + width / 2.0, center_y + height / 2.0));
			extra_params.push_back(Point3d(vel_x, vel_y, age));
		}
		ctr++;
		lo = timestamp - new_timestamp;
	}
	return ctr;
};


// Calculates rotation matrix given euler angles.
cv::Mat eulerAnglesToRotationMatrix(double* theta)
{
	// Calculate rotation about x axis
	cv::Mat R_x = (cv::Mat_<double>(3, 3) <<
		1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0])
		);

	// Calculate rotation about y axis
	cv::Mat R_y = (Mat_<double>(3, 3) <<
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	cv::Mat R_z = (Mat_<double>(3, 3) <<
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);


	// Combined rotation matrix
	cv::Mat R = R_z * R_y * R_x;

	return R;

}


// Return true if 2 given boxes overlap anywhere
bool overlap(cv::Vec4d obj1, cv::Vec4d obj2) {
	return !(obj1[0] > obj2[2] || obj2[0] > obj1[2] || obj1[1] < obj2[3] || obj2[1] < obj1[3]);
}

//Return the distance between point and a given rectangle
double distance(cv::Vec4d rect, cv::Point3d p) {
	double dx = std::max(rect[0] - p.x, p.x - rect[2]);
	dx = std::max(dx, 0.0);
	double dy = std::max(rect[1] - p.y, p.y - rect[3]);
	dy = std::max(dy, 0.0);
	return std::sqrt(dx*dx + dy * dy);
}

//Fuses all boxes in the given array that overlap
void fuse_similar_objects(std::vector<cv::Vec4d> objects) {
	for (int i = 0; i < objects.size(); i++) {
		for (int j = i + 1; j < objects.size(); j++) {
			if (overlap(objects[i], objects[j])) {
				objects[i][0] = std::min(objects[i][0], objects[j][0]);
				objects[i][1] = std::min(objects[i][1], objects[j][1]);
				objects[i][2] = std::min(objects[i][0], objects[j][0]);
				objects[i][3] = std::min(objects[i][1], objects[j][1]);
				objects.erase(objects.begin() + j);
				i = 0;
				j = 0;
			}
		}
	}
}

int main(int, char**)
{
	//Lidar rotation relative to camera
	double rv[3];

	rv[0] = -0.012;
	rv[1] = -0.036;
	rv[2] = 0.018;

	//Lidar translation relative to camera
	cv::Mat tvec(3, 1, cv::DataType<double>::type);
	tvec.at<double>(0) = -0.05;
	tvec.at<double>(1) = -0.2;
	tvec.at<double>(2) = 0.;

	//Distortion matrix for the camera
	cv::Mat K(3, 3, cv::DataType<double>::type);

	K.at<double>(0, 0) = 6.149738e+02 * 2; // multiply by 2
	K.at<double>(0, 1) = 0.;
	K.at<double>(0, 2) = 640.;//960.;

	K.at<double>(1, 0) = 0.;
	K.at<double>(1, 1) = 6.149738e+02 * 1.5;     // multiply 1.5
	K.at<double>(1, 2) = 360;//540;

	K.at<double>(2, 0) = 0.;
	K.at<double>(2, 1) = 0.;
	K.at<double>(2, 2) = 1.;
	cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);
	distCoeffs.at<double>(0) = -4.5165982392117821e-01;
	distCoeffs.at<double>(1) = 2.1024584023087808e-01;
	distCoeffs.at<double>(2) = 0.;
	distCoeffs.at<double>(3) = 0.;
	distCoeffs.at<double>(4) = -3.3092525200640177e-01;


													 //Matrix for relative rotation of lidar and camera
	cv::Mat rvecR(3, 3, cv::DataType<double>::type);
	cv::Mat baseR(3, 3, cv::DataType<double>::type);
	baseR.at<double>(0, 0) = 0.;
	baseR.at<double>(1, 0) = 0.;
	baseR.at<double>(2, 0) = 1.;
	baseR.at<double>(0, 1) = -1.;
	baseR.at<double>(1, 1) = 0.;
	baseR.at<double>(2, 1) = 0.;
	baseR.at<double>(0, 2) = 0.;
	baseR.at<double>(1, 2) = -1.;
	baseR.at<double>(2, 2) = 0.;
	cv::Mat er = eulerAnglesToRotationMatrix(rv);
	rvecR = baseR * er;
	cv::Mat rvec(3, 1, cv::DataType<double>::type);
	cv::Rodrigues(rvecR, rvec);

	//Initialization of point arrays

	//Current frame for point projection
	Mat baseImg;
	//Points from lidar
	std::vector<cv::Point3d> points;
	//Projected points from lidar
	std::vector<cv::Point2d> projection_points;
	//Distances to points from lidar
	std::vector<double> distances;
	//Objects as detected by lidar
	std::vector<cv::Vec4d> objects;
	//Projected objects in opencv format
	std::vector<cv::Rect> projection_objects;

	//Some extra paremeters
	std::vector<cv::Point3d> extra_pars;
	std::vector<std::string> lpoints_str;

	//current timestamp
	double timestamp = 0.0;
	double prev_stamp = -100.0;
	//whether to play the video
	int play = 1;
	//in paused mode tells to read the next frame
	int nf = 0;

	//Files to use for point projection
	std::ifstream file("./lidarPoints.txt");
	std::ifstream ofile("./lidarObjects.txt");
	std::ifstream tfile("./lidarTimestamps.txt");
	VideoCapture cap("./lidarVideo.mp4");

	if (!cap.isOpened()) {
		return -1; // check if we succeeded 
	}

	//open lidar logs and read 2 lines from them before starting
	std::string str;
	std::string ostr;
	std::getline(file, str);
	std::getline(ofile, str);
	str = "";
	ostr = "";
	std::getline(ofile, ostr);
	std::getline(file, str);

	//Configure offset, as zero for time for camera and lidar are different
	double orig_start = std::stod(str.substr(1, str.find(":")));
	double offset = -7.4;
	double next_timestamp = orig_start;

	//Initialize window for showing data
	namedWindow("fusion", 1);


	for (;;)
	{
		Mat frame;
		//If the video is playing, read the next frame
		if (play || nf) {
			nf = 0;
			cap >> frame;
			frame.copyTo(baseImg);
			timestamp = read_next_video_timestamp(tfile) + orig_start;
		}
		else {
			baseImg.copyTo(frame);
		}

		//Clear the arrays and read new data into them when the timestamps are passed
		if (timestamp >= next_timestamp) {
			if (timestamp>(orig_start - offset) && prev_stamp < (timestamp + offset)) {
				points.clear();
				objects.clear();
				extra_pars.clear();
				lpoints_str.clear();
				int sz = read_until(file, points, timestamp + offset, lpoints_str);
				if (sz < 0) { return 0; }
				read_obj_until(ofile, objects, timestamp + offset, extra_pars);
				prev_stamp = timestamp + offset;
			}
			next_timestamp = timestamp + 0.2;
		}

		//If there are Lidar points to parse
		if (points.size() > 0) {
			//Clear the arrays
			projection_points.clear();
			projection_objects.clear();
			distances.clear();
			//Project the 3D Lidar points onto a 2D pane, that can be shown on a picture 
			projectPoints(points, rvec, tvec, K, distCoeffs, projection_points);
			//For each point add it's distance to a separate array
			for (int i = 0; i < points.size(); i++) {
				distances.push_back(points[i].x);
			}

			//if lidar has detected objects
			if (objects.size()> 0) {
				//fuse similar objects together
				fuse_similar_objects(objects);
				for (int i = 0; i < objects.size(); i++) {
					projection_objects.push_back(cv::Rect(0, 0, 0, 0));
				}

				// for each point figure out what object it belongs to
				for (int i = 0; i < points.size(); i++)
				{
					int leastIndex = 0;    // not used?                                 
					double leastDistance = distance(objects[0], points[i]); // assume its closest to first object by default
					for (int j = 1; j < objects.size(); j++)
					{
						double temp = distance(objects[j], points[i]);
						if (leastDistance > temp)
						{
							// we found a new closest object
							leastIndex = j;         // this is the point -> object mapping (projection_objects[leastIndex])
							leastDistance = temp;   // temporary variable to find the closest object for each point
						}
					}
					if (leastDistance >= 100.0)
					{
						// discard this point, (its not close enough to any object)
						// nothing happens
					}
					else
					{
						// modify the drawn rectangles to encompass this projected points
						// modify y-axis

						// if the rectangle hasn't been modified
						if (projection_objects[leastIndex].y == 0 && projection_objects[leastIndex].x == 0)
						{
							// set object y to point y
							projection_objects[leastIndex].y = projection_points[i].y;
							projection_objects[leastIndex].x = projection_points[i].x;


						}
						// rectangle is not high enough
						if (projection_objects[leastIndex].y > projection_points[i].y)
						{
							// must rise y coordinate and increase height by same amount
							int diff = projection_objects[leastIndex].y - projection_points[i].y;
							projection_objects[leastIndex].y -= diff;
							projection_objects[leastIndex].height += diff;


						}
						// rectangle is not low enough
						if (projection_objects[leastIndex].y + projection_objects[leastIndex].height < projection_points[i].y)
						{
							int diff = projection_points[i].y - projection_objects[leastIndex].y - projection_objects[leastIndex].height;
							projection_objects[leastIndex].height += diff;
						}


						// modify x-axis

						// if the rectangle hasn't been modified

						// rectangle is not wide enough left hand side
						if (projection_objects[leastIndex].x > projection_points[i].x)
						{
							// must move x coordinate and increase width by same amount
							int diff = projection_objects[leastIndex].x - projection_points[i].x;
							projection_objects[leastIndex].x -= diff;
							projection_objects[leastIndex].width += diff;


						}
						// rectangle is not wide enough righthand side
						if (projection_objects[leastIndex].x + projection_objects[leastIndex].width < projection_points[i].x)
						{
							int diff = projection_points[i].x - projection_objects[leastIndex].x - projection_objects[leastIndex].width;
							projection_objects[leastIndex].width += diff;
						}
					}

				}

				for (int j = 0; j < objects.size(); j++)
				{
					int color = (50 * j * 255 / 150) % 255;           // make arbitrary colors for different objects
					cv::rectangle(frame, cv::Rect(projection_objects[j].x - 10, projection_objects[j].y - 10, projection_objects[j].width + 20, projection_objects[j].height + 20), Scalar(color, 0, 255 - color), 2);     // draw rectangles around objects, and make the rectangle slighty bigger   
				}



			}

			points.clear();
			objects.clear();
		}
		//For each projected point, draw it and the distance on top of it.
		std::vector<cv::Rect> textBoxes;
		for (int i = 0; i < projection_points.size(); i++) {
			if (projection_points[i].x < 0 || projection_points[i].x > 1280 || projection_points[i].y < 0 || projection_points[i].y > 720) { continue; }
			int color = distances[i] * 255 / 150;
			if (color > 255) { color = 255; }
			//Do not draw points that are closer than 3 meters, since those are likely false positives
			if (color > 3) {
				circle(frame, projection_points[i], 2, Scalar(color, 0, 255 - color), 2);

				std::string dist = std::to_string(int(distances[i]));
				Point p = Point(projection_points[i].x + 3, projection_points[i].y - 5);
			}
		}

		imshow("fusion", frame);

		if (play) {
			int key = waitKey(30);
			switch (key) {
			case 'v':
				offset -= 0.2;
				std::cout << "new offset: " << (offset) << std::endl;
				break;
			case 'b':
				offset += 0.2;
				std::cout << "new offset: " << (offset) << std::endl;
				break;
			case 32:
				play = 0;
				break;
			case 27:
				return 0;
			}
		}
		else {
			int waitingForKey = 1;
			while (waitingForKey) {
				int key = waitKey(30);
				switch (key) {
				case 32:
					waitingForKey = 0;
					play = 1;
					break;
				case 'n':
					waitingForKey = 0;
					nf = 1;
					break;
				case 27:
					return 0;
				}
			}
		}
	}
	return 0;
}



