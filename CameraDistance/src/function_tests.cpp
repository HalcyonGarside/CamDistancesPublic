#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

#define CAM_LENGTH 1600
#define CAM_HEIGHT 1200

using namespace std;

class function_tests {
public:
	int doTest() {
		/*
		Start solvePnP test...
		*/

		cout << "---------Starting the solvePnP test---------" << endl;

		//Initializing output matrices (for solvePnP's useExtrinsicGuess)
		cout << "Initializing output matrices..." << endl;
		cv::Mat mat_rotRes = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		cv::Mat mat_transRes = (cv::Mat_<double>(3, 1) << 0, 0, 0);

		//Camera matrix
		cout << "Initializing the camera matrix..." << endl;
		cv::Mat mat_cam = (cv::Mat_<double>(3, 3) << CAM_LENGTH, 0, CAM_LENGTH / 2, 0, CAM_HEIGHT, CAM_HEIGHT / 2, 0, 0, 1);

		//Distortion matrix
		cout << "Initializing the distortion matrix..." << endl;
		cv::Mat mat_dist = cv::Mat::zeros(4, 1, cv::DataType<double>::type);

		//Mess with these values
		cout << "Initializing the 3D points matrix..." << endl;
		cv::Mat mat_modelPts = (cv::Mat_<double>(3, 5) << 0, 0, 0, 100, 100, 0, -100, -100, 0, -100, 100, 0, 100, -100, 0);
		cout << "Initializing the 2D points matrix..." << endl;
		cv::Mat mat_imgPts = (cv::Mat_<double>(2, 5) << 800, 600, 800, 500, 800, 700, 600, 600, 1000, 600);

		//Our main problem to solve
		cout << "Finding the orientation of the camera..." << endl;
		cv::solvePnP(mat_modelPts, mat_imgPts, mat_cam, mat_dist, mat_rotRes, mat_transRes, true);

		//Debug
		cout << "Rotation: " << endl << mat_rotRes << endl << endl;
		cout << "Translation: " << endl << mat_transRes << endl << endl;



		/*
		Start findChessboardCorners test
		*/

		cout << "---------Starting the findChessboardCorners test---------" << endl;

		int i_points_x = 14;
		int i_points_y = 9;

		//Output matrix
		cout << "Initializing output matrix..." << endl;
		cv::Mat mat_detectedCorners;

		//Image
		cout << "Initializing image matrix..." << endl;
		cv::Mat mat_img = cv::imread("better_board.jpg");

		//Size of the chessboard on the image
		cout << "Initializing the chessboard size..." << endl;
		cv::Size sz_pattern = cvSize(i_points_x, i_points_y);

		//Main problem
		cout << "Finding the corners of the chessboard..." << endl;
		bool found = cv::findChessboardCorners(mat_img, sz_pattern, mat_detectedCorners);

		//Output
		if (found) {
			cout << "Detected corners:" << endl << mat_detectedCorners << endl << endl;
		}
		else {
			cout << "Corners not found." << endl << endl;
		}



		/*
		Using findChessboardCorners test to find position of camera
		*/

		//Create a matrix of the 3D positions of each of the checkerboard corners.


		cv::Mat mat_chess3DPts = cv::Mat::zeros((i_points_x * i_points_y), 3, cv::DataType<double>::type);
		int i_arrPos = 0;

		for (int i = 0; i < i_points_y; i++) {
			for (int j = 0; j < i_points_x; j++, i_arrPos++) {
				mat_chess3DPts.at<double>(i_arrPos, 0) = j * 0.021;
				mat_chess3DPts.at<double>(i_arrPos, 1) = i * 0.021;
				mat_chess3DPts.at<double>(i_arrPos, 2) = 0;
			}
		}

		//Re-initialize the rotation and translation matrices
		mat_rotRes = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		mat_transRes = (cv::Mat_<double>(3, 1) << 0, 0, 0);

		//Make camera matrix fit the picture
		mat_cam = (cv::Mat_<double>(3, 3) << mat_img.cols, 0, mat_img.cols / 2, 0, mat_img.rows, mat_img.rows / 2, 0, 0, 1);

		//DEBUG: Check the points of the initialized checkerboard coords
		cout << "3D positions of corners: " << endl << mat_chess3DPts << endl << endl;

		//Find position of the camera using those matrices
		cout << "Finding the position of the camera... " << endl << endl;
		cv::solvePnP(mat_chess3DPts, mat_detectedCorners, mat_cam, mat_dist, mat_rotRes, mat_transRes, true);

		//Output
		cout << "Cam 1:" << endl;
		cout << "Rotation: " << endl << mat_rotRes << endl << endl;
		cout << "Translation: " << endl << mat_transRes << endl << endl;


		/*
		Finding distance between two cameras in space.
		*/

		cv::Mat mat_rotRes_1 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		cv::Mat mat_transRes_1 = (cv::Mat_<double>(3, 1) << 0, 0, 0);
		cv::Mat mat_detectedCorners_1;

		cv::Mat mat_img_1 = cv::imread("better_board_1.jpg");

		cv::findChessboardCorners(mat_img_1, sz_pattern, mat_detectedCorners_1);

		cv::solvePnP(mat_chess3DPts, mat_detectedCorners_1, mat_cam, mat_dist, mat_rotRes_1, mat_transRes_1, true);

		cout << "Cam 2:" << endl;
		cout << "Rotation: " << endl << mat_rotRes_1 << endl << endl;
		cout << "Translation: " << endl << mat_transRes_1 << endl << endl;

		double i_dX = mat_transRes_1.at<double>(0) - mat_transRes.at<double>(0);
		double i_dY = mat_transRes_1.at<double>(1) - mat_transRes.at<double>(1);
		double i_dZ = mat_transRes_1.at<double>(2) - mat_transRes.at<double>(2);

		double totDist = sqrt((i_dX * i_dX) + (i_dY * i_dY) + (i_dZ * i_dZ));

		cout << "The total distance between cameras is " << totDist << endl << endl;

		cout << "---------Tests end---------" << endl;
	};
};