#pragma once

#define DEBUG

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <conio.h>

#include "..\include\camera_calib.h"

using namespace std;

int main(int argc, char** argv) 
{
	int camNum;
	string filepath = "";
	string file0 = ""; //Location of camera calib file 1
	string file1 = ""; //Location of camera calib file 2

	CameraCalibration* cam = new CameraCalibration(2);
	cam->captureCameras();

	cout << "----------Welcome to camera distance calibration!-----------" << endl
		<< "c -> to calibrate the two images that are displayed." << endl
		<< "r -> capture a different set of pictures." << endl
		<< "d -> set the file locations of the calibration JSONs." << endl
		<< "l -> Display all current filepaths set for the calibration files" << endl
		<< "\\ -> close the program." << endl << endl;

	char keyPressed = '!';
	while (keyPressed != '\\') 
	{
		keyPressed = cv::waitKey(0);

		switch (keyPressed) 
		{
			case 'c':
				if (!cam->start())
				{
					cout << "[ERROR] - CANNOT CALIBRATE CAMERAS" << endl << endl;
					break;
				}

				cam->showDistances();

				break;


			case 'r':
				cam->captureCameras();
				break;


			case 'd':
#ifdef DEBUG
				cam->setCalibrationFile(0, file0);
				cam->setCalibrationFile(1, file1);

#else
				system("CLS");
				cout << "Press a number key to change that camera's filepath." << endl;
				cout << "Press any other key to return to the main screen" << endl;

				keyPressed = cv::waitKey(0);

				if (keyPressed < '0' || keyPressed > '9') {
					system("CLS");
					cout << "c -> to calibrate the two images that are displayed." << endl
						<< "r -> capture a different set of pictures." << endl
						<< "d -> set the file locations of the calibration JSONs." << endl
						<< "l -> Display all current filepaths set for the calibration files" << endl
						<< "\\ -> close the program." << endl << endl;
					break;
				}

				cout << "Enter the filepath to add to this camera" << endl;

				camNum = keyPressed - 48;
				cin >> filepath;

				if (!cam->setCalibrationFile(camNum, filepath)) {
					system("CLS");
					cout << "That camera doesn't exist.  Please try again or change the number of cameras you have! " << endl << endl
						<< "c -> to calibrate the two images that are displayed." << endl
						<< "r -> capture a different set of pictures." << endl
						<< "d -> set the file locations of the calibration JSONs." << endl
						<< "l -> Display all current filepaths set for the calibration files" << endl
						<< "\\ -> close the program." << endl << endl;
					break;
				}

				system("CLS");
				cout << "Successfully changed filepath!" << endl << endl
					<< "c -> to calibrate the two images that are displayed." << endl
					<< "r -> capture a different set of pictures." << endl
					<< "d -> set the file locations of the calibration JSONs." << endl
					<< "l -> Display all current filepaths set for the calibration files" << endl
					<< "\\ -> close the program." << endl << endl;
#endif
				break;


			case 'l':
				cam->getFilepaths();
				break;

			case 'p':
				cam->getCamMatrix();
				break;
		}
	}

	return 0;
}