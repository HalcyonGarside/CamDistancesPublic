#ifndef __CAMERACALIB__
#define __CAMERACALIB__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "..\ext\setforge\src\CameraParameters.h"

using namespace std;

struct camPos {
	cv::Mat rotation; // The rotation matrix of the camera
	cv::Mat translation; // The translation matrix of the camera
};

class CameraCalibration {
private:
	int i_numCameras;
	vector<string> rgsCalibFilePaths;
	vector<double> m_iDistances;

	/*TODO: ALLOW USERS TO CHANGE THESE VALUES*/
	int icbX;
	int icbY;
	double dcbSquareSize;

	/*
	checkerboardPnP: Solves PnP
	*/
	camPos checkerboardPnP(cv::Mat& cboardPic, cv::Mat& cboard3DPts, string& calibFilepath);

public:

	CameraCalibration();
	CameraCalibration(int numCams);
	~CameraCalibration();
	
	/*
	setNumCameras: Sets the number of cameras being used in this camera_calib.

		@param: num - the number of cameras to be used.
		@return: a true if the number of cameras in this object was changed.
	*/
	bool setNumCameras(int num);

	/*
	setCalibrationFile: Sets the file path of the calibration file specific to the camera.
		@param: camNum - the camera whose calibration filepath is being changed.
		@param: path - the filepath of the camera's calibration json.
		@return: true if the filepath was changed.
	*/
	bool setCalibrationFile(int camNum, string& path);

	/*
	displayCamera: Displays the image from the specified camera at the moment the function is called.  
		Returns true if the camera exists and the image is displayed.
	*/
	bool captureCameras();

	/*
	start: Starts the calibration program.  Returns true when the program is finished.
	*/

	bool start();

	/*
	showDistances: Shows the position of each camera in reference to the origin camera.
	*/
	void showDistances();


	/*
	getFilepaths: Prints out all filepaths for the camera jsons
	*/
	void getFilepaths();


	/*
	getCamMatrix: Debug.  Gets the camera matrix from the camera json.
	*/
	void getCamMatrix();

};
#endif