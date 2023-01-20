#define _CRT_SECURE_NO_WARNINGS

#include "../include/camera_calib.h"
#include <stdio.h>
#include <fstream>

CameraCalibration::CameraCalibration() 
	: icbX(4), icbY(7), dcbSquareSize(30.0)
{
	rgsCalibFilePaths = vector<string>();
	m_iDistances = vector<double>();
	i_numCameras = 0;
}

CameraCalibration::CameraCalibration(int numCams) 
	: icbX(4), icbY(7), dcbSquareSize(0.03)
{
	rgsCalibFilePaths = vector<string>(numCams);
	m_iDistances = vector<double>(numCams);
	i_numCameras = numCams;
}

CameraCalibration::~CameraCalibration() {}

bool CameraCalibration::setNumCameras(int num) 
{
	if (i_numCameras != num) {
		i_numCameras = num;
		rgsCalibFilePaths.resize(num);
		return true;
	}
	return false;
}

bool CameraCalibration::setCalibrationFile(int camNum, std::string& path) 
{
	if (i_numCameras == 0 || camNum >= i_numCameras || camNum < 0) return false;

	rgsCalibFilePaths.at(camNum) = path;
	return true;
}

bool CameraCalibration::captureCameras() 
{
	if (!i_numCameras) return false;

	cv::VideoCapture cap;
	cv::Mat frame;

	vector<cv::Mat> matrices(i_numCameras);
	
	for (int camNum = 0; camNum < i_numCameras; camNum++)
	{
		cap.open(camNum);
		cap.read(matrices.at(camNum));
		cap.release();
	}

	for (int camNum = 0; camNum < i_numCameras; camNum++)
	{
		char picName[20] = "";

		sprintf(picName, "out/cam%d.png", camNum);

		cv::imwrite(picName, matrices.at(camNum));
		cv::imshow(picName, matrices.at(camNum));
	}

	return true;
}

bool CameraCalibration::start() 
{

	vector<camPos> camPositions = vector<camPos>(i_numCameras);

	for (int camNum = 0; camNum < i_numCameras; camNum++)
	{
		cv::Mat cboard3DPts = cv::Mat::zeros((icbX * icbY), 3, cv::DataType<double>::type);

		int i_arrPos = 0;

		for (int i = 0; i < icbY; i++) 
		{
			for (int j = 0; j < icbX; j++, i_arrPos++) {
				cboard3DPts.at<double>(i_arrPos, 0) = j * dcbSquareSize;
				cboard3DPts.at<double>(i_arrPos, 1) = i * dcbSquareSize;
				cboard3DPts.at<double>(i_arrPos, 2) = 0;
			}
		}

		char picname[20];
		sprintf(picname, "out/cam%d.png", camNum);

		cv::Mat cboardPic = cv::imread(picname);

		camPositions.at(camNum) = checkerboardPnP(cboardPic, cboard3DPts, rgsCalibFilePaths.at(camNum));
	}

	cv::Mat originTrans = camPositions.at(0).translation;
	m_iDistances.at(0) = 0;

	for (int camNum = 1; camNum < i_numCameras; camNum++)
	{
		cv::Mat compTrans = camPositions.at(camNum).translation;

		double dX = compTrans.at<double>(0) - originTrans.at<double>(0);

		cout << dX << endl;

		double dY = compTrans.at<double>(1) - originTrans.at<double>(1);

		cout << dY << endl;

		double dZ = compTrans.at<double>(2) - originTrans.at<double>(2);

		cout << dZ << endl << endl;

		m_iDistances.at(camNum) = sqrt((dX * dX) + (dY * dY) + (dZ * dZ));

		cout << m_iDistances.at(camNum) << endl << endl;
	}

	return true;
}

void CameraCalibration::showDistances()
{
	for (int camNum = 0; camNum < i_numCameras; camNum++) 
	{
		cout << "Camera " << camNum << ": " << m_iDistances.at(camNum) << endl;
	}
}

void CameraCalibration::getFilepaths()
{
	for (int camNum = 0; camNum < i_numCameras; camNum++)
	{
		cout << "Camera " << camNum << ": " << rgsCalibFilePaths.at(camNum) << endl;
	}
}

void CameraCalibration::getCamMatrix() 
{
	cv::Mat error = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	CameraParameters params;
	params.Read(rgsCalibFilePaths.at(0), true);
}

camPos CameraCalibration::checkerboardPnP(cv::Mat& cboardPic, cv::Mat& cboard3DPts, string& calibFilepath)
{

	camPos position;
	position.rotation = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	position.translation = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	cv::Mat detectedCorners;
	if (!cv::findChessboardCorners(cboardPic, cv::Size(icbX, icbY), detectedCorners))
	{
		cout << "ERROR: Cannnot detect chessboard corners" << endl;
		return position;
	}

	CameraParameters params;
	if(!params.Read(calibFilepath)) return position;

	cv::Mat camRot = (cv::Mat_<double>(3, 1) << 0, 0, 0);
	cv::Mat camTrans = (cv::Mat_<double>(3, 1) << 0, 0, 0);

	cv::solvePnP(cboard3DPts, detectedCorners, params.getIntrinsic(), params.getDistortions(), camRot, camTrans, true);

	cv::Mat R;
	cv::Rodrigues(camRot, R);

	R = R.t().inv();

	camTrans = -R * camTrans;

	cout << camRot << endl;
	cout << camTrans << endl << endl;

	cout << sqrt((camTrans.at<double>(0) * camTrans.at<double>(0)) + (camTrans.at<double>(1) * camTrans.at<double>(1)) + (camTrans.at<double>(2) * camTrans.at<double>(2))) << endl << endl;

	position.rotation = camRot;
	position.translation = camTrans;

	return position;
}