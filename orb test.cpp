#include <stdio.h>
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\aruco.hpp>
#include "opencv2/imgproc/types_c.h"
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main()
{
	Mat markerImage, frameImage;
	Mat markerGrayImage, frameGrayImage;
	Mat keypointsMarkerImage, keypointsFrameImage;

	VideoCapture cap;

	/*create*/
	Ptr<ORB>orbF = ORB::create();

	vector<KeyPoint> markerKeypoints, frameKeypoints;
	Mat markerDescriptors, frameDescriptors;
	

	bool WebCamFlag = true;																		// Choosing video from WebCam or files
	markerImage = imread("h1.jpg", IMREAD_ANYCOLOR); 

	if (markerImage.empty())
		return -1;

	if (WebCamFlag)
		cap.open(0);																			// Access webCam
	else
		cap.open("scaleRotation.avi"); // 파일명 바꾸기


	if (!cap.isOpened())																		//if not success to open video, exit program
	{
		cout << "Cannot open the video file" << endl;
		return -1;
	}

	/*writing video file
	VideoWriter outputVideo;
	const string NAME = "Output.avi";
	int ex = -1;																				// Get Codec Type- Int form

	char EXT[] = { (char)(ex & 0XFF), (char)((ex & 0XFF00) >> 8), 
		(char)((ex & 0XFF0000) >> 16), (char)((ex & 0XFF000000) >> 24), 0 };					// Transform from int to char via Bitwise operators

	Size S = Size((int)cap.get(CAP_PROP_FRAME_WIDTH), (int)cap.get(CAP_PROP_FRAME_HEIGHT));		// Acquire input size
	outputVideo.open(NAME, ex, 12, S, true);													// Open the output
																								// cap.get(CV_CAP_PROP_FPS)
	if (!outputVideo.isOpened())
	{
		cout << "Could not open the output video for write: " << endl;
		return -1;
	}
	*/

	double fps = cap.get(CAP_PROP_FPS);															// get the frames per seconds of the video

	cout << "Frame per seconds : " << fps << endl;

	/* #1. Marker feature extraction */

	cvtColor(markerImage, markerGrayImage, CV_RGB2GRAY);										// change markerImage to grayscale 
	orbF->detectAndCompute(markerGrayImage, noArray(), markerKeypoints, markerDescriptors);		// detect and compute
	drawKeypoints(markerImage, markerKeypoints, keypointsMarkerImage);							// draw keypoint
	imshow("Detected Marker Features", keypointsMarkerImage);									// show keypoint

	while (1)
	{
		cap >> frameImage;
		cvtColor(frameImage, frameGrayImage, CV_RGB2GRAY);
		orbF->detectAndCompute(frameGrayImage, noArray(), frameKeypoints, frameDescriptors);
		drawKeypoints(frameImage, frameKeypoints, keypointsFrameImage);							// draws keypoints on image and stores result in matric showkeypoints
		imshow("Detected Frame Features", keypointsFrameImage);


		/* #2. Match */

		vector<DMatch> matches, good_matches;
		Mat img_matches;
		BFMatcher matcher(NORM_HAMMING);
		matcher.match(markerDescriptors, frameDescriptors, matches);


		double max_dist = 0;
		double min_dist = 100;

		for (const auto& match : matches)
		{
			double dist = match.distance;
			if (dist < min_dist) min_dist = dist;
			if (dist > max_dist) max_dist = dist;
		}

		cout << "min dist" << min_dist << endl;
		cout << "max dist" << max_dist << endl;

		double fth = 4 * min_dist;
		for (const auto& match : matches)
		{
			if (match.distance <= max(fth, 0.02))
			{
				good_matches.push_back(match);
			}
		}

		cout << "good matches.size()" << good_matches.size() << endl;

		drawMatches(markerImage, markerKeypoints, frameImage, frameKeypoints, good_matches, img_matches,
			Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

		// find Homography between markerKeypoints and frameKeypoints
		vector<Point2f>obj;
		vector<Point2f>scene;
		for (const auto& good_match : good_matches)
		{
			obj.push_back(markerKeypoints[good_match.queryIdx].pt);
			scene.push_back(frameKeypoints[good_match.queryIdx].pt);
		}
		Mat H = findHomography(obj, scene, RANSAC);

		//레퍼런스 이미지로부터 각 코너점들을 얻는다. 
		vector<Point2f> obj_corners(4);
		obj_corners[0] = Point2f(0, 0);
		obj_corners[1] = Point2f(markerImage.cols, 0);
		obj_corners[2] = Point2f(markerImage.cols, markerImage.rows);
		obj_corners[3] = Point2f(0, markerImage.rows);

		//cam위에 객체 검출된 사각형을 그린다. 
		vector<Point2f> scene_corners(4);
		perspectiveTransform(obj_corners, scene_corners, H);

		for (int i = 0; i < 4; i++)
			scene_corners[i] += Point2f(markerImage.cols, 0);
		for (int i = 0; i < 4; i++)
			line(img_matches, scene_corners[i], scene_corners[(i + 1) % 4], Scalar(255, 0, 0), 4);
		//line(img_matches, scene_corners[0] + Point2f(markerImage.cols, 0), scene_corners[1] + Point2f(markerImage.cols, 0), Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[1] + Point2f(markerImage.cols, 0), scene_corners[2] + Point2f(markerImage.cols, 0), Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[2] + Point2f(markerImage.cols, 0), scene_corners[3] + Point2f(markerImage.cols, 0), Scalar(0, 255, 0), 4);
		//line(img_matches, scene_corners[3] + Point2f(markerImage.cols, 0), scene_corners[0] + Point2f(markerImage.cols, 0), Scalar(0, 255, 0), 4); 
	
		imshow("Matches", img_matches);
	

		/* #3. Pose Estimation */

		// matching pairs
		vector<Point3f> objectPoints;	// 3d world coordinates
		vector<Point2f> imagePoints;	// 2d image coordinates
		double fx = 4104.012, cx = 386.934, fy = 1189.039, cy = 225.349;
		double k1 = -1.627789, k2 = 7.985848, p1 = 0.632560, p2 = -0.478797;

		// camera parameters
		double m[] = { fx, 0, cx, 0, fy, cy, 0, 0, 1 };		// intrinsic parameters
		Mat cameraIntrinsic(3, 3, CV_64FC1, m); // camera matrix. 3x3 1ch floating 

		double d[] = { k1, k2, p1, p2 };	// k1, k2 : radial distortion. p1, p2 : tangential distortion
		Mat cameraDistortionCoefficient(4, 1, CV_64FC1, d);

		for (const auto& match : good_matches) // 이 부분을 잘 모르겠음
		{
			imagePoints.push_back(frameKeypoints[match.trainIdx].pt);
			objectPoints.push_back(Point3f(markerKeypoints[match.queryIdx].pt.x, 0.0f, markerKeypoints[match.queryIdx].pt.y));
		}

		Mat rotationVector, translationVector;
		solvePnP(objectPoints, imagePoints, cameraIntrinsic, cameraDistortionCoefficient, rotationVector, translationVector);
		
		cout << "rotation_vector" << endl << rotationVector << endl;
		cout << "translation_vector" << endl << translationVector << endl;

		aruco::drawAxis(frameImage, cameraIntrinsic, cameraDistortionCoefficient, rotationVector, translationVector, 1.0);		//draw coordinate system


		//outputVideo << keypointsFrameImage;

		if (waitKey(30) == 27)																									//wait for 'esc' key press for 30 ms. If 'esc' key is pressed, break loop
		{
			break;
			return 0;
		}
		

	}

}