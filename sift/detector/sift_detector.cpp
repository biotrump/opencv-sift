/*
import cv2
import numpy as np

img = cv2.imread('home.jpg')
gray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT()
kp = sift.detect(gray,None)

img=cv2.drawKeypoints(gray,kp)
cv2.imwrite('sift_keypoints.jpg',img)

img=cv2.drawKeypoints(gray,kp,flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
cv2.imwrite('sift_keypoints.jpg',img)
*/
//  demo.cpp
//
//	Here is an example on how to use the descriptor presented in the following paper:
//	A. Alahi, R. Ortiz, and P. Vandergheynst. FREAK: Fast Retina Keypoint. In IEEE Conference on Computer Vision and Pattern Recognition, 2012.
//  CVPR 2012 Open Source Award winner
//
//	Copyright (C) 2011-2012  Signal processing laboratory 2, EPFL,
//	Kirell Benzi (kirell.benzi@epfl.ch),
//	Raphael Ortiz (raphael.ortiz@a3.epfl.ch),
//	Alexandre Alahi (alexandre.alahi@epfl.ch)
//	and Pierre Vandergheynst (pierre.vandergheynst@epfl.ch)
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
//  This software is provided by the copyright holders and contributors "as is" and
//  any express or implied warranties, including, but not limited to, the implied
//  warranties of merchantability and fitness for a particular purpose are disclaimed.
//  In no event shall the Intel Corporation or contributors be liable for any direct,
//  indirect, incidental, special, exemplary, or consequential damages
//  (including, but not limited to, procurement of substitute goods or services;
//  loss of use, data, or profits; or business interruption) however caused
//  and on any theory of liability, whether in contract, strict liability,
//  or tort (including negligence or otherwise) arising in any way out of
//  the use of this software, even if advised of the possibility of such damage.

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;

static void help( char** argv )
{
    std::cout << "\nUsage: " << argv[0] << " [path/to/image1] [path/to/image2] \n"
              << "This is an example on how to use the keypoint descriptor presented in the following paper: \n"
              << "A. Alahi, R. Ortiz, and P. Vandergheynst. FREAK: Fast Retina Keypoint. \n"
              << "In IEEE Conference on Computer Vision and Pattern Recognition, 2012. CVPR 2012 Open Source Award winner \n"
              << std::endl;
}

int main( int argc, char** argv ) {
    // check http://docs.opencv.org/doc/tutorials/features2d/table_of_content_features2d/table_of_content_features2d.html
    // for OpenCV general detection/matching framework details

    if( argc != 2 ) {
        help(argv);
        return -1;
    }

    // Load images
    Mat imgA = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    if( !imgA.data ) {
        std::cout<< " --(!) Error reading image " << argv[1] << std::endl;
        return -1;
    }

    std::vector<KeyPoint> keypoints1, keypoints2;
    
    // DETECTION
    // Any openCV detector such as
    SurfFeatureDetector detectorSurf(2000,4);
	SiftFeatureDetector detectorSift;
	//OrbFeatureDetector detector(400);
	//FastFeatureDetector detector(10);
    
    // detect
    double t = (double)getTickCount();
    detectorSift.detect( imgA, keypoints1);
    t = ((double)getTickCount() - t)/getTickFrequency();
    std::cout << "SIFT detection time [s]: " << t/1.0 << std::endl;
	//-- Draw keypoints
	Mat imgKeypoint1;
    cv::drawKeypoints(imgA, keypoints1, imgKeypoint1, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("SIFT keypoint", imgKeypoint1);

    // detect
    t = (double)getTickCount();
    detectorSurf.detect( imgA, keypoints2);
    t = ((double)getTickCount() - t)/getTickFrequency();
    std::cout << "SURF detection time [s]: " << t/1.0 << std::endl;
	//-- Draw keypoints
	Mat imgKeypoint2;
    cv::drawKeypoints(imgA, keypoints2, imgKeypoint2, Scalar::all(-1), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    imshow("SURF keypoint", imgKeypoint2);
	waitKey(0);
}
