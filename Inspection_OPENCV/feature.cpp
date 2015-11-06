#pragma once
#include "StdAfx.h"
#include <featureAPI.h>
#include <opencv2\contrib\contrib.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <dmtxreadAPI.h>
#include <logCollection.h>
using namespace cv;
using namespace std;
using namespace FG;


featureAPI::featureAPI(){
	debug = false ;
	SaveImage = true;
	CamTune = false;
}
featureAPI::featureAPI(bool flag){
	debug = flag ;
	CamTune = false;
}
featureAPI::~featureAPI(){
}
static void MarshalString ( System::String ^ s, string& os ) {
	using namespace System;
	using namespace Runtime::InteropServices;
	const char* chars = 
		(const char*)(Marshal::StringToHGlobalAnsi(s)).ToPointer();
	os = chars;
	Marshal::FreeHGlobal(IntPtr((void*)chars));
}

Vcircle featureAPI::findLargeCircle(Mat *image , int minDist , int minRadius  , int maxRadius  , int param1 , int param2){
	Mat cimg = image->clone();
    vector<Vec3f> circles;
	Vcircle vc ;
    HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, minDist,
                 param1, param2, minRadius, maxRadius // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles <165-180> <15-30>
                 );
	if (circles.size() == 0) {
			vc.c = Point2d(-1.0,-1.0);
		    vc.r = 0;
		return vc;
	}
	if (debug) {
		cvtColor( cimg, cimg, CV_GRAY2BGR );
	}
    Vec3d c1 ;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3d c = circles[i];
		if (debug)
			circle(cimg, Point2d(c[0],c[1]), cvRound(c[2]), Scalar(0, 255, 0), 1, CV_AA);
		c1[0] += c[0];
        c1[1] += c[1];
        c1[2] += c[2];
    }
    if (circles.size() != 0) {
        c1[0] /= circles.size() ;
		c1[1] /= circles.size() ;
		c1[2] /= circles.size() ;
		if (debug) {
			
			circle(cimg, Point2d(c1[0],c1[1]), cvRound(c1[2]), Scalar(0, 255, 255), 1, CV_AA);
			circle(cimg, Point2d(c1[0],c1[1]), 1, Scalar(255, 255, 0), -1, CV_AA);
			//putText(cimg , "test" ,Point2d(c1[0],c1[1]) ,FONT_HERSHEY_SIMPLEX ,3.0 ,Scalar(255, 255, 0) ,1 ,8 );
			//putText(Mat& img, const string& text, Point org, int fontFace, double fontScale, Scalar color, int thickness=1, int lineType=8, bool bottomLeftOrigin=false )
		}
    }
	if (debug) {
		namedWindow("detected circle",0);
		imshow("detected circle", cimg);
	}
    cimg.release();
	vc.c = Point2d(c1[0]  , c1[1] );
	vc.r = c1[2];
    return vc;
}
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i < j );
}
bool compareCircleR (VCcircle vc1, VCcircle vc2 ) {
	return (vc1.Vc.r < vc2.Vc.r);
}
bool compareCircleVote (VoteVcircle vc1, VoteVcircle vc2 ) {
	return (vc1.vote > vc2.vote);
}
Point2d featureAPI::findContourMinArea(Mat *image ){
	Mat cimg; 
	image->copyTo(cimg);
	vector<vector<Point> >  contours;
	vector<Vec4i> hierarchy;
	//CV_RETR_CCOMP  CV_RETR_TREE
	findContours(cimg, contours, hierarchy, CV_RETR_TREE , CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
	if (contours.size() == 0) return Point2d(-1,-1);
	double area =0.0 ;
	int maxContourIndex = 0 , secxContourIndex =0;
	vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
	cvtColor( cimg, cimg, CV_GRAY2BGR );
	std::sort(contours.begin(), contours.end(), compareContourAreas);
	for( size_t i = (contours.size() > 2 ? contours.size() -2 : 0)  ; i< contours.size(); i++ )
     {
	   Point2f center;
       float radius = 0; 
	   RNG rng(12345); 
	   approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
       boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	   if (debug) {
		   Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		   drawContours( cimg, contours, (int)i, color, 1 , CV_AA, hierarchy,1, Point() );
		   //minEnclosingCircle(contours[i], center, radius);
		   //circle(cimg, center, cvRound(radius), Scalar(0, 255, 255), 1, CV_AA);
		   rectangle( cimg, boundRect[i].tl(), boundRect[i].br(), color, 1, CV_AA, 0 );
	   }
     }
	if (debug){
		namedWindow("detected1",0);
		imshow("detected1", cimg);
	}
	cimg.release();
	if (contours.size() < 2) 
		return Point2d(boundRect[contours.size()-1].x + boundRect[contours.size()-1].width /2 , boundRect[contours.size()-1].y + boundRect[contours.size()-1].height/2);
	return Point2d(boundRect[contours.size()-2].x + boundRect[contours.size()-2].width /2 , boundRect[contours.size()-2].y + boundRect[contours.size()-2].height/2);
}
Point2d featureAPI::findoffset(Mat *image , int vThreshold , int minDist , int minRadius  , int maxRadius ,int param1 , int param2){
	 Mat tmpimage ;
	 image->copyTo(tmpimage);
	 GaussianBlur( *image, *image, Size(3, 3), 3, 3 );
	 threshold( *image, *image, vThreshold, 255,1 );
	 Mat element = getStructuringElement( 2, Size( 1, 1 ), Point( 0, 0 ) );
	 /// Apply the specified morphology operation
	 //morphologyEx( *image, *image, 2, element );
	 //Canny(*image, *image, vThreshold, 200, 3);
	 if (debug) {
		 namedWindow("threshold",0);
		 imshow("threshold", *image);
	 }
	 Vcircle tmp2d = findLargeCircle(image, minDist ,minRadius ,maxRadius,param1,param2); 
	 Point2d Center = findContourMinArea(image);
	// if (tmp2d.c.x == -1.0 && tmp2d.c.y ==-1.0) return Point2d(-1,-1);
	 if(SaveImage) {
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, Center, cvRound(tmp2d.r), Scalar(0, 255, 255), 1, CV_AA);
		  circle(tmpimage, Center, 1, Scalar(255, 255, 0), -1, CV_AA);
		  System::String ^ ImgPath = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\Scara_Vision\\"+System::DateTime::Now.ToString("MMddHHmmss")+"_CarrierMark.bmp";
		  std::string ImagePath ;
		  MarshalString( ImgPath ,ImagePath);
		  imwrite (ImagePath, tmpimage);
	 }

	 tmpimage.release();
	 return Center;
}
Point2d featureAPI::Calibration(Mat *image , int vThreshold , int minDist , int minRadius  , int maxRadius ,int param1 , int param2){
	 Mat tmpimage ;
	 image->copyTo(tmpimage);
	 threshold( *image, *image, vThreshold, 255,1 );
	 if (debug) {
		 namedWindow("threshold",0);
		 imshow("threshold", *image);
	 }
	 Vcircle tmp2d = findLargeCircle(image, minDist ,minRadius ,maxRadius,param1,param2); 
	 Point2d Center = 0;
	 if ( (tmp2d.c.x == -1) && (tmp2d.c.y == -1) )
		Center = findContourMinArea(image);
	 else
		Center = Point2d(cvRound(tmp2d.c.x) , cvRound(tmp2d.c.y));

	 if(SaveImage) {
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, Center, cvRound(tmp2d.r), Scalar(0, 255, 255), 1, CV_AA);
		  circle(tmpimage, Center, 1, Scalar(255, 255, 0), -1, CV_AA);
		  System::String ^ ImgPath = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\Carrier_calibration\\"+System::DateTime::Now.ToString("mmss_fff")+"_Calibration.bmp";
		  std::string ImagePath ;
		  MarshalString( ImgPath ,ImagePath);
		  imwrite (ImagePath, tmpimage);
	 }

	 tmpimage.release();
	 return Center;
}
vector<CheckPoint> featureAPI::findoffset(Mat *image , int vThreshold , int minDist , int minRadius  , int maxRadius , int minDist2 , int minRadius2  , int maxRadius2 ,int pixel, int param1 , int param2 ) {
	 Mat binary;
	 vector<CheckPoint> vpp ;
	 threshold( *image, *image, vThreshold, 255,1 );
	 GaussianBlur( *image, *image, Size(3, 3), 3, 3 );
	 Vcircle tmp2d = findLargeCircle(image, minDist ,minRadius ,maxRadius,param1,param2); // i
	 if (tmp2d.c.x == -1.0 && tmp2d.c.y ==-1.0) return vpp;
	 Point2i t = Point2i(cvRound(tmp2d.c.x) , cvRound(tmp2d.c.y));

	 Vcircle tmp2o = findLargeCircle(image, minDist2 ,minRadius2 ,maxRadius2,param1,param2); // o
	 if (tmp2o.c.x == -1.0 && tmp2o.c.y ==-1.0) return vpp;
	 Point2i g = Point2i(cvRound(tmp2d.c.x) , cvRound(tmp2d.c.y));
	  CheckPoint pp ;
	  pp.upY2 = t.x;
	  pp.upY1 =t.y;
	  pp.downY2 = g.x;
	  pp.downY1 = g.y;
	  vpp.push_back(pp); 
	 binary.release();
	 return vpp;

}

void featureAPI::rotImage(Mat *image , int angle) {
	Point2d center;
	Mat rot_mat(2,3,CV_8UC1);
	Mat tmp;
  /// Rotate the warped image
	switch (angle) {
	case 1: // 90
		transpose(*image, tmp);
		tmp.copyTo(*image);
		flip(*image, *image, 1);
		break;
	case 2: //180
		/// Compute a rotation matrix with respect to the center of the image
		center = Point2d( image->cols/2, image->rows/2 );
		/// Get the rotation matrix with the specifications above
		rot_mat = getRotationMatrix2D( center, 180.0, 1.0 );
		warpAffine( *image, *image, rot_mat, image->size());
		break;
	case 3: //270
		/// Compute a rotation matrix with respect to the center of the image
		center = Point2d( image->cols/2, image->rows/2 );
		/// Get the rotation matrix with the specifications above
		rot_mat = getRotationMatrix2D( center, 180.0, 1.0 );
		warpAffine( *image, *image, rot_mat, image->size());
		transpose(*image, tmp);
		tmp.copyTo(*image);
		flip(*image, *image, 1);
		break;
	}
	if (debug) {
		namedWindow("rotImage",0);
		imshow("rotImage", *image);
	}
	rot_mat.release();
	tmp.release();
}
void featureAPI::cropImage(Mat *image , Mat *outImage, int x ,int y ,int width , int  height ){
	x = x >= 0 ? x : 0;
	y = y >= 0 ? y : 0 ;
	width = x+width > image->cols ? image->cols - x -1 :width ;
	height = y+height > image->rows ? image->rows - y -1 :height ;
	//if (x + width > image->cols || y+ height > image->rows) {
	//	outImage->create(image->size() , image->type());
	//	Mat(*image,Rect(x, y, width, height)).copyTo(*outImage);
	//}else {
		outImage->create(Size(width, height) , image->type());
		Mat(*image,Rect(x, y, width, height)).copyTo(*outImage);
	//}
	if (debug) {
		namedWindow("cropImage",0);
		imshow("cropImage", *outImage);
	}
}

Point2d featureAPI::getMarkPoint(Mat *srcimage , Mat *tmpimage, int maxlevel , double matcherThreshold) {
	std::vector<cv::Mat> refs, tpls, results;
	Mat tmprefs ,tmptpls ;
	bool result = 0 ;
    // Build Gaussian pyramid
	resize (*srcimage , tmprefs , Size(), 0.5 , 0.5 ,INTER_LINEAR );
	resize (*tmpimage , tmptpls , Size(), 0.5 , 0.5 ,INTER_LINEAR );
    cv::buildPyramid(tmprefs, refs, maxlevel);
    cv::buildPyramid(tmptpls, tpls, maxlevel);

    cv::Mat ref, tpl, res;
	cv::Point maxresult ;
    // Process each level
    for (int level = maxlevel; level >= 0; level--)
    {
        ref = refs[level];
        tpl = tpls[level];
        res = cv::Mat::zeros(ref.size() + cv::Size(1,1) - tpl.size(), CV_32FC1);

        if (level == maxlevel)
        {
            // On the smallest level, just perform regular template matching
            cv::matchTemplate(ref, tpl, res, CV_TM_CCORR_NORMED);
        }
        else
        {
            // On the next layers, template matching is performed on pre-defined 
            // ROI areas.  We define the ROI using the template matching result 
            // from the previous layer.

            cv::Mat mask;
            cv::pyrUp(results.back(), mask);

            cv::Mat mask8u;
            mask.convertTo(mask8u, CV_8U);

            // Find matches from previous layer
            std::vector<std::vector<cv::Point> > contours;
            cv::findContours(mask8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

            // Use the contours to define region of interest and 
            // perform template matching on the areas
            for (size_t i = 0; i < contours.size(); i++)
            {
                cv::Rect r = cv::boundingRect(contours[i]);
                cv::matchTemplate(
                    ref(r + (tpl.size() - cv::Size(1,1))), 
                    tpl, 
                    res(r), 
                    CV_TM_CCORR_NORMED
                );
            }
        }

        // Only keep good matches
        cv::threshold(res, res, (matcherThreshold-0.1) <=0 ? 0.1 : (matcherThreshold-0.1) , 1., CV_THRESH_TOZERO);
        results.push_back(res);
    }
	//double matcherThreshold =0.9;
	while (true)
    {
        double minval, maxval;
		cv::Point minloc, maxloc;
        cv::minMaxLoc(res, &minval, &maxval, &minloc, &maxloc);
        if (maxval >= matcherThreshold)
        {
            cv::floodFill(
                res, maxloc, 
                cv::Scalar(0), 0, 
                cv::Scalar(.1), 
                cv::Scalar(1.)
            );
			matcherThreshold = maxval ;
			maxresult = maxloc ;
			result = true ;
			break ;
        }
        else {
			result = false ;
            break;
		}
    }
	if (debug || SaveImage) {
		cvtColor( ref, ref, CV_GRAY2BGR );
		cv::rectangle(
        ref, maxresult, 
        cv::Point(maxresult.x + tpl.cols, maxresult.y + tpl.rows), 
        CV_RGB(0,255,0), 2
        );
		if (debug){
			namedWindow("result", CV_WINDOW_NORMAL );
			imshow("result", ref);
		}
		if (SaveImage){
			System::String ^ ImgPath2 = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\XYZ_Vision\\";
    		System::String ^ ImgPath = ImgPath2+System::DateTime::Now.ToString("MMddhhmmss")+"_TrayMark.bmp";
			if ( ! System::IO::Directory::Exists( ImgPath2 ) )
				System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath2 );
			std::string ImagePath ;
			MarshalString( ImgPath ,ImagePath);
			imwrite (ImagePath, ref);
		}
	}
	ref.release();
	if (maxresult.x == 0 && maxresult.y == 0 || result == false)
		return Point2d(-1, -1);
	return Point2d(2*(maxresult.x +  tpl.cols/2 ), 2*( maxresult.y + tpl.rows/2));
}
Point2d featureAPI::getMarkPoint(Mat *srcimage , Mat *tmpimage){
  Mat img_object ;
  Mat img_scene  ;
  //resize (*tmpimage , img_object , Size(), 1 , 1 ,INTER_LINEAR );
  //resize (*srcimage , img_scene , Size(), 1 , 1 ,INTER_LINEAR );
  tmpimage->copyTo(img_object);
  cropImage(srcimage , &img_scene ,1200 ,1000 , 400 ,300);

  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  SurfFeatureDetector detector( minHessian );

  vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );

  //-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor ;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < ( min_dist ? 1.5* min_dist : max_dist /3 ))
    { good_matches.push_back( matches[i]); }
  }
  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
  //-- Localize the object from img_1 in img_2
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }
  
  Mat H = findHomography( obj, scene, CV_RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = cvPoint(0,0); 
  obj_corners[1] = cvPoint( img_object.cols, 0 );
  obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); 
  obj_corners[3] = cvPoint( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);
  /*
  line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
  */
  Point2d markpoint(0,0) ;
  for (size_t i = 0 ; i < scene_corners.size() ; i++) {
	 markpoint.x += scene_corners[i].x ;
	 markpoint.y += scene_corners[i].y ;
  }
   markpoint.x /= scene_corners.size();
   markpoint.y /= scene_corners.size();
   //markpoint.x *= 2;
   //markpoint.y *= 2;
  //namedWindow("detected circles-3",CV_WINDOW_NORMAL);
  // imshow("detected circles-3", img_scene);
  return markpoint;
}
bool featureAPI::MatchingMethod(Mat *srcimage , Mat *tmpimage){
	Mat tmprefs ,tmptpls ,res;
    // Build Gaussian pyramid
	resize (*srcimage , tmprefs , Size(), 0.5 , 0.5 ,INTER_LINEAR );
	resize (*tmpimage , tmptpls , Size(), 0.5 , 0.5 ,INTER_LINEAR );
	res = cv::Mat::zeros(tmprefs.size() + cv::Size(1,1) - tmptpls.size(), CV_32FC1);
	matchTemplate( *srcimage, *tmpimage, res, CV_TM_CCOEFF_NORMED );
	double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    minMaxLoc( res, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
	if (maxVal > 0.8) 
		return true ;
	else 
		return false;
}
int featureAPI::checkFlex(Mat *image ,int x ,int y , int w, int h  ,int minDist , int minRadius  , int maxRadius , int param1 , int param2 
																						,int minDist2 , int minRadius2  , int maxRadius2 , int param21 ,int param22
																						,int minDist3 , int minRadius3  , int maxRadius3 , int param31 ,int param32)
{

	 Mat binary;
	 Mat SaveImg;
	 System::String ^ FileName ;

	 System::String ^ ImgPath  = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\";
	 System::String ^ ImgPath2 = ImgPath + "Scara_Vision\\";//ImgPath4 + System::DateTime::Now.ToString(" mm_ss_fff");
	 System::String ^ ImgPath3 = ImgPath2 + "Pass\\";//ImgPath + "Scara_Vision\\";
	 System::String ^ ImgPath4 = ImgPath2 + "Fail\\";
	 System::String ^ ImgPath5 = System::DateTime::Now.ToString("mm_ss_fff");
	 System::String ^ Barcode = "";

	 std::string ImagePath ;
	 std::string P_ImagePath ;
	 std::string F_ImagePath ;

	 if ( ! System::IO::Directory::Exists( ImgPath ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath );

	 if ( ! System::IO::Directory::Exists( ImgPath2 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath2 );

	 if ( ! System::IO::Directory::Exists( ImgPath3 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath3 );

	 if ( ! System::IO::Directory::Exists( ImgPath4 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath4 );

	 if (CamTune){
		 MarshalString(ImgPath2 + "_Original.bmp",ImagePath);
		 imwrite (ImagePath , *image);
	 }

	 if (debug || SaveImage) {
		 BarcodeAPI * barcodeapi = new BarcodeAPI();
		 cropImage(image ,&binary, x+750 , y+50 , 220, 220);
		 rotImage(&binary , 1); 
		 FileName  = gcnew System::String(""+ System::DateTime::Now.ToString("MMddhhmmss")+"_" +barcodeapi->decoder(&binary)) ;
		 if (SaveImage){
            Barcode = barcodeapi->decoder(&binary);
			MarshalString( ImgPath2 + ImgPath5 + "_" + Barcode,ImagePath);
			MarshalString( ImgPath3 + ImgPath5 + "_" + Barcode,P_ImagePath);
			MarshalString( ImgPath4 + ImgPath5 + "_" + Barcode,F_ImagePath);
			imwrite (ImagePath + ".bmp", binary);
			cropImage(image ,&SaveImg, x, y , w, h); 
		 }
		 delete barcodeapi;
	 }
	 std::string path ;

	 cropImage(image ,&binary, x+230 , y+25 , w-400, h-50);
	 GaussianBlur( binary, binary, Size(3, 3), 2, 2 );
	 threshold( binary, binary, 135, 255,1);
	 if (debug) {
		 namedWindow("Debugimagethreshold",0);
		 imshow("Debugimagethreshold", binary);
		 MarshalString( FileName + "_CenterBinary" + ".bmp",path);
		 imwrite(path,binary);
	 }
	 Point2d contourCcenter = findContourMinArea(&binary);
	 GaussianBlur( binary, binary, Size(7, 7), 2, 2 );
	 Vcircle tmp2d = findLargeCircle(&binary, minDist ,minRadius ,maxRadius,param1,param2); // co
	 if (tmp2d.c.x == -1.0 && tmp2d.c.y ==-1.0) {
		 if (SaveImage){
			 imwrite(F_ImagePath + "_Center_fail.bmp",SaveImg); 
			 LogCollection::LogCWriteLine(FileName +",Center_fail,,,,,NA");
		 }
		 SaveImg.release();
		 binary.release() ;  
		 return CENTERERROR; 
	 }
	 if(debug) {
		  Mat tmpimage ;
		  cropImage(image ,&tmpimage, x+230 , y+25 , w-400, h-50);
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, tmp2d.c, cvRound(tmp2d.r), Scalar(0, 255, 255), 1, CV_AA);
		  circle(tmpimage, tmp2d.c, 1, Scalar(255, 255, 0), -1, CV_AA);
		  circle(tmpimage, contourCcenter, 1, Scalar(0, 255, 0), -1, CV_AA);
		  //imwrite("DebugimageCenter.bmp",tmpimage);
		  namedWindow("DebugimageCenter",0);
		  imshow("DebugimageCenter", tmpimage);
		  tmpimage.release();
	 }
	 contourCcenter.x += x+230;
	 contourCcenter.y += y+25;
	 tmp2d.c.x+=x+230 ; 
	 tmp2d.c.y+=y+25;
	 x = contourCcenter.x - w / 2;
	 y = contourCcenter.y - h / 2;
	 cropImage(image ,&binary, x , y+100 ,w-600 ,h-200);
	 threshold( binary, binary, 35, 255,0);
	 if (debug) {
		 namedWindow("Debugimagethreshold1",0);
		 imshow("Debugimagethreshold1", binary);
		 MarshalString( FileName + "_LeftBinary" + ".bmp",path);
		 imwrite(path,binary);
	 }
	 GaussianBlur( binary, binary, Size(7, 7), 0, 0 );
	 Vcircle tmp2o = findLargeCircle(&binary, minDist2 ,minRadius2 ,maxRadius2,param21,param22); // lo
	 if (tmp2o.c.x == -1.0 && tmp2o.c.y ==-1.0) {
		 if (SaveImage){
			 imwrite(F_ImagePath + "_Left_fail.bmp",SaveImg); 
		     LogCollection::LogCWriteLine(FileName +",Left_fail,,,,,NA");
		 }
		 SaveImg.release();
		 binary.release() ;  
		 return LEFTERROR; 
	 }
	 Mat contourLimage ;
	 cropImage(&binary ,&contourLimage, (int)(tmp2o.c.x-cvRound(tmp2o.r) - (maxRadius2-minRadius2)) , (int)(tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2)), (int)(maxRadius2 + maxRadius2-tmp2o.r)*2.3 ,(int)(maxRadius2 + maxRadius2-tmp2o.r)*2.3);
	 threshold( contourLimage, contourLimage, 35, 255,0);
	 Point2d contourLeftCenter = findContourMinArea(&contourLimage);
	 contourLimage.release();

	 if(debug) {
		  Mat tmpimage ;
		  cropImage(image ,&tmpimage, x , y+100 ,w-600 ,h-200);
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, tmp2o.c, cvRound(tmp2o.r), Scalar(0, 255, 255), 1, CV_AA);
		  circle(tmpimage, tmp2o.c, 1, Scalar(255, 255, 0), -1, CV_AA);
		  circle(tmpimage, Point2d(contourLeftCenter.x+tmp2o.c.x-cvRound(tmp2o.r) -(maxRadius2-minRadius2) , contourLeftCenter.y +tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2)), 1, Scalar(0, 0, 255), -1, CV_AA);
		  //imwrite("DebugimageLeft.bmp",tmpimage);
		  namedWindow("DebugimageLeft",0);
		  imshow("DebugimageLeft", tmpimage);
		  tmpimage.release();
	 }
	 contourLeftCenter.x += x + tmp2o.c.x-cvRound(tmp2o.r) -(maxRadius2-minRadius2) ;
	 contourLeftCenter.y += y+100  + tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2);
	 tmp2o.c.x+=x ; 
	 tmp2o.c.y+=y+100;
	 cropImage(image ,&binary, x+600 , y+100 ,w-600 ,h-200);
	 threshold( binary, binary, 40, 255,0 );
	 if (debug) {
		 namedWindow("Debugimagethreshold2",0);
		 imshow("Debugimagethreshold2", binary);
		 MarshalString( FileName + "_RightBinary" + ".bmp",path);
		 imwrite(path,binary);

	 }
	 GaussianBlur( binary, binary, Size(7, 7), 0, 0 );
	 Vcircle tmp2a = findLargeCircle(&binary, minDist3 ,minRadius3 ,maxRadius3,param31,param32); // ro
	 if (tmp2a.c.x == -1.0 && tmp2a.c.y ==-1.0) {
		 if (SaveImage){
			 imwrite(F_ImagePath + "_Right_fail.bmp",SaveImg); 
			 LogCollection::LogCWriteLine(FileName +",Right_fail,,,,,NA");
		 }
		 SaveImg.release();
		 binary.release() ;  
		 return RIGHTERROR ;
	 }

	 Mat contourRimage ;
	 cropImage(&binary ,&contourRimage, (int)tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) , (int)tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3), (int)(maxRadius3 + maxRadius3-tmp2a.r)*2.5 ,(int)(maxRadius3 + maxRadius3-tmp2a.r)*2.5);
	 threshold( contourRimage, contourRimage, 40, 255,0);
	 Point2d contourRightCenter = findContourMinArea(&contourRimage);
	 contourRimage.release();
	 binary.release();
	 if(debug) {
		  Mat tmpimage ;
		  cropImage(image ,&tmpimage, x+600 , y+100 ,w-600 ,h-200);
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, tmp2a.c, cvRound(tmp2a.r), Scalar(0, 255, 255), 1, CV_AA);
		  circle(tmpimage, tmp2a.c, 1, Scalar(255, 255, 0), -1, CV_AA);
		  circle(tmpimage, Point2d(contourRightCenter.x + tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) , contourRightCenter.y + tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3)), 1, Scalar(0, 0, 255), -1, CV_AA);
		  //imwrite("DebugimageRight.bmp",tmpimage);
		  namedWindow("DebugimageRight",0);
		  imshow("DebugimageRight", tmpimage);
		  tmpimage.release();
	 }
	 contourRightCenter.x += x + 600 + tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) ;
	 contourRightCenter.y += y+100  + tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3);
	 tmp2a.c.x+=x+600 ; 
	 tmp2a.c.y+=y+100;
	 double k =0 , b=0, ax=0 ,ay=0 ,angle =0;
	 if ((contourRightCenter.y - contourLeftCenter.y) == 0 ||  (contourRightCenter.x - contourLeftCenter.x) ==0) {
		 k = 0;
		 ay = b = ((contourLeftCenter.y + contourRightCenter.y) /2) - k * ((contourLeftCenter.x + contourRightCenter.x)/2);
		 ax = ((contourLeftCenter.x + contourRightCenter.x) /2);
		 angle = 0;
	 }else{
		 k =  -1 * (contourRightCenter.y - contourLeftCenter.y) / (contourRightCenter.x - contourLeftCenter.x);
		 angle = System::Math::Atan(k) * 180 /  System::Math::PI ;
		 if (k < 0 ) angle = 180 + angle;
		 k = 1 /k ;
		 b = ((contourLeftCenter.y + contourRightCenter.y) /2) - k * ((contourLeftCenter.x + contourRightCenter.x)/2);
		 ax =  (contourCcenter.x + k * contourCcenter.y - k * b) / (1 + k *k ) ;
		 ay = k * ax + b ;
	 }

	 cropImage(image ,&binary, x+300 , y+100 , w-600, h-150);
	 float theta = findLines(&binary , 48);
	 if (angle > 90 && theta > 90)
		 theta = 180 - (theta - angle);
	 else if ( angle > 90 && theta < 90 )
		 theta = theta - (180 -angle);
	 else if ( angle < 90 && theta > 90)
		 theta = (180 - theta) -angle  ;
	 else 
		 theta = theta -angle ;
	 theta = 180 -theta;
	 double Rtheta = System::Convert::ToDouble(theta);
	 if (Rtheta < 0 )
		Rtheta =  System::Math::Abs(theta);
	 else if (theta > 180)
		Rtheta = Rtheta -180 ;
	 else if (Rtheta > 90 && Rtheta < 180)
		Rtheta = 180 -Rtheta;
	 bool pass = Rtheta > 1.5 ? false :true ; 

	 if (debug || SaveImage) {
		  Mat tmpimage;
		  image->copyTo(tmpimage);
		  std::string pointC ;
		  double fb =0.0 ;
		  fb =  contourCcenter.y - System::Math::Tan(theta * System::Math::PI /180) *contourCcenter.x;
		  cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		  circle(tmpimage, contourCcenter, 1, Scalar(200, 0, 30), -1, CV_AA);
		  circle(tmpimage, contourCcenter, cvRound(tmp2d.r), Scalar(0, 255, 255), 1, CV_AA);
		  MarshalString ( "contourCcenterOfFlex : ( " +contourCcenter.x + " , " + contourCcenter.y + " )" , pointC);
		  putText(tmpimage , pointC ,Point2d(x,y+20) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(200, 0, 30) ,1 ,CV_AA );
		  circle(tmpimage, contourLeftCenter, 1, Scalar(30, 0, 200), -1, CV_AA);
		  circle(tmpimage, contourLeftCenter, cvRound(tmp2o.r), Scalar(0, 255, 255), 1, CV_AA);
		  MarshalString ("contourLeftCcenterOfBracket : ( " +contourLeftCenter.x + " , " + contourLeftCenter.y + " )" , pointC);
		  putText(tmpimage , pointC ,Point2d(x,y+40) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(100, 150, 200) ,1 ,CV_AA );
		  circle(tmpimage, contourRightCenter, 1, Scalar(100, 150, 200), -1, CV_AA);
		  circle(tmpimage, contourRightCenter, cvRound(tmp2a.r), Scalar(0, 255, 255), 1, CV_AA);
		  MarshalString ( "contourRightCcenterOfBracket : ( " +contourRightCenter.x + " , " + contourRightCenter.y + " )" , pointC);
		  putText(tmpimage , pointC ,Point2d(x,y+60) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(0, 30, 200) ,1 ,CV_AA );
		  line(tmpimage, contourLeftCenter , contourRightCenter, Scalar(255, 0, 255), 1, CV_AA);
		  line(tmpimage, contourCcenter , Point2d(contourLeftCenter.x, contourLeftCenter.x * System::Math::Tan(theta * System::Math::PI /180)+fb), Scalar(255, 255, 0), 1, CV_AA);
		  line(tmpimage, contourCcenter , Point2d(contourRightCenter.x, contourRightCenter.x * System::Math::Tan(theta * System::Math::PI /180)+fb), Scalar(255, 255, 0), 1, CV_AA);
		  circle(tmpimage, Point2d(((contourLeftCenter.x + contourRightCenter.x)/2),((contourLeftCenter.y + contourRightCenter.y) /2)), 1, Scalar(125, 125, 200), -1, CV_AA);
		  MarshalString ( "centerOfTwoContoursLineOnBracket : ( " +((contourLeftCenter.x + contourRightCenter.x)/2) + " , " + ((contourLeftCenter.y + contourRightCenter.y) /2)+ " )" , pointC);
		  putText(tmpimage , pointC ,Point2d(x,y+80) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(125, 125, 200) ,1 ,CV_AA );
		  MarshalString ((System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110) ? "Fail" :"Pass", pointC);
		  putText(tmpimage , pointC ,Point2d(x,y+100) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(150, 150, 0) ,1 ,CV_AA );	
		  Mat debugimage ;
		  if (debug){
			  cropImage(&tmpimage ,&debugimage, x , y , w, h);
			  namedWindow("detected offset point",0);
			  imshow("detected offset point", debugimage);
		  }
		  if (SaveImage){

			  cropImage(&tmpimage ,&debugimage, x , y , w, h);

			  if (System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110)
				imwrite (F_ImagePath + ".bmp", debugimage);
			  else
				imwrite (P_ImagePath + ".bmp", debugimage);

		  }
		  if (debug){
			MarshalString (FileName + ".bmp", pointC);
			imwrite(pointC,debugimage);
		  }
		  tmpimage.release();
		  debugimage.release();

		  LogCollection::LogCWriteLine(FileName +
			  "," +contourCcenter.x + "," + contourCcenter.y  + 
			  "," +((contourLeftCenter.x + contourRightCenter.x)/2) + "," + ((contourLeftCenter.y + contourRightCenter.y) /2) +
//			  "," +ax + "," + ay +
//			  "," +contourLeftCenter.x + "," + contourLeftCenter.y +
//			  "," +contourRightCenter.x + "," + contourRightCenter.y +
			  "," + Rtheta +
			  "," + ((System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110) ? "Fail":"Pass"));
	 }
	 
	 SaveImg.release();
	 binary.release();
	 bool result = pass & (System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) < 110);
	 return result ? 1 : ERROR_MV_INSPECTION_FAIL ; 
	 
}

float featureAPI::findLines(Mat *image , int thr){
	Mat tmpImage ;
	//vector<Vec4i> lines;
	image->copyTo(tmpImage);
	GaussianBlur( tmpImage, tmpImage, Size(5, 5), 0, 0 );
	threshold( tmpImage, tmpImage, 245, 255,0);
	Canny(tmpImage, tmpImage, 50, 200, 3);
	vector<Vec2f> lines;
	HoughLines(tmpImage, lines, 1, CV_PI/1800, thr, 0, 0 );
	cli::array<int>^ vote = gcnew cli::array<int> (lines.size());
	int maxVote = 0 , maxIndex = 0;
	float tmpVote;
	if (lines.size() > 0) {
		for( size_t i = 0; i < lines.size(); i++ ){
			float theta = lines[i][1] ;
			vote[i] = 0;
			for (size_t j = i ; j < lines.size() ; j++) {
				if ( lines[j][1] >= theta -0.5 &&  lines[j][1] <= theta+0.5  ) 
					vote[i]++ ;
			}
			if (maxVote < vote[i]) {
				maxIndex = i ;
				maxVote =vote[i];
			}
		}
		tmpVote = 0;
		for (size_t i = 0 ; i < lines.size() ; i++) {
			if ( lines[i][1] >=  lines[maxIndex][1] -0.5 &&  lines[i][1] <= lines[maxIndex][1]+0.5 ){
				tmpVote += lines[i][1];
			}
		}
		if (debug) {
			cvtColor( tmpImage, tmpImage, CV_GRAY2BGR );
			//for( size_t i = 0; i < lines.size(); i++ ){
				float rho = lines[maxIndex][0], theta = tmpVote/vote[maxIndex];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000*(-b));
				pt1.y = cvRound(y0 + 1000*(a));
				pt2.x = cvRound(x0 - 1000*(-b));
				pt2.y = cvRound(y0 - 1000*(a));
				line( tmpImage, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
    	//	}
			namedWindow("DebugimageLine",0);
			imshow("DebugimageLine", tmpImage);
	   }
	  tmpImage.release();
	  return  lines[maxIndex][0] < 0 ? (float) 180.0 - (tmpVote/vote[maxIndex]) : System::Math::Abs(tmpVote/vote[maxIndex]);
	}

	return -255;
}

Point2d featureAPI::CalFindCenter(Mat *image  ) {

	System::String ^ ImgPath = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\Carrier_calibration\\"+System::DateTime::Now.ToString("mmss_fff")+"_CarPaleet.bmp";
	std::string ImagePath ;
	MarshalString( ImgPath ,ImagePath);

	threshold( *image, *image, 40, 255,1);

	if (debug) {
		namedWindow("Debugimagethreshold",0);
		imshow("Debugimagethreshold", *image);
		//MarshalString( FileName + "_CenterBinary" + ".bmp",path);
		//imwrite(path,binary);
	}

	Point2d contourCcenter = findContourMinArea(image);

	if (SaveImage) {
		Mat tmpimage;
		image->copyTo(tmpimage);
		std::string pointC ;
		cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		circle(tmpimage, contourCcenter, 1, Scalar(200, 0, 30), -1, CV_AA);
		circle(tmpimage, contourCcenter, 86, Scalar(0, 255, 255), 1, CV_AA);

		imwrite (ImagePath, tmpimage);
	}
	return contourCcenter;
}

int featureAPI::checkFlexCenter(Mat *image ,int x ,int y , int w, int h  ,int minDist , int minRadius  , int maxRadius , int param1 , int param2 
																						,int minDist2 , int minRadius2  , int maxRadius2 , int param21 ,int param22
																						,int minDist3 , int minRadius3  , int maxRadius3 , int param31 ,int param32) {
	Mat binary;
	Mat SaveImg	;	 
	System::String ^ FileName ;

	 System::String ^ ImgPath  = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\";
	 System::String ^ ImgPath2 = ImgPath + "Scara_Vision\\";//ImgPath4 + System::DateTime::Now.ToString(" mm_ss_fff");
	 System::String ^ ImgPath3 = ImgPath2 + "Pass\\";//ImgPath + "Scara_Vision\\";
	 System::String ^ ImgPath4 = ImgPath2 + "Fail\\";
	 System::String ^ ImgPath5 = System::DateTime::Now.ToString("mm_ss_fff");
	 System::String ^ Barcode = "";

	 std::string ImagePath ;
	 std::string P_ImagePath ;
	 std::string F_ImagePath ;

	Vcircle tmp2d ;
	tmp2d.c.x = 1; 
	tmp2d.c.y = 1;
	tmp2d.r = 1;

	 if ( ! System::IO::Directory::Exists( ImgPath ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath );

	 if ( ! System::IO::Directory::Exists( ImgPath2 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath2 );

	 if ( ! System::IO::Directory::Exists( ImgPath3 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath3 );

	 if ( ! System::IO::Directory::Exists( ImgPath4 ) )
		 System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath4 );

	if (CamTune){
		MarshalString(ImgPath2 + "_Original.bmp",ImagePath);
		imwrite (ImagePath , *image);
	}
	if (debug || SaveImage) {
		BarcodeAPI * barcodeapi = new BarcodeAPI();
		cropImage(image ,&binary, x+750 , y+50 , 220, 220);
		rotImage(&binary , 1); 
		FileName  = gcnew System::String(""+ System::DateTime::Now.ToString("MMddhhmmss")+"_" +barcodeapi->decoder(&binary)) ;
		if (SaveImage){
			Barcode = barcodeapi->decoder(&binary);
			MarshalString( ImgPath2 + ImgPath5 + "_" + Barcode,ImagePath);
			MarshalString( ImgPath3 + ImgPath5 + "_" + Barcode,P_ImagePath);
			MarshalString( ImgPath4 + ImgPath5 + "_" + Barcode,F_ImagePath);
			imwrite (ImagePath + "_Barcode.bmp", binary);
			cropImage(image , &SaveImg , x ,y , w ,h);
		}
		delete barcodeapi;
	}
	std::string path ;

	cropImage(image ,&binary, x+230 , y+25 , w-400, h-50);
	//GaussianBlur( binary, binary, Size(3, 3), 2, 2 );
	threshold( binary, binary, 40, 255,1);
	if (debug) {
		namedWindow("Debugimagethreshold",0);
		imshow("Debugimagethreshold", binary);
		MarshalString( FileName + "_CenterBinary" + ".bmp",path);
		imwrite(path,binary);
	}
	Point2d contourCcenter = findContourMinArea(&binary);

	contourCcenter.x += x+230;
	contourCcenter.y += y+25;
	tmp2d.c.x+=x+230 ; 
	tmp2d.c.y+=y+25;
	x = contourCcenter.x - w / 2;
	y = contourCcenter.y - h / 2;
	cropImage(image ,&binary, x , y+100 ,w-600 ,h-200);
	threshold( binary, binary, 35, 255,0);
	if (debug) {
		namedWindow("Debugimagethreshold1",0);
		imshow("Debugimagethreshold1", binary);
		MarshalString( FileName + "_LeftBinary" + ".bmp",path);
		imwrite(path,binary);
	}
	GaussianBlur( binary, binary, Size(7, 7), 0, 0 );
	Vcircle tmp2o = findLargeCircle(&binary, minDist2 ,minRadius2 ,maxRadius2,param21,param22); // lo
	if (tmp2o.c.x == -1.0 && tmp2o.c.y ==-1.0) {
		if (SaveImage){
			imwrite(F_ImagePath + "_Left_fail.bmp",SaveImg); 
			LogCollection::LogCWriteLine(FileName +",Left_fail,,,,NA");
		}
		SaveImg.release();
		binary.release() ;  
		return LEFTERROR; 
	}
	Mat contourLimage ;
	cropImage(&binary ,&contourLimage, (int)(tmp2o.c.x-cvRound(tmp2o.r) - (maxRadius2-minRadius2)) , (int)(tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2)), (int)(maxRadius2 + maxRadius2-tmp2o.r)*2.3 ,(int)(maxRadius2 + maxRadius2-tmp2o.r)*2.3);
	threshold( contourLimage, contourLimage, 35, 255,0);
	Point2d contourLeftCenter = findContourMinArea(&contourLimage);
	contourLimage.release();

	if(debug) {
		Mat tmpimage ;
		cropImage(image ,&tmpimage, x , y+100 ,w-600 ,h-200);
		cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		circle(tmpimage, tmp2o.c, cvRound(tmp2o.r), Scalar(0, 255, 255), 1, CV_AA);
		circle(tmpimage, tmp2o.c, 1, Scalar(255, 255, 0), -1, CV_AA);
		circle(tmpimage, Point2d(contourLeftCenter.x+tmp2o.c.x-cvRound(tmp2o.r) -(maxRadius2-minRadius2) , contourLeftCenter.y +tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2)), 1, Scalar(0, 0, 255), -1, CV_AA);
		//imwrite("DebugimageLeft.bmp",tmpimage);
		namedWindow("DebugimageLeft",0);
		imshow("DebugimageLeft", tmpimage);
		tmpimage.release();
	}
	contourLeftCenter.x += x + tmp2o.c.x-cvRound(tmp2o.r) -(maxRadius2-minRadius2) ;
	contourLeftCenter.y += y+100  + tmp2o.c.y-cvRound(tmp2o.r) -(maxRadius2-minRadius2);
	tmp2o.c.x+=x ; 
	tmp2o.c.y+=y+100;
	cropImage(image ,&binary, x+600 , y+100 ,w-600 ,h-200);
	threshold( binary, binary, 40, 255,0 );
	if (debug) {
		namedWindow("Debugimagethreshold2",0);
		imshow("Debugimagethreshold2", binary);
		MarshalString( FileName + "_RightBinary" + ".bmp",path);
		imwrite(path,binary);

	}
	GaussianBlur( binary, binary, Size(7, 7), 0, 0 );
	Vcircle tmp2a = findLargeCircle(&binary, minDist3 ,minRadius3 ,maxRadius3,param31,param32); // ro
	if (tmp2a.c.x == -1.0 && tmp2a.c.y ==-1.0) {
		if (SaveImage){
			imwrite(F_ImagePath + "_Right_fail.bmp",SaveImg); 
			LogCollection::LogCWriteLine(FileName +",Right_fail,,,,NA");
		}
		SaveImg.release();
		binary.release() ;  
		return RIGHTERROR ;
	}

	Mat contourRimage ;
	cropImage(&binary ,&contourRimage, (int)tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) , (int)tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3), (int)(maxRadius3 + maxRadius3-tmp2a.r)*2.5 ,(int)(maxRadius3 + maxRadius3-tmp2a.r)*2.5);
	threshold( contourRimage, contourRimage, 40, 255,0);
	Point2d contourRightCenter = findContourMinArea(&contourRimage);
	contourRimage.release();
	binary.release();
	if(debug) {
		Mat tmpimage ;
		cropImage(image ,&tmpimage, x+600 , y+100 ,w-600 ,h-200);
		cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		circle(tmpimage, tmp2a.c, cvRound(tmp2a.r), Scalar(0, 255, 255), 1, CV_AA);
		circle(tmpimage, tmp2a.c, 1, Scalar(255, 255, 0), -1, CV_AA);
		circle(tmpimage, Point2d(contourRightCenter.x + tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) , contourRightCenter.y + tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3)), 1, Scalar(0, 0, 255), -1, CV_AA);
		//imwrite("DebugimageRight.bmp",tmpimage);
		namedWindow("DebugimageRight",0);
		imshow("DebugimageRight", tmpimage);
		tmpimage.release();
	}
	contourRightCenter.x += x + 600 + tmp2a.c.x-cvRound(tmp2a.r) -(maxRadius3-minRadius3) ;
	contourRightCenter.y += y+100  + tmp2a.c.y-cvRound(tmp2a.r) -(maxRadius3-minRadius3);
	tmp2a.c.x+=x+600 ; 
	tmp2a.c.y+=y+100;
	double k =0 , b=0, ax=0 ,ay=0 ,angle =0;
	if ((contourRightCenter.y - contourLeftCenter.y) == 0 ||  (contourRightCenter.x - contourLeftCenter.x) ==0) {
		k = 0;
		ay = b = ((contourLeftCenter.y + contourRightCenter.y) /2) - k * ((contourLeftCenter.x + contourRightCenter.x)/2);
		ax = ((contourLeftCenter.x + contourRightCenter.x) /2);
		angle = 0;
	}else{
		k =  -1 * (contourRightCenter.y - contourLeftCenter.y) / (contourRightCenter.x - contourLeftCenter.x);
		angle = System::Math::Atan(k) * 180 /  System::Math::PI ;
		if (k < 0 ) angle = 180 + angle;
		k = 1 /k ;
		b = ((contourLeftCenter.y + contourRightCenter.y) /2) - k * ((contourLeftCenter.x + contourRightCenter.x)/2);
		ax =  (contourCcenter.x + k * contourCcenter.y - k * b) / (1 + k *k ) ;
		ay = k * ax + b ;
	}

	cropImage(image ,&binary, x+300 , y+100 , w-600, h-150);
	float theta = findLines(&binary , 48);
	if (angle > 90 && theta > 90)
		theta = 180 - (theta - angle);
	else if ( angle > 90 && theta < 90 )
		theta = theta - (180 -angle);
	else if ( angle < 90 && theta > 90)
		theta = (180 - theta) -angle  ;
	else 
		theta = theta -angle ;
	theta = 180 -theta;
	double Rtheta = System::Convert::ToDouble(theta);
	if (Rtheta < 0 )
		Rtheta =  System::Math::Abs(theta);
	else if (theta > 180)
		Rtheta = Rtheta -180 ;
	else if (Rtheta > 90 && Rtheta < 180)
		Rtheta = 180 -Rtheta;
	bool pass = Rtheta > 1.5 ? false :true ; 

	cropImage(image ,&binary, 1195 , 1005 , 50,50);
	threshold( binary, binary, 40, 255,0);
	vector<Vcircle> resultCircle = detectCirclePoint(&binary , 50 ,50);

	vector<Point2i> RightPoint , LeftPoint; 
	vector<Point2i> LineRecordR , LineRecordL;
	int RightDis = 0 , LeftDis = 0;
	int RightBrackTan = 0 , RightFlexTan = 0 , LeftBrackTan = 0 , LeftFlexTan = 0;
	int DistanceAvgR = 0 ,DistanceAvgL = 0 ,TmpAvg = 0 ,Nums = 0;

	RightPoint = DistanceCheckR(image , 1250 , 970 ,50 ,80);
	
	for ( int i = 0 ; i < 8 ; i++){
		if ((RightPoint[2*i+1].x != -1) && (RightPoint[2*i].x != -1)){
			TmpAvg += ( RightPoint[2*i+1].x - RightPoint[2*i].x);
			Nums ++;
		}	
	}

	if (Nums != 0){
		TmpAvg = TmpAvg/Nums;
		Nums = 0;

		for ( int i = 0 ; i < 8 ; i++){
			if ((RightPoint[2*i+1].x != -1) && (RightPoint[2*i].x != -1)){
				if ((RightPoint[2*i+1].x - RightPoint[2*i].x) <= TmpAvg ){
					DistanceAvgR += (RightPoint[2*i+1].x - RightPoint[2*i].x);
					LineRecordR.push_back(RightPoint[2*i]);
					LineRecordR.push_back(RightPoint[2*i+1]);
					Nums ++;
				}	
			} 
		}
	}else
		LineRecordR.push_back(Point2i(-1,-1));

	if (Nums != 0){
		DistanceAvgR = DistanceAvgR/Nums;

		if (Nums > 1){
			RightBrackTan = (LineRecordR[1].x - LineRecordR[2*Nums-1].x) / (LineRecordR[1].y - LineRecordR[2*Nums-1].y);
			RightFlexTan = (LineRecordR[0].x - LineRecordR[2*Nums-2].x) / (LineRecordR[0].y - LineRecordR[2*Nums-2].y);
		}else{
			RightBrackTan = 0 ;
			RightFlexTan = 0 ;
		}
	}

	Nums = 0;
	TmpAvg = 0;
	LeftPoint = DistanceCheckL(image , 790  , 975 , 42 ,150);

	for ( int i = 0 ; i < 15 ; i++){
		if ((LeftPoint[2*i+1].x != -1) && (LeftPoint[2*i].x != -1)){
			TmpAvg += (LeftPoint[2*i].x - LeftPoint[2*i+1].x);
			Nums ++;
		}	
	}

	if (Nums != 0){
		TmpAvg = TmpAvg/Nums;
		Nums = 0;

		for ( int i = 0 ; i < 15 ; i++){
			if ((LeftPoint[2*i+1].x != -1) && (LeftPoint[2*i].x != -1)){
				if ((LeftPoint[2*i].x - LeftPoint[2*i+1].x) <= TmpAvg ){
					DistanceAvgL += (LeftPoint[2*i].x - LeftPoint[2*i+1].x);
					LineRecordL.push_back(LeftPoint[2*i]);
					LineRecordL.push_back(LeftPoint[2*i+1]);
					Nums ++;
				}	
			} 
		}
	}else
		LineRecordL.push_back(Point2i(-1,-1));

	if (Nums != 0){
		DistanceAvgL = DistanceAvgL/Nums;

		if (Nums > 1){
			LeftBrackTan = (LineRecordL[1].x - LineRecordL[2*Nums-1].x) / (LineRecordL[1].y - LineRecordL[2*Nums-1].y);
			LeftFlexTan = (LineRecordL[0].x - LineRecordL[2*Nums-2].x) / (LineRecordL[0].y - LineRecordL[2*Nums-2].y);
		}else{
			LeftBrackTan = 0 ;
			LeftFlexTan = 0 ;
		}
	}


	if (debug || SaveImage) {
		Mat tmpimage;
		image->copyTo(tmpimage);
		std::string pointC ;
		double fb =0.0 ;
		fb =  contourCcenter.y - System::Math::Tan(theta * System::Math::PI /180) *contourCcenter.x;
		cvtColor( tmpimage, tmpimage, CV_GRAY2BGR );
		circle(tmpimage, contourCcenter, 1, Scalar(200, 0, 30), -1, CV_AA);
		circle(tmpimage, contourCcenter, 86, Scalar(0, 255, 255), 1, CV_AA);
		MarshalString ( "contourCcenterOfFlex : ( " +contourCcenter.x + " , " + contourCcenter.y + " )" , pointC);
		putText(tmpimage , pointC ,Point2d(x,y+20) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(200, 0, 30) ,1 ,CV_AA );
		circle(tmpimage, contourLeftCenter, 1, Scalar(30, 0, 200), -1, CV_AA);
		circle(tmpimage, contourLeftCenter, cvRound(tmp2o.r), Scalar(0, 255, 255), 1, CV_AA);
		MarshalString ("contourLeftCcenterOfBracket : ( " +contourLeftCenter.x + " , " + contourLeftCenter.y + " )" , pointC);
		putText(tmpimage , pointC ,Point2d(x,y+40) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(100, 150, 200) ,1 ,CV_AA );
		circle(tmpimage, contourRightCenter, 1, Scalar(100, 150, 200), -1, CV_AA);
		circle(tmpimage, contourRightCenter, cvRound(tmp2a.r), Scalar(0, 255, 255), 1, CV_AA);
		MarshalString ( "contourRightCcenterOfBracket : ( " +contourRightCenter.x + " , " + contourRightCenter.y + " )" , pointC);
		putText(tmpimage , pointC ,Point2d(x,y+60) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(0, 30, 200) ,1 ,CV_AA );
		line(tmpimage, contourLeftCenter , contourRightCenter, Scalar(255, 0, 255), 1, CV_AA);
		//line(tmpimage, contourCcenter , Point2d(contourLeftCenter.x, contourLeftCenter.x * System::Math::Tan(theta * System::Math::PI /180)+fb), Scalar(255, 255, 0), 1, CV_AA);
		//line(tmpimage, contourCcenter , Point2d(contourRightCenter.x, contourRightCenter.x * System::Math::Tan(theta * System::Math::PI /180)+fb), Scalar(255, 255, 0), 1, CV_AA);
		circle(tmpimage, Point2d(((contourLeftCenter.x + contourRightCenter.x)/2),((contourLeftCenter.y + contourRightCenter.y) /2)), 1, Scalar(125, 125, 200), -1, CV_AA);
		MarshalString ( "centerOfTwoContoursLineOnBracket : ( " +((contourLeftCenter.x + contourRightCenter.x)/2) + " , " + ((contourLeftCenter.y + contourRightCenter.y) /2)+ " )" , pointC);
		putText(tmpimage , pointC ,Point2d(x,y+80) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(125, 125, 200) ,1 ,CV_AA );
		MarshalString ((System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110) ? "Fail" :"Pass",pointC);
		putText(tmpimage , pointC ,Point2d(x,y+100) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(150, 150, 0) ,1 ,CV_AA );	

		if (!resultCircle.empty()) {
			for (int i = 0 ; i < resultCircle.size() ;i++) {
				circle(tmpimage, Point2d (1195 +resultCircle[i].c.x/4 , 1005 +resultCircle[i].c.y/4 ), resultCircle[i].r /4, Scalar(200, i*255, 30), 1, CV_AA);
				circle(tmpimage, resultCircle[i].c, 1, Scalar(200, i*255, 30), -1, CV_AA);
				MarshalString ( "resultCircle : ( " +(1195 +resultCircle[i].c.x/4) + " , " + (1005 +resultCircle[i].c.y/4)+ " )" , pointC);
				putText(tmpimage , pointC ,Point2d(x,y+160 + i *20) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(200, i*255, 30) ,1 ,CV_AA );
			}
		}
	
		if (LineRecordR[0].x != -1){
			for (int round = 0 ; round < LineRecordR.size() ; round ++){
				LineRecordR[round].x += 1250;
				LineRecordR[round].y += 970;
			}
			//circle(tmpimage, RightPoint[0], 1, Scalar(0, 255, 0), -1, CV_AA);
			//circle(tmpimage, RightPoint[1], 1, Scalar(0, 0, 255), -1, CV_AA);
			line(tmpimage, Point2i(LineRecordR[0].x -80*RightFlexTan, LineRecordR[0].y - 50) , Point2i(LineRecordR[0].x + 80*RightFlexTan, LineRecordR[0].y + 50), Scalar(125, 255, 200), 1, CV_AA);
			line(tmpimage, Point2i(LineRecordR[1].x -80*RightBrackTan, LineRecordR[1].y - 50) , Point2i(LineRecordR[1].x + 80*RightBrackTan, LineRecordR[1].y + 50), Scalar(255, 125, 200), 1, CV_AA);
			RightDis = DistanceAvgR;
		}else
			RightDis = -1;

		MarshalString ("DisR = " + RightDis ,pointC);
		putText(tmpimage , pointC ,Point2d(x,y+120) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(255, 255, 100) ,1 ,CV_AA );

		if (LineRecordL[0].x != -1){
			for (int round = 0 ; round < 2 ; round ++){
				LineRecordL[round].x += 790;
				LineRecordL[round].y += 975;
			}
			//circle(tmpimage, LeftPoint[1], 1, Scalar(0, 255, 0), -1, CV_AA);
			//circle(tmpimage, LeftPoint[0], 1, Scalar(0, 0, 255), -1, CV_AA);
			line(tmpimage, Point2i(LineRecordL[0].x -80*LeftFlexTan, LineRecordL[0].y - 50) , Point2i(LineRecordL[0].x + 80*LeftFlexTan, LineRecordL[0].y + 50), Scalar(125, 255, 200), 1, CV_AA);
			line(tmpimage, Point2i(LineRecordL[1].x -80*LeftFlexTan, LineRecordL[1].y - 50) , Point2i(LineRecordL[1].x + 80*LeftFlexTan, LineRecordL[1].y + 50), Scalar(255, 125, 200), 1, CV_AA);
			LeftDis = DistanceAvgL;
		}else
			LeftDis = -1;

		MarshalString ("DisL = " + LeftDis ,pointC);
		putText(tmpimage , pointC ,Point2d(x,y+140) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(255, 255, 100) ,1 ,CV_AA );


		Mat debugimage ;
		if (debug){
			cropImage(&tmpimage ,&debugimage, x , y , w, h);
			namedWindow("detected offset point",0);
			imshow("detected offset point", debugimage);
		}
		if (SaveImage){

			cropImage(&tmpimage ,&debugimage, x , y , w, h);

			if (System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110)
				imwrite (F_ImagePath + ".bmp", debugimage);
			 else
				imwrite (P_ImagePath + ".bmp", debugimage);

		}
		if (debug){
			MarshalString (FileName + ".bmp", pointC);
			imwrite(pointC,debugimage);
		}
		tmpimage.release();
		debugimage.release();

		LogCollection::LogCWriteLine(FileName +
			"," +contourCcenter.x + "," + contourCcenter.y  + 
			"," +((contourLeftCenter.x + contourRightCenter.x)/2) + "," + ((contourLeftCenter.y + contourRightCenter.y) /2) +
//			"," +ax + "," + ay +
//			"," +contourLeftCenter.x + "," + contourLeftCenter.y +
//			"," +contourRightCenter.x + "," + contourRightCenter.y +
//			"," + Rtheta +
			"," + ((System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) > 110) ? "Fail" :"Pass"));
		if (!resultCircle.empty()) {
			for (int i = 0 ; i < resultCircle.size() ;i++) {
				LogCollection::LogCWriteLine( "" + (1195 +resultCircle[i].c.x/4) + "," + (1005 +resultCircle[i].c.y/4) );
			}
		}
	}

	binary.release();
	SaveImg.release();
	//bool result = pass & (System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) < 12.25);
	bool result = (System::Math::Pow((contourCcenter.x - ((contourLeftCenter.x + contourRightCenter.x)/2)) , 2) + System::Math::Pow((contourCcenter.y - ((contourLeftCenter.y + contourRightCenter.y)) /2) ,2) < 110);
	return result ? 1 : ERROR_MV_INSPECTION_FAIL ; 

}


bool featureAPI::isFlexEmpty(Mat *image ,int x ,int y , int w, int h , int vth , float rate){
	Mat binary;
	cropImage(image ,&binary, x+230 , y+25 , w-400, h-50);
	GaussianBlur( binary, binary, Size(3, 3), 2, 2 );
	threshold( binary, binary, vth, 255,1);
	if (debug){
		namedWindow("Test Window",0);
		imshow("Test Window", binary);
	}
	int TotalNumberOfPixels = binary.rows * binary.cols;
	int ZeroPixels = TotalNumberOfPixels - countNonZero(binary);
	binary.release();
	return (ZeroPixels > TotalNumberOfPixels * rate) ? true : false;
}

Point2d featureAPI::CameraTest(Mat *image ,int x ,int y , int w, int h , int vth , int mindp, int minR ,int MaxR ,int pa1 ,int pa2){
	Mat binary;
	cropImage(image ,&binary, x , y , w , h); //190 930 180 180
	GaussianBlur( binary, binary, Size(3, 3), 2, 2 );
	threshold( binary, binary, vth, 255,1);
	if (debug){
		namedWindow("Test Window",0);
		imshow("Test Window", binary);
	}

	Vcircle tmp2o = findLargeCircle(&binary, mindp ,minR ,MaxR,pa1,pa2);
	Point2d tmp;
	tmp.x = tmp2o.c.x + x;
	tmp.y = tmp2o.c.y + y;
	//int TotalNumberOfPixels = binary.rows * binary.cols;
	//int ZeroPixels = TotalNumberOfPixels - countNonZero(binary);
	binary.release();
	return tmp;
	//return (ZeroPixels > TotalNumberOfPixels * rate) ? true : false;
}
Vcircle pointTocircle(Point2i point1 , Point2i point2 , Point2i point3 , int w){
	float m_12,m_13;
	Point2f c_12 , c_13; 
	Point2f c_p;
	Vcircle result ;
	Point2i temp = point1;
	Point2i temp1 = point2;
	Point2i temp2 = point3;
	if (point2.y != point1.y) {
		m_12 = (float)-(point2.x-point1.x) /  (point2.y-point1.y);
		c_12 = Point2f((point1.x+point2.x)/2,((point1.y+point2.y))/2);
	}
	else {
		m_12 = (float) -(point2.x-point3.x) / (point2.y-point3.y);
		c_12 = Point2f((point3.x+point2.x)/2,((point3.y+point2.y))/2);
	}
	if (point3.y != point1.y) {
		m_13 = (float)- (point3.x-point1.x) / (point3.y-point1.y);
	    c_13 = Point2f((point1.x+point3.x)/2, ((point1.y+point3.y))/2);
	}
	else {
		m_13 = (float) -(point3.x-point2.x)/(point3.y-point2.y);
		c_13 = Point2f((point2.x+point3.x)/2,((point2.y+point3.y))/2);
	}
	//這兩個中垂線段分別通過L_12和L_13的中點 ((x1+x2)/2, (y1+y2)/2) 和 ((x1+x3)/2, (y1+y3)/2)
	//c_12 = Point2f((point1.x+point2.x)/2,(point1.y+point2.y)/2);
	//c_13 = Point2f((point1.x+point3.x)/2,(point1.y+point3.y)/2);

	//兩個中垂線的公式
	//y = m_12*(x - c_12.x) + c_12.y
	//y = m_13*(x - c_13.x) + c_13.y
	//解這兩條線的交點，即得圓心座標(cx, cy)。解二元一次方程式
	float cx , cy ,r;
	cx = (float) (c_13.y-c_12.y-m_13*c_13.x+m_12*c_12.x)/(m_12-m_13);
	cy = (float)m_12*(cx-c_12.x)+c_12.y ;
	result.c = Point2d(cx,cy);
	//而半徑就是圓心到三點的距離
	result.r = Math::Sqrt((point1.x-cx)*(point1.x-cx)+(point1.y-cy)*(point1.y-cy));
	return result;
}
vector<VoteVcircle> featureAPI::findCandidateCircle(vector<Point2i> CandidatePoint , int w){
	vector<VCcircle> VCandidateC ;
	for (int i = 0 ; i < CandidatePoint.size() ; i++) {
		for (int j = i + 1 ;  j < CandidatePoint.size() ; j++) {
			for (int k = j +1 ; k < CandidatePoint.size() ; k++) {
				if (CandidatePoint[i] == CandidatePoint[j] || CandidatePoint[j] == CandidatePoint[k] || CandidatePoint[k]==CandidatePoint[i]
				|| (CandidatePoint[i].y == CandidatePoint[j].y && CandidatePoint[i].y == CandidatePoint[k].y)
					|| (CandidatePoint[i].x == CandidatePoint[j].x && CandidatePoint[i].x == CandidatePoint[k].x))
					continue;
				VCcircle VCCandidateC;
				Vcircle  CandidateC = pointTocircle( Point2i(CandidatePoint[i]), Point2i(CandidatePoint[j]), Point2i(CandidatePoint[k]),w);
				if (CandidateC.c.x > w || CandidateC.c.x < 0 || CandidateC.c.y >w || CandidateC.c.y < 0)
					continue;
				VCCandidateC.Vc = CandidateC;
				VCCandidateC.point1 = CandidatePoint[i];
				VCCandidateC.point2 = CandidatePoint[j];
				VCCandidateC.point3 = CandidatePoint[k];
				VCandidateC.push_back(VCCandidateC);
			}
		}
	}
	vector<VoteVcircle> RVCandidateC ;
	if (VCandidateC.size() >  0) {
		std::sort(VCandidateC.begin(), VCandidateC.end(), compareCircleR);
		VoteVcircle tmpv;
		tmpv.Vcc = VCandidateC[0] ;
		tmpv.vote = 0;
		RVCandidateC.push_back(tmpv);
		for (int i = 1 ; i < VCandidateC.size() ; i++) {

			if( cvRound(RVCandidateC[RVCandidateC.size()-1].Vcc.Vc.c.x) == cvRound(VCandidateC[i].Vc.c.x) && 
				cvRound(RVCandidateC[RVCandidateC.size()-1].Vcc.Vc.c.y) == cvRound(VCandidateC[i].Vc.c.y) &&
				cvRound(RVCandidateC[RVCandidateC.size()-1].Vcc.Vc.r) == cvRound(VCandidateC[i].Vc.r)) {
					RVCandidateC[RVCandidateC.size()-1].vote++;
					if (cvRound(VCandidateC[i].Vc.r) > w /2)
						break;
					continue;
			}
			tmpv.Vcc = VCandidateC[i];	
			tmpv.vote = 0;
			RVCandidateC.push_back(tmpv);
		}
		std::sort(RVCandidateC.begin(), RVCandidateC.end(), compareCircleVote);
		VCandidateC.clear();
	}
	return RVCandidateC;
}
vector<Vcircle> featureAPI::detectCirclePoint (Mat *image , int w , int h){
	Mat drawImage;
	resize (*image , *image , Size(), 4 , 4 ,INTER_LINEAR );
	image->copyTo(drawImage);
	w*=4;
	float const_b [19];
	vector<Point2i> CandidateFlexPoint , CandidateBreaketPoint;
	for (int i  = 0 ; i < (180/10)+1 ; i++ ){
		if (i == 0 || i == 9 || i ==18) 
			const_b[i] = 0;
		else
			const_b[i] =  w -(w / 2 -1 ) -  (w / 2 -1)* Math::Tan(Math::PI / 180 * i * 10);
	}

	for (int i  = 0 ; i < (180/10)+1 ; i++ ){
		int cc = 255;
		switch(i){
		case 0:
		case 18:
				for (int x = Math::Round(w/2 -1,System::MidpointRounding::ToEven) ; x > 0  ; x-- ){
					if (image->at<uchar>(w / 2 -1 , x) == cc) {
						if (cc == 0 ) { 
							CandidateFlexPoint.push_back(Point2i( x , w / 2 -1));
							break;
						}else {
							cc = 0 ;
							CandidateBreaketPoint.push_back(Point2i(x ,w / 2 -1));
						}
					} 
				}
				cc = 255;
				for (int x =  Math::Round(w/2 -1 ,System::MidpointRounding::ToEven) ; x < w  ; x++ ){
					if (image->at<uchar>(w / 2-1 , x) == cc) {
						if (cc == 0 ) { 
							CandidateFlexPoint.push_back(Point2i(x , w / 2 -1 ));
							break;
						}else {
							cc = 0 ;
							CandidateBreaketPoint.push_back(Point2i(x , w / 2 -1));
						}
					} 
				}
			break;
		case 9:
			for (int y = Math::Round(w/2,System::MidpointRounding::ToEven) ; y > 0  ; y-- ){
				if (image->at<uchar>(y , w / 2 -1 ) == cc) {
					if (cc == 0  ) { 
						CandidateFlexPoint.push_back(Point2i(w / 2 -1 ,y));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(w / 2 -1 ,y));
					}
				} 
			}
			cc = 255; 
			for (int y =  Math::Round(w/2,System::MidpointRounding::ToEven) ; y < w  ; y++ ){
				if (image->at<uchar>(y , w / 2 -1 ) == cc) {
					if (cc == 0) { 
						CandidateFlexPoint.push_back(Point2i(w / 2 -1 ,y));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(w / 2 -1 ,y));
					}
				} 
			}
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 14:
		case 15:
		case 16:
		case 17:
			for (int x = Math::Round(w/2 -1,System::MidpointRounding::ToEven) ; x > 0  ; x-- ){
				float index;
				//index =Math::Abs((w / 2 -1 - x) * Math::Tan(Math::PI / 180 * i * 10) + const_b[i]);
				index = -x*Math::Tan(Math::PI / 180 * i * 10) - const_b[i] +w;
				if (Math::Round( index ,System::MidpointRounding::ToEven)  > w-1 || index < 0 ) break;  
				//index = index > w-1 ? w-1 : index ;
				//index = index < 0 ?  0: index ;
				if (image->at<uchar>(Math::Round( index ,System::MidpointRounding::ToEven) , x) == cc) {
					if (cc == 0 ) { 
						CandidateFlexPoint.push_back(Point2i(x,Math::Round( index ,System::MidpointRounding::ToEven)));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(x,Math::Round( index ,System::MidpointRounding::ToEven)));
					}
				} 
			}
			 cc = 255;
			for (int x =  Math::Round(w/2 -1,System::MidpointRounding::ToEven) ; x < w  ; x++ ){
				float index;
				//index =Math::Abs((w / 2 -1 - x) * Math::Tan(Math::PI / 180 * i * 10) + const_b[i]);
				index = -x*Math::Tan(Math::PI / 180 * i * 10) - const_b[i] +w;
				if (Math::Round( index ,System::MidpointRounding::ToEven)  > w-1 || index < 0 ) break; 
				//index = index > w-1 ? w-1 : index ;
				//index = index < 0 ?  0: index ;
				if (image->at<uchar>(Math::Round( index ,System::MidpointRounding::ToEven) , x) == cc) {
					if (cc == 0) { 
						CandidateFlexPoint.push_back(Point2i(x,Math::Round( index ,System::MidpointRounding::ToEven)));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(x,Math::Round( index ,System::MidpointRounding::ToEven)));
					}
				} 
			}
			break;
		case 5:
		case 6:
		case 7:
		case 8:
		case 10:
		case 11:
		case 12:
		case 13:
			for (int y = Math::Round(w/2 -1 ,System::MidpointRounding::ToEven) ; y > 0  ; y-- ){
				float index;
				//index = (w /2 -1  + y) - const_b[i] /  Math::Tan(Math::PI / 180 * i * 10);
				index = ( w - y - const_b[i]) /Math::Tan(Math::PI / 180 * i * 10);
				if (Math::Round( index ,System::MidpointRounding::ToEven)  > w-1 || index < 0 ) break; 
				//index = index > w-1 ? w-1 : index ;
				//index = index < 0 ?  0: index ;
				if (image->at<uchar>(y , Math::Round( index ,System::MidpointRounding::ToEven)) == cc) {
					if (cc == 0) { 
						CandidateFlexPoint.push_back(Point2i(Math::Round( index ,System::MidpointRounding::ToEven) ,y));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(Math::Round( index ,System::MidpointRounding::ToEven),y ));
					}
				} 
			}
			cc = 255;
			for (int y =  Math::Round(w/2,System::MidpointRounding::ToEven) ; y < w  ; y++ ){
				float index;
				//index = (-y - w /2 + 1) - const_b[i] /  Math::Tan(Math::PI / 180 * i * 10);
				index = ( w - y - const_b[i]) /Math::Tan(Math::PI / 180 * i * 10);
				if (Math::Round( index ,System::MidpointRounding::ToEven)  > w-1 || index < 0 ) break; 
				//index = index > w-1 ? w-1 : index ;
				//index = index < 0 ?  0: index ;
				if (image->at<uchar>(y , Math::Round( index ,System::MidpointRounding::ToEven)) == cc) {
					if (cc == 0 ) { 
						CandidateFlexPoint.push_back(Point2i(Math::Round( index ,System::MidpointRounding::ToEven) ,y ));
						break;
					}else {
						cc = 0 ;
						CandidateBreaketPoint.push_back(Point2i(Math::Round( index ,System::MidpointRounding::ToEven) ,y));
					}
				} 
			}
			break;
		}
	}
	cvtColor( drawImage, drawImage, CV_GRAY2BGR );

	if (!CandidateFlexPoint.empty()) {
		for( int i = 0 ; i < CandidateFlexPoint.size(); i++ ){
			circle(drawImage, CandidateFlexPoint[i], 1, Scalar(0, 255, 0), -1, CV_AA);
		}
	}
	if (!CandidateBreaketPoint.empty()){
		for( int i = 0 ; i < CandidateBreaketPoint.size(); i++ ){
			circle(drawImage, CandidateBreaketPoint[i], 1, Scalar(0, 0, 255), -1, CV_AA);
		}
	}
	vector<VoteVcircle> FlexCandidateCircle = findCandidateCircle(CandidateFlexPoint , w);
	vector<Vcircle> VcircleResult ;
	if (!FlexCandidateCircle.empty()) {
		for (int i = 0 , count = 0; i < FlexCandidateCircle.size() ; i++ , count++ ) {
			if (count > 2) break;
			circle(drawImage, FlexCandidateCircle[i].Vcc.Vc.c, cvRound(FlexCandidateCircle[i].Vcc.Vc.r), Scalar(255, 255, 0), 1, CV_AA);
			circle(drawImage, FlexCandidateCircle[i].Vcc.point1, 1, Scalar(255, 0, 0), -1, CV_AA);
			circle(drawImage, FlexCandidateCircle[i].Vcc.point2, 1, Scalar(255, 0, 0), -1, CV_AA);
			circle(drawImage, FlexCandidateCircle[i].Vcc.point3, 1, Scalar(255, 0, 0), -1, CV_AA);
		}
		VcircleResult.push_back(FlexCandidateCircle[0].Vcc.Vc);
	}

	vector<VoteVcircle> BreakCandidateCircle = findCandidateCircle(CandidateBreaketPoint , w);

	if (!BreakCandidateCircle.empty()) {
		for (int i = 0 , count = 0 ; i < BreakCandidateCircle.size() ; i++ , count++) {
			if (count > 2) break;
			circle(drawImage, BreakCandidateCircle[i].Vcc.Vc.c, cvRound(BreakCandidateCircle[i].Vcc.Vc.r), Scalar(255, 0, 255), 1, CV_AA);
			circle(drawImage, BreakCandidateCircle[i].Vcc.point1, 1, Scalar(0, 255, 255), -1, CV_AA);
			circle(drawImage, BreakCandidateCircle[i].Vcc.point2, 1, Scalar(0, 255, 255), -1, CV_AA);
			circle(drawImage, BreakCandidateCircle[i].Vcc.point3, 1, Scalar(0, 255, 255), -1, CV_AA);
		}
		VcircleResult.push_back(BreakCandidateCircle[0].Vcc.Vc);
	}

	namedWindow("CandidatePoint",0);
	imshow("CandidatePoint",drawImage);
	drawImage.release();
	image->release();
	return VcircleResult;
}
void featureAPI::FindBracketEdge(Mat *image , int x) {
	//Canny(*image, *image, x, x*3, 3);
	namedWindow("CandidatePoint",0);
	imshow("CandidatePoint",*image);
	image->release();
}

vector<Point2i> featureAPI::DistanceCheckR (Mat *image , int x , int y , int w ,int h ){

	Mat corpImage; // , drawImage;
	bool FlexGet = false;
	int const_b [8];

	vector<Point2i> MarkPointR;
	cropImage(image , &corpImage , x , y , w ,h);
//	corpImage.copyTo(drawImage);

	for ( int i = 0 ; i < 8 ; i++){

		const_b[i] = (h-5) - i*(h -10)/(8-1) ;
		FlexGet = false;

		for (int xj =  1 ; xj < w  ; xj++ ){
			int cc = 30;

			if (FlexGet == false){
				if (corpImage.at<uchar>( const_b[i] , xj) > 37) { //比30亮
					MarkPointR.push_back(Point2i(xj , const_b[i]));
					xj -= 1; 
					FlexGet = true;
				}else if (xj > 30){
					MarkPointR.push_back(Point2i(-1 ,-1));
					MarkPointR.push_back(Point2i(-1 ,-1));
					break;
				} 
			}else if (FlexGet == true){
				if (corpImage.at<uchar>( const_b[i] , xj) > 200){ // 比220亮
					MarkPointR.push_back(Point2i(xj , const_b[i]));
					break;
				}else if ( xj > (MarkPointR[2*i].x + 15) ){
					MarkPointR.push_back(Point2i(-1 ,-1));
					break;
				}

			}
		}

	}

	corpImage.release();

	return MarkPointR;

/*
	vector<Point2i> CandidateFlexPointR , CandidateBreaketPointR;
	cropImage(image , &corpImage , x , y , w ,h);
	corpImage.copyTo(drawImage);

	for ( int i = 0 ; i < 8 ; i++){

		const_b[i] = (h-5) - i*(h -10)/(8-1) ;
		FlexGet = false;

		for (int xj =  1 ; xj < w  ; xj++ ){
			int cc = 30;

			if (FlexGet == false){
				if (corpImage.at<uchar>( const_b[i] , xj) > 37) { //比30亮
					CandidateFlexPointR.push_back(Point2i(xj , const_b[i]));
					xj -= 1; 
					FlexGet = true;
				}else if (xj > 30){
					CandidateFlexPointR.push_back(Point2i(-1 ,-1));
					CandidateBreaketPointR.push_back(Point2i(-1 ,-1));
					//FlexGet = true;
					break;
				} 
			}else if (FlexGet == true){
				if (corpImage.at<uchar>( const_b[i] , xj) > 200){ // 比220亮
					CandidateBreaketPointR.push_back(Point2i(xj , const_b[i]));
					break;
				}else if ( xj > (CandidateFlexPointR[i].x + 15) ){
					CandidateBreaketPointR.push_back(Point2i(-1 ,-1));
					break;
				}

			}
		}

	}



	int DistanceAvg , TmpAvg , Nums = 0 ;

	cvtColor( drawImage, drawImage, CV_GRAY2BGR );

	for ( int i = 0 ; i < 8 ; i++){
		if ((CandidateBreaketPointR[i].x ) != -1 && (CandidateFlexPointR[i].x != -1)){
			TmpAvg += ( CandidateBreaketPointR[i].x - CandidateFlexPointR[i].x);
			//circle(drawImage, CandidateFlexPointR[i], 1, Scalar(0, 255, 0), -1, CV_AA);
			Nums ++;
		}	
	}

	if (Nums != 0){
		TmpAvg = TmpAvg/Nums;
		Nums = 0;
	}else
		return CandidateBreaketPointR;

	vector<Point2i> ReturnValue;
	for ( int i = 0 ; i < 8 ; i++){
		if ((CandidateBreaketPointR[i].x) != -1 && (CandidateBreaketPointR[i].x != -1)){
			if ((CandidateBreaketPointR[i].x - CandidateFlexPointR[i].x) <= TmpAvg ){
				DistanceAvg += (CandidateBreaketPointR[i].x- CandidateFlexPointR[i].x);
				if (Nums == 0){
					ReturnValue.push_back(CandidateFlexPointR[i]);
					ReturnValue.push_back(CandidateBreaketPointR[i]);
				}

				circle(drawImage, CandidateFlexPointR[i], 1, Scalar(0, 255, 0), -1, CV_AA);
				circle(drawImage, CandidateBreaketPointR[i], 1, Scalar(0, 0, 255), -1, CV_AA);
				Nums ++;
			}	
		}
	}

	if (Nums != 0)
		DistanceAvg = DistanceAvg/Nums;


	System::String ^ ImgPath  = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\";
	System::String ^ ImgPath2 = ImgPath + "Scara_Vision\\";

	if ( ! System::IO::Directory::Exists( ImgPath ) )
		System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath );

	if ( ! System::IO::Directory::Exists( ImgPath2 ) )
		System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath2 );

	std::string ImagePath;
	std::string DisplayD;
	MarshalString ( "D=" + DistanceAvg.ToString() , DisplayD);
	putText(drawImage , DisplayD ,Point2d(5,15) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(200, 0, 30) ,1 ,CV_AA );

	MarshalString( ImgPath2 +  System::DateTime::Now.ToString("MMddhhmmss") + "_Right.bmp" ,ImagePath);
	imwrite (ImagePath , drawImage);

	if (debug){
		namedWindow("CandidatePointR",0);
		imshow("CandidatePointR",drawImage);
	}

	drawImage.release();
	corpImage.release();

	return ReturnValue;
*/
}

vector<Point2i> featureAPI::DistanceCheckL (Mat *image , int x , int y , int w ,int h ){

	Mat corpImageL; //, drawImageL;
	bool FlexGet = false;
	int const_b [15];

	vector<Point2i> MarkPointL;
	cropImage(image , &corpImageL , x , y , w ,h);
//	corpImageL.copyTo(drawImageL);

	for ( int i = 0 ; i < 15 ; i++){

		const_b[i] = (h-5) - i*(h -10)/(15-1) ;
		FlexGet = false;

		for (int xj = (w-1) ; xj >= 1  ; xj-- ){
			int cc = 30;

			if (FlexGet == false){
				if (corpImageL.at<uchar>( const_b[i] , xj) > 37) { //比30亮
					MarkPointL.push_back(Point2i(xj , const_b[i]));
					xj += 1; 
					FlexGet = true;
				}else if (xj < 20){
					MarkPointL.push_back(Point2i(-1 ,-1));
					MarkPointL.push_back(Point2i(-1 ,-1));
					break;
				} 
			}else if (FlexGet == true){
				if (corpImageL.at<uchar>( const_b[i] , xj) > 200){ // 比220亮
					MarkPointL.push_back(Point2i(xj , const_b[i]));
					break;
				}else if ( xj < (MarkPointL[2*i].x - 15) ){
					MarkPointL.push_back(Point2i(-1 ,-1));
					break;
				}

			}
		}

	}

//	drawImageL.release();
	corpImageL.release();

	return MarkPointL;

	/*
	vector<Point2i> CandidateFlexPointL , CandidateBreaketPointL;
	cropImage(image , &corpImageL , x , y , w ,h);
	corpImageL.copyTo(drawImageL);


	for ( int i = 0 ; i < 15 ; i++){

		const_b[i] = (h-5) - i*(h -10)/(15-1) ;
		FlexGet = false;

		for (int xj = (w-1) ; xj > 1  ; xj-- ){
			int cc = 30;

			if (FlexGet == false){
				if (corpImageL.at<uchar>( const_b[i] , xj) > 37) { //比30亮
					CandidateFlexPointL.push_back(Point2i(xj , const_b[i]));
					xj += 1; 
					FlexGet = true;
				}else if (xj < 20){
					CandidateFlexPointL.push_back(Point2i(-1 ,-1));
					CandidateBreaketPointL.push_back(Point2i(-1 ,-1));
					//FlexGet = true;	
					break;
				} 
			}else if (FlexGet == true){
				if (corpImageL.at<uchar>( const_b[i] , xj) > 200){ // 比220亮
					CandidateBreaketPointL.push_back(Point2i(xj , const_b[i]));
					break;
				}else if ( xj < (CandidateFlexPointL[i].x - 15) ){
					CandidateBreaketPointL.push_back(Point2i(-1 ,-1));
					break;
				}

			}
		}

	}


	int DistanceAvg, Tmp, Nums = 0 ;
	vector<Point2i> ReturnValueL;

	cvtColor( drawImageL, drawImageL, CV_GRAY2BGR );

	for ( int i = 0 ; i < 15 ; i++){
		if ((CandidateBreaketPointL[i].x ) != -1 && (CandidateFlexPointL[i].x != -1)){
			Tmp += ( CandidateFlexPointL[i].x - CandidateBreaketPointL[i].x );
			//circle(drawImageL, CandidateFlexPointL[i], 1, Scalar(0, 255, 0), -1, CV_AA);
			Nums ++;
		}	
	}

	if (Nums != 0){
		Tmp = Tmp/Nums;
		Nums = 0;
	}else
		return CandidateBreaketPointL;


	for ( int i = 0 ; i < 15 ; i++){
		if ((CandidateBreaketPointL[i].x) != -1 && (CandidateBreaketPointL[i].x != -1)){
			if ((CandidateFlexPointL[i].x - CandidateBreaketPointL[i].x) <= Tmp ){
				DistanceAvg += (CandidateFlexPointL[i].x - CandidateBreaketPointL[i].x);
//				if (Nums == 0){
//					ReturnValueL.push_back(CandidateBreaketPointL[i]);
//					ReturnValueL.push_back(CandidateFlexPointL[i]);
//				}
//				circle(drawImageL, CandidateFlexPointL[i], 1, Scalar(0, 255, 0), -1, CV_AA);
//				circle(drawImageL, CandidateBreaketPointL[i], 1, Scalar(0, 0, 255), -1, CV_AA);
				Nums ++;
			}	
		}
	}

	if (Nums != 0){
		DistanceAvg = DistanceAvg/Nums;
		Nums = 0;
		Tmp = 0;
	}

	int AvgNearest = 999;

	for ( int i = 0 ; i < 15 ; i++){
		if ((CandidateBreaketPointL[i].x) != -1 && (CandidateBreaketPointL[i].x != -1)){
			Tmp = System::Math::Abs((CandidateFlexPointL[i].x - CandidateBreaketPointL[i].x) - DistanceAvg);
			if ((AvgNearest - Tmp) > 0){
				AvgNearest = Tmp;
				Nums = i;
			}
		}
		if (i == 14){
			ReturnValueL.push_back(CandidateBreaketPointL[Nums]);
			ReturnValueL.push_back(CandidateFlexPointL[Nums]);
		}
	}


	System::String ^ ImgPath  = "C:\\FG_UNLOAD\\" + System::DateTime::Today.ToString("yyyy_MM_dd")+"\\";
	System::String ^ ImgPath2 = ImgPath + "Scara_Vision\\";

	if ( ! System::IO::Directory::Exists( ImgPath ) )
		System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath );

	if ( ! System::IO::Directory::Exists( ImgPath2 ) )
		System::IO::DirectoryInfo^ di =  System::IO::Directory::CreateDirectory( ImgPath2 );

	std::string ImagePath;
	std::string DisplayD;
	MarshalString ( "D=" + DistanceAvg.ToString() , DisplayD);
	putText(drawImageL , DisplayD ,Point2d(5,15) ,FONT_HERSHEY_SIMPLEX ,0.5 ,Scalar(200, 0, 30) ,1 ,CV_AA );

	MarshalString( ImgPath2 +  System::DateTime::Now.ToString("MMddhhmmss") + "_Left.bmp" ,ImagePath);
	imwrite (ImagePath , drawImageL);
	if (debug){
		namedWindow("CandidatePointL",0);
		imshow("CandidatePointL",drawImageL);
	}


	drawImageL.release();
	corpImageL.release();
//	image->release();

	return ReturnValueL;
	*/
}