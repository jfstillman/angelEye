////AngelEye3
//
// TO DO:
// make videos loop
// calculate ground coordinates
// port to Unity

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

///*
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/flann.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/photo.hpp"
#include "opencv2/shape.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/superres.hpp"
#include "opencv2/video.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/videostab.hpp"
//*/


//------------ LIBRARY INCLUDES ABOVE --------------------//

using namespace cv;
using namespace std;

#include "Blob.h"

#define SHOW_STEPS            // un-comment or comment this line to show steps or not

// global variables ////////////////////////////////////////////////////////
const Scalar SCALAR_BLACK = Scalar(0.0, 0.0, 0.0);
const Scalar SCALAR_WHITE = Scalar(255.0, 255.0, 255.0);
const Scalar SCALAR_BLUE = Scalar(255.0, 0.0, 0.0);
const Scalar SCALAR_GREEN = Scalar(0.0, 200.0, 0.0);
const Scalar SCALAR_RED = Scalar(0.0, 0.0, 255.0);

// function prototypes  /////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(vector<Blob> &existingBlobs, vector<Blob> &currentFrameBlobs);
void addBlobToExistingBlobs(Blob &currentFrameBlob, vector<Blob> &existingBlobs, int &intIndex);
void addNewBlob(Blob &currentFrameBlob, vector<Blob> &existingBlobs);
double distanceBetweenPoints(Point point1, Point point2);
void drawAndShowContours(Size imageSize, vector<vector<cv::Point> > contours, string strImageName);
void drawAndShowContours(Size imageSize, vector<Blob> blobs, string strImageName);
void drawBlobInfoOnImage(vector<Blob> &blobs, Mat &imgFrame2Copy);

/////////////////////////////////////////////////////////////////////////////////////////////


//------------  original Tim Lupo main() START --------------------//
/*
int main(){
    
    //VideoCapture cap(0);//Webcam Cap
    VideoCapture capture(0);//Webcam Cap
    VideoCapture QTcapture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/D-lab3.mov");//QT Cap
       
        while (true){
    
            Mat Webcam;
            capture.read(Webcam);
            imshow("Webcam",Webcam);
 
            Mat QTcamMat;
            QTcapture.read(QTcamMat);
            imshow("QTcamMat- JS ",QTcamMat);
 
    waitKey(33); // added to Tim Lupo per tutorial 2 comments for webcam
       }
    
}
*/
//------------  original Tim Lupo main() END --------------------//


//------------  Lupo-based quicktime play main() START --------------------//
/*
int main(){
    
    //VideoCapture cap(0);//Webcam Cap
    //VideoCapture QTcapture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/D-lab3.mov");//Mass AveQT Cap
    VideoCapture QTcapture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/768x576_480P.mov");//Tut avi Cap
    while (true){
        
        Mat QTcamMat;
        QTcapture.read(QTcamMat);
        imshow("QTcamMat- JS ",QTcamMat);
        
        waitKey(66); // added to Tim Lupo per tutorial 2 comments for webcamvand QT- Determines speed of playback
          }
    
}
*/
//------------  Lupo-based quicktime play main() END --------------------//


//------------  temp fileReadTest() START --------------------//
/*
 VideoCapture capture;
 
 //VideoCapture capture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/D-lab3.mov");
capture.open("/shows/MIT/OpenCV/OpenCV_AngelEyeProj/OpenCV_AngelEyeProj/D-lab1.mov");
 
 capture.read(frame1);
 imshow("Frame1",frame1);
 */
//------------  temp fileReadTest() END --------------------//



//------------  Tutorial Main() START --------------------//


//int main(void) {
int main() {
 
 // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> from Lupo <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    /*
 //VideoCapture cap(0);//Webcam Cap
 //VideoCapture QTcapture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/D-lab3.mov");//Mass AveQT Cap
 VideoCapture QTcapture("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/768x576_480P.mov");//Tut avi Cap
 while (true){
 
 Mat QTcamMat;
 QTcapture.read(QTcamMat);
 imshow("QTcamMat",QTcamMat);
 
 waitKey(66); // added to Tim Lupo per tutorial 2 comments for webcamvand QT- Determines speed of playback
 */
 // >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> from Lupo <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
 
 
    //VideoCapture capVideo();
    //VideoCapture capVideo("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/768x576_480P.mov"); //Tut footage
   VideoCapture capVideo("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/E40_long1_640x426.mov"); // E40
    
    Mat imgFrame;
    //capVideo.read(imgFrame); //tut1
    Mat imgFrame1;//tut2
   	Mat imgFrame2;

vector<Blob> blobs;

    capVideo.open("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/E40_long1_640x426.mov");

    //capVideo.open("/shows/MIT/OpenCV/OpenCV_test/OpenCV_test/768x576_480P.mov");

    //capVideo.open(0);
    
//---------------cutout of bottom of Tut Main() START--------------------
    if (!capVideo.isOpened()) {                                                 // if unable to open video file
        cout << "\nerror reading video file" << endl << endl;      // show error message
        //    _getch();                    // it may be necessary to change or remove this line if not using Windows
        return(0);                                                              // and exit program
    }
    
    if (capVideo.get(CV_CAP_PROP_FRAME_COUNT) < 2) {
        cout << "\nerror: video file must have at least two frames\n";
        //      _getch();
        return(0);
    }
    
    //capVideo.read(imgFrame);
    capVideo.read(imgFrame1);
    capVideo.read(imgFrame2);

    //imshow("imgFrame", imgFrame); //JS add
    
    char chCheckForEscKey = 0;

    bool blnFirstFrame = true;

    int frameCount = 8; //JS changed from 2 to x

    while (capVideo.isOpened() && chCheckForEscKey != 27) {

        vector<Blob> currentFrameBlobs;

        Mat imgFrame1Copy = imgFrame1.clone();
        Mat imgFrame2Copy = imgFrame2.clone();

        Mat imgDifference;
        Mat imgThresh;
        
        //cvtColor	(InputArray src,OutputArray dst,int code, int dstCn = 0)
        cvtColor(imgFrame1Copy, imgFrame1Copy, CV_BGR2GRAY);
        cvtColor(imgFrame2Copy, imgFrame2Copy, CV_BGR2GRAY);

       // imshow("imgFrame1Copy", imgFrame1Copy); //JS off
        
        GaussianBlur(imgFrame1Copy, imgFrame1Copy, Size(5, 5), 0);
        GaussianBlur(imgFrame2Copy, imgFrame2Copy, Size(5, 5), 0);

        //imshow("GaussianBlur", imgFrame1Copy); //JS off
        
        //absdiff(Mat src1, Mat src2, Mat dst)
        // Computes the per-element absolute difference between two arrays or between an array and a scalar.
        absdiff(imgFrame1Copy, imgFrame2Copy, imgDifference);
//        imshow("imgDifference", imgDifference); //JS off
        
        
        threshold(imgDifference, imgThresh, 30, 255.0, CV_THRESH_BINARY);

//        imshow("imgThresh2", imgThresh); //JS off

        // specify fx and fy and let the function compute the destination image size.
        //resize(src, dst, dst.size(), 0, 0, interpolation);
        //resize(imgThresh, imgThresh, Size(), 2, 2, INTER_LINEAR);
         //imshow("resized", imgThresh); //JS off
        
        
        Mat structuringElement3x3 = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat structuringElement5x5 = getStructuringElement(MORPH_RECT, Size(5, 5));
        Mat structuringElement7x7 = getStructuringElement(MORPH_RECT, Size(7, 7));
        Mat structuringElement9x9 = getStructuringElement(MORPH_RECT, Size(9, 9));

        ///*
        dilate(imgThresh, imgThresh, structuringElement9x9);
        dilate(imgThresh, imgThresh, structuringElement9x9);
 //       imshow("dilate7x7", imgThresh); //JS off
        erode(imgThresh, imgThresh, structuringElement3x3);
//         imshow("erode3x3", imgThresh); //JS off
        //*/
/*
        dilate(imgThresh, imgThresh, structuringElement5x5);
        imshow("dilate5x5.1", imgThresh); //JS off
        dilate(imgThresh, imgThresh, structuringElement5x5);
        imshow("dilate5x5.2", imgThresh); //JS off
        erode(imgThresh, imgThresh, structuringElement5x5);
        imshow("erode5x5", imgThresh); //JS off
 */
        
        Mat imgThreshCopy = imgThresh.clone();

        vector<vector<Point> > contours;

        findContours(imgThreshCopy, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

         //drawAndShowContours(imgThresh.size(), contours, "imgContours"); // JS off

        vector<std::vector<Point> > convexHulls(contours.size());

        for (unsigned int i = 0; i < contours.size(); i++) {
            convexHull(contours[i], convexHulls[i]);
        }

        //drawAndShowContours(imgThresh.size(), convexHulls, "imgConvexHulls"); // JS off

        for (auto &convexHull : convexHulls) {
            Blob possibleBlob(convexHull);

            if (possibleBlob.currentBoundingRect.area() > 100 &&
                possibleBlob.dblCurrentAspectRatio >= 0.2 &&
                possibleBlob.dblCurrentAspectRatio <= 1.25 &&
                possibleBlob.currentBoundingRect.width > 20 &&
                possibleBlob.currentBoundingRect.height > 20 &&
                possibleBlob.dblCurrentDiagonalSize > 30.0 &&
                (cv::contourArea(possibleBlob.currentContour) / (double)possibleBlob.currentBoundingRect.area()) > 0.40) {
                currentFrameBlobs.push_back(possibleBlob);
            }
        }

        //drawAndShowContours(imgThresh.size(), currentFrameBlobs, "imgCurrentFrameBlobs");// JS off

        if (blnFirstFrame == true) {
            for (auto &currentFrameBlob : currentFrameBlobs) {
                blobs.push_back(currentFrameBlob);
            }
        }
        else {
            matchCurrentFrameBlobsToExistingBlobs(blobs, currentFrameBlobs);
        }

        //drawAndShowContours(imgThresh.size(), blobs, "imgBlobs"); // JS off

        imgFrame2Copy = imgFrame2.clone();          // get another copy of frame 2 since we changed the previous frame 2 copy in the processing above

        drawBlobInfoOnImage(blobs, imgFrame2Copy);

        imshow(">>>imgFrame2Copy<<<", imgFrame2Copy);

        //cv::waitKey(0);     // uncomment this line to go frame by frame for debugging  JS OFF

                // now we prepare for the next iteration

        currentFrameBlobs.clear();

        imgFrame1 = imgFrame2.clone();           // move frame 1 up to where frame 2 is

        if ((capVideo.get(CV_CAP_PROP_POS_FRAMES) + 1) < capVideo.get(CV_CAP_PROP_FRAME_COUNT)) {
            capVideo.read(imgFrame2);
        }
        else {
            std::cout << "end of video\n";
            break;
        }

        blnFirstFrame = false;
        frameCount++;
        chCheckForEscKey = cv::waitKey(1);
    }

    if (chCheckForEscKey != 27) {               // if the user did not press esc (i.e. we reached the end of the video)
        cv::waitKey(0);                         // hold the windows open to allow the "end of video" message to show
    }
    // note that if the user did press esc, we don't need to hold the windows open, we can simply let the program end which will close the windows

    return(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void matchCurrentFrameBlobsToExistingBlobs(std::vector<Blob> &existingBlobs, std::vector<Blob> &currentFrameBlobs) {

    for (auto &existingBlob : existingBlobs) {

        existingBlob.blnCurrentMatchFoundOrNewBlob = false;

        existingBlob.predictNextPosition();
    }

    for (auto &currentFrameBlob : currentFrameBlobs) {

        int intIndexOfLeastDistance = 0;
        double dblLeastDistance = 100000.0;

        for (unsigned int i = 0; i < existingBlobs.size(); i++) {
            if (existingBlobs[i].blnStillBeingTracked == true) {
                double dblDistance = distanceBetweenPoints(currentFrameBlob.centerPositions.back(), existingBlobs[i].predictedNextPosition);

                if (dblDistance < dblLeastDistance) {
                    dblLeastDistance = dblDistance;
                    intIndexOfLeastDistance = i;
                }
            }
        }

        if (dblLeastDistance < currentFrameBlob.dblCurrentDiagonalSize * 1.15) {
            addBlobToExistingBlobs(currentFrameBlob, existingBlobs, intIndexOfLeastDistance);
        }
        else {
            addNewBlob(currentFrameBlob, existingBlobs);
        }

    }

    for (auto &existingBlob : existingBlobs) {

        if (existingBlob.blnCurrentMatchFoundOrNewBlob == false) {
            existingBlob.intNumOfConsecutiveFramesWithoutAMatch++;
        }

        if (existingBlob.intNumOfConsecutiveFramesWithoutAMatch >= 5) {
            existingBlob.blnStillBeingTracked = false;
        }

    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addBlobToExistingBlobs(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs, int &intIndex) {

    existingBlobs[intIndex].currentContour = currentFrameBlob.currentContour;
    existingBlobs[intIndex].currentBoundingRect = currentFrameBlob.currentBoundingRect;

    existingBlobs[intIndex].centerPositions.push_back(currentFrameBlob.centerPositions.back());

    existingBlobs[intIndex].dblCurrentDiagonalSize = currentFrameBlob.dblCurrentDiagonalSize;
    existingBlobs[intIndex].dblCurrentAspectRatio = currentFrameBlob.dblCurrentAspectRatio;

    existingBlobs[intIndex].blnStillBeingTracked = true;
    existingBlobs[intIndex].blnCurrentMatchFoundOrNewBlob = true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void addNewBlob(Blob &currentFrameBlob, std::vector<Blob> &existingBlobs) {

    currentFrameBlob.blnCurrentMatchFoundOrNewBlob = true;

    existingBlobs.push_back(currentFrameBlob);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
double distanceBetweenPoints(cv::Point point1, cv::Point point2) {
    
    int intX = abs(point1.x - point2.x);
    int intY = abs(point1.y - point2.y);

    return(sqrt(pow(intX, 2) + pow(intY, 2)));
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndShowContours(cv::Size imageSize, std::vector<std::vector<cv::Point> > contours, std::string strImageName) {
    Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

    drawContours(image, contours, -1, SCALAR_WHITE, -1);

    imshow(strImageName, image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawAndShowContours(cv::Size imageSize, std::vector<Blob> blobs, std::string strImageName) {

    Mat image(imageSize, CV_8UC3, SCALAR_BLACK);

    std::vector<std::vector<cv::Point> > contours;

    for (auto &blob : blobs) {
        if (blob.blnStillBeingTracked == true) {
            contours.push_back(blob.currentContour);
        }
    }

    drawContours(image, contours, -1, SCALAR_WHITE, -1);

    imshow(strImageName, image);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
void drawBlobInfoOnImage(std::vector<Blob> &blobs, cv::Mat &imgFrame2Copy) {

    for (unsigned int i = 0; i < blobs.size(); i++) {

        if (blobs[i].blnStillBeingTracked == true) {
            cv::rectangle(imgFrame2Copy, blobs[i].currentBoundingRect, SCALAR_RED, 2);

            int intFontFace = CV_FONT_HERSHEY_SIMPLEX;
            double dblFontScale = blobs[i].dblCurrentDiagonalSize / 60.0;
            int intFontThickness = (int)std::round(dblFontScale * 1.0);

            cv::putText(imgFrame2Copy, std::to_string(i), blobs[i].centerPositions.back(), intFontFace, dblFontScale, SCALAR_GREEN, intFontThickness);
        }
    }
}
