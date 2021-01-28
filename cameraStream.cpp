#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main() {
    cerr << "Program started" << endl;
    VideoCapture stream1(0);   //0 is the id of video device.0 if you have only one camera.
    cerr << "Stream Opened" << endl;

    if (!stream1.isOpened()) { //check if video device has been initialised
        cout << "cannot open camera";
    }

//unconditional loop
    while (true) {
        Mat cameraFrame;
        stream1.read(cameraFrame);
        cout << cameraFrame.size() << endl;
        
        if (waitKey(30) >= 0)
            break;
        Mat cameraGauche = cameraFrame(Range::all(), Range(0, 1087));
        Mat cameraDroite = cameraFrame(Range::all(), Range(1088, 2175));
        imshow("cam", cameraGauche);
        
        
        //imshow("cam", cameraDroite);
        
    }
    return 0;
}
