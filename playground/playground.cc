#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>

using namespace std;

int main(int argc, const char** argv){
    IplImage *img = cvLoadImage("chessboard.jpg");
    cvNamedWindow("testwindow");
    cvShowImage("testwindow", img);
    cout << "Press any key to exit" << endl;
    cvWaitKey(0);
    cvReleaseImage(&img);
    
}
