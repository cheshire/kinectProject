#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, const char** argv){
    Point3f blah(1.234324, 2.3432423, 4.23432434);
    Point3i iblah = blah;
    cout << iblah << endl;
//    CvPoint3D32f b = cvPoint3D32f(1.12323213, 2.2342342, 3.345345);
//    cout << Point3i(b) << endl;
/*    Point3f blah(1.f, 2.f, 3.f);
    vector<Point3f> vBlah;
    vBlah.push_back(blah);
    vector<Point2f> out;
    convertPointsHomogeneous(Mat(vBlah), out);
    cout << out.at(0) << endl;*/
/*    float a =12;
    float b = 65;
    double m[2][2] = {{1.f, 2.f}, {3.f, 4.f}};
    Mat M = Mat(2, 2, CV_64F, m);
    Point2d blah(a, b);
    Mat MM = M * Mat(blah);*/

/*    float m[2][2] = {{1.f, 2.f}, {3.f, 4.f}};
    Mat M = Mat(2, 2, CV_32F, m);
    Point2f blah(5.f, 6.f);
    Mat MM = M * Mat(blah);
    Point2f b = MM.at<Point2f>(0, 0);
    cout << b.x << endl;
    vector<Point2f> vblah;
    Point2f blah(1, 2);
    Point2f qlah(3, 4);
    vblah.push_back(blah);
    vblah.push_back(qlah);
    cout << blah.x << endl;
    cout << vblah << endl;*/
/*    float m[2][9] = {{-1, 0, 1, -1, 0, 1, -1, 0, 1},
                    {1, 1, 1, 0, 0, 0, -1, -1, -1}};                    
    Mat M = Mat(2, 9, CV_32F, m);
    Mat Z;
    transpose(M, Z);

    cout << M << endl;
    cout << Z << endl;*/
    return 0; 
}
