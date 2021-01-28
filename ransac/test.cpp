#include <opencv2/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
vector<float> vec{0.1,0.9,0.2,0.8,0.3,0.7,0.4,0.6,0.5,1};

Mat m1( vec );
imshow("m1",m1);
waitKey();

Mat m2( 1,vec.size(), CV_32FC1,vec.data());
imshow("m2",m2);
waitKey();

Mat1f m3( vec.size(), 1, vec.data());
imshow("m3",m3);
waitKey();

Mat1f m4( 1, vec.size(), vec.data());
imshow("m4",m4);
waitKey();

cout << "as seen below all Mat and vector use same data" << endl;
cout << vec[0] << endl;
m1 *= 2;
cout << vec[0] << endl;
m2 *= 2;
cout << vec[0] << endl;
m3 *= 2;
cout << vec[0] << endl;
m4 *= 2;
cout << vec[0] << endl;

return 0;
}
