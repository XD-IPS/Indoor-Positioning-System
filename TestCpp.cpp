# include<iostream>
# include<opencv2/core/core.hpp>
# include<opencv2/highgui/highgui.hpp>
# include<opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

int main()
{
	Mat img = imread("C:\\Users\\Lobster\\Desktop\\Test.png");

	resize(img, img, Size(500, 500));

	imshow("image", img);

	waitKey();
	destroyAllWindows();
	return 0;
}
