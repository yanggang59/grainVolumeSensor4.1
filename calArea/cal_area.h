#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

/*������������*/
/*diatance��������� ���ľ���,�����㵽�ΰ��ʵ�ʾ��룬width�ǹΰ�Ŀ��,�����޳�����Ҫ�ĵ�*/
float cal_area(vector<Vec3f> pointsAfterTransform, float distance, float width);