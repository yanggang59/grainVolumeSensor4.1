
#include "cal_area.h"
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

/*������������*/
/*diatance��������� ���ľ���,������㵽�ΰ��ʵ�ʾ��룬width�ǹΰ�Ŀ��,�����޳�����Ҫ�ĵ�*/
float cal_area(vector<Vec3f> pointsAfterTransform, float distance, float width)
{

	/*�ҵ������ϵĵ�*/
	vector<Vec3f> pointsOnGrain;
	for (int i = 0; i<pointsAfterTransform.size(); i++)
	{
		if (pointsAfterTransform[i][2] - distance<width / 2 && pointsAfterTransform[i][2] - distance>0)
			pointsOnGrain.push_back(pointsAfterTransform[i]);

	}

	//���û�е㣬�����Ϊ0
	if (0 == pointsOnGrain.size())
		return 0;



	/*�ҵ�x��������ֵ����Сֵ���Լ���ֵ�������*/
	float max_x_coor = pointsOnGrain[1][0];
	float min_x_coor = pointsOnGrain[1][0];
	int x_max_index = 0, x_min_index = 0;

	for (int i = 0; i<pointsOnGrain.size(); i++)
	{
		if (max_x_coor<pointsOnGrain[i][0])
		{
			max_x_coor = pointsOnGrain[i][0];
			x_max_index = i;
		}
		if (min_x_coor>pointsOnGrain[i][0])
		{
			min_x_coor = pointsOnGrain[i][0];
			x_min_index = i;
		}
	}


	/*�����֮���ˮƽ���*/
	int count = pointsOnGrain.size();
	float gap = (max_x_coor - min_x_coor) / (count - 1);

	/*���*/
	float area;

	/*z����֮��*/
	float z_sum = 0;
	for (int i = 0; i<count; i++)
	{
		z_sum += pointsOnGrain[i][2] - distance;
	}
	area = (max_x_coor - min_x_coor)*(pointsOnGrain[x_max_index][2] + pointsOnGrain[x_min_index][2]) / 2 - z_sum*gap;

	return area;
}