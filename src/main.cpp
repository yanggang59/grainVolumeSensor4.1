
/*
*this is a program created by YangGang,ME,SJTU,CN
*Feel free to  use any code from here , let me know 
*using this email:1449381582@qq.com if you have any questions
*good luck!
*/


/*OpenCV和计算截面积相关的头文件*/
#include <opencv2/opencv.hpp>
#include "preprocessing.h" 
#include "cal_3d_coor.h"
#include "cal_area.h"
#include "serialSend.h"


/*和时间相关的头文件，用来计算时间*/
#include <sys/time.h>

#define USE_PRE_BG_IMAGE 1
//#define TAKE_PIC_WHEN_BOOT 1

using namespace cv;
using namespace std;

float a, b, c, A, B, C; //abc是ax+by+cz+1=0的形式的参数，ABC是Z=Ax+By+C形式的参数,需要转换一下

/*内参数矩阵，之前已经标定好*/
Mat K = (Mat_<float>(3, 3) << 518.8906422065566, 0, 294.5896218285499,
			      0, 520.0230989700873, 226.11902054293,
			      0, 0, 1);
								
VideoCapture capLeft(0);    //左边相机
VideoCapture capRight(1);   //右边相机

Mat leftFrame,rightFrame;    //左右相机的照片


/*用来存放开始和结束时间的结构体*/
struct timeval startTime,endTime;  


/*用来存放使用时间的变量,毫秒计*/
int Timeuse;  


/*时间累积*/
int timeAcc=0;


/*体积累加*/
float volumeAcc=0;



/*定义帧类型的枚举*/
typedef enum
{
  grain_frame,
  board_frame,
  void_frame
}Frame_Type;


int main()
{


	a = 0;
	b = -0.0153846;
	c = -0.0068497;

	A = -a / c;
	B = -b / c;
	C = -1 / c;

	Vec3f coeff(A, B, C);

	sleep(10);
	
      #ifdef USE_PRE_BG_IMAGE
	Mat leftMask = imread("../../res/left_mask.jpg", 0);
	Mat rightMask = imread("../../res/right_mask.jpg", 0);
      #endif
	
	
	
	
	int width = 640;
	int height = 480;
	Size ImageSize(width, height);
	
	
	/*判断当前帧是什么帧的变量*/
	Frame_Type frame_type;
	
	
	float flow_rate=0;
	
	/*串口相关的设置*/
	int fd;                            //文件描述符  
	int err;                           //返回调用函数的状态  
	int len;                          
	unsigned char send_buf[8]={0};  
	struct termios options; 
	

	/* 打开串口，返回文件描述符，在此之前必须先给串口写权限，否则会报错  */	
	fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY); 
	
	  /*若选择的是在开机的时候自动拍照作为背景照片*/
	#ifdef TAKE_PIC_WHEN_BOOT	  
	  int count4bg=1;
	  Mat leftMask,rightMask;
	#endif

	
	while(1)
	{
		/*若打开串口失败，则退出*/
		if  ( tcgetattr( fd,&options)  !=  0)  
		   {  
			  perror("SetupSerial 1");      
			  return(FALSE);   
		   } 
		   
		   
		   
		/*若左相机或者右边相机未打开，则退出*/
		if(!capLeft.isOpened() | !capRight.isOpened())
		{
			//cout<<"open camera  failure"<<endl;
			return (FALSE);
		}
		
		
		
		/*记录开始时间*/
		gettimeofday(&startTime,NULL);
		
	      #ifdef TAKE_PIC_WHEN_BOOT
		if(count4bg==1)
		{
		capLeft>>leftMask;
		capRight>>rightMask;
		count4bg++;
		contunue;
		}
	      #endif
		
		capLeft>>leftFrame;
		capRight>>rightFrame;
		    
		Mat left_skeleton(Size(width, height), CV_8UC1, Scalar(0));
		Mat right_skeleton(Size(width, height), CV_8UC1, Scalar(0));
		
		
		int countLeft = 0;
		int countRight = 0;
		
		//图像预处理函数,Input表示输入图像，uMask掩模图像，skeleton是输出的骨架，count计数骨架点的个数
		preprocessing(leftFrame, leftMask, left_skeleton, ImageSize,countLeft);
		preprocessing(rightFrame, rightMask, right_skeleton, ImageSize,countRight);
		
		//imshow("left_skeleton", left_skeleton);
		//cout << "countLeft is " << countLeft << endl;
		

		//如果点的个数少于50，则认为是空白帧,应该输出截面积为0，continue，进行下一次循环

		if (countLeft<50 |countRight<50)
		{
			cout << "this frame is void \n " << endl;
			frame_type=void_frame;
		}
		
		
		    
		

		/*计算直线上所有点相对于相机的坐标*/
		vector<Vec3f> allPointsLeft;
		vector<Vec3f> allPointsRight;
		
		int z_count_left = 0;  //记录大于z_threshold的点的个数
		int z_count_right = 0;  //记录大于z_threshold的点的个数
		
		

		int z_threshold = 20;
		
																																
		cal_3d_coor(left_skeleton, allPointsLeft, K, coeff, z_count_left, z_threshold);
		cal_3d_coor(right_skeleton, allPointsRight, K, coeff, z_count_right, z_threshold);
		
		
		cout << "number of left points that are larger than z_threshold is " << z_count_left << endl;
		cout << "the count of all left points is ：" << allPointsLeft.size() << endl;
		
		cout << "number of right points that are larger than z_threshold is " << z_count_left << endl;
		cout << "the count of all right points is ：" << allPointsLeft.size() << endl;	

		if (z_count_left<50 | z_count_right<50)
		{
			cout << "this frame is a board frame" << endl;
			frame_type=board_frame;
		}
		
		else frame_type=grain_frame;


		
		/*如果当前帧是谷物帧*/
		if(frame_type==grain_frame)
		{
		
		    
		    //计算面积,先分别计算左右面积，然后相加,这里的单位是平方毫米
		    int distance=40, board_width = 190;
		    float area_leftSide = cal_area(allPointsLeft, distance, board_width);
		    float area_rightSide = cal_area(allPointsRight, distance, board_width);
		    
		    float area=area_leftSide+area_rightSide;
		    cout << "area is " << area << endl;
		    
		    
		    
		    /*计算流量,速度是330mm/s,因此计算出来的结果是立方毫米/s，除以1000,单位是立方分米/s，也就是L/s,发送时要转化为int类型*/
		    float velocity=330.0;
		    float volume=velocity*area;
		    
		    int n_volume=(int)(volume/1000);
		    
		    
		    gettimeofday(&endTime,NULL);  
		    
		    /*时间，以毫秒为单位*/
	  
		    Timeuse = (1000000*(endTime.tv_sec - startTime.tv_sec) + (endTime.tv_usec - startTime.tv_usec))/1000; 
		    
		    timeAcc+=Timeuse;
		    
		    volumeAcc+=n_volume*Timeuse;
		    
		    continue;
		
		}
		
		else 
		{
		  
			if(timeAcc==0)
		      {
			
			volumeAcc=0;
			continue;
		      }
		      
		      else
		      {
			flow_rate=volumeAcc/timeAcc;
			
			volumeAcc=0;
			timeAcc=0;
			
			
		      }
		
		}
		send_buf[2]=((int)flow_rate)%256;
		send_buf[3]=((int)flow_rate)/256;
		
		
		
		
		serial_send(fd,&options,send_buf);
		  				    
		
	    }
	    
	 close(fd); 
}
