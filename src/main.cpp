
/*
*this is a program created by YangGang,ME,SJTU,CN
*Feel free to  use any code from here , let me know 
*using this email:1449381582@qq.com if you have any questions
*good luck!
*/


/*OpenCV�ͼ���������ص�ͷ�ļ�*/
#include <opencv2/opencv.hpp>
#include "preprocessing.h" 
#include "cal_3d_coor.h"
#include "cal_area.h"
#include "serialSend.h"


/*��ʱ����ص�ͷ�ļ�����������ʱ��*/
#include <sys/time.h>

#define USE_PRE_BG_IMAGE 1
//#define TAKE_PIC_WHEN_BOOT 1

using namespace cv;
using namespace std;

float a, b, c, A, B, C; //abc��ax+by+cz+1=0����ʽ�Ĳ�����ABC��Z=Ax+By+C��ʽ�Ĳ���,��Ҫת��һ��

/*�ڲ�������֮ǰ�Ѿ��궨��*/
Mat K = (Mat_<float>(3, 3) << 518.8906422065566, 0, 294.5896218285499,
			      0, 520.0230989700873, 226.11902054293,
			      0, 0, 1);
								
VideoCapture capLeft(0);    //������
VideoCapture capRight(1);   //�ұ����

Mat leftFrame,rightFrame;    //�����������Ƭ


/*������ſ�ʼ�ͽ���ʱ��Ľṹ��*/
struct timeval startTime,endTime;  


/*�������ʹ��ʱ��ı���,�����*/
int Timeuse;  


/*ʱ���ۻ�*/
int timeAcc=0;


/*����ۼ�*/
float volumeAcc=0;



/*����֡���͵�ö��*/
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
	
	
	/*�жϵ�ǰ֡��ʲô֡�ı���*/
	Frame_Type frame_type;
	
	
	float flow_rate=0;
	
	/*������ص�����*/
	int fd;                            //�ļ�������  
	int err;                           //���ص��ú�����״̬  
	int len;                          
	unsigned char send_buf[8]={0};  
	struct termios options; 
	

	/* �򿪴��ڣ������ļ����������ڴ�֮ǰ�����ȸ�����дȨ�ޣ�����ᱨ��  */	
	fd = open( "/dev/ttyUSB0", O_RDWR|O_NOCTTY|O_NDELAY); 
	
	  /*��ѡ������ڿ�����ʱ���Զ�������Ϊ������Ƭ*/
	#ifdef TAKE_PIC_WHEN_BOOT	  
	  int count4bg=1;
	  Mat leftMask,rightMask;
	#endif

	
	while(1)
	{
		/*���򿪴���ʧ�ܣ����˳�*/
		if  ( tcgetattr( fd,&options)  !=  0)  
		   {  
			  perror("SetupSerial 1");      
			  return(FALSE);   
		   } 
		   
		   
		   
		/*������������ұ����δ�򿪣����˳�*/
		if(!capLeft.isOpened() | !capRight.isOpened())
		{
			//cout<<"open camera  failure"<<endl;
			return (FALSE);
		}
		
		
		
		/*��¼��ʼʱ��*/
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
		
		//ͼ��Ԥ������,Input��ʾ����ͼ��uMask��ģͼ��skeleton������ĹǼܣ�count�����Ǽܵ�ĸ���
		preprocessing(leftFrame, leftMask, left_skeleton, ImageSize,countLeft);
		preprocessing(rightFrame, rightMask, right_skeleton, ImageSize,countRight);
		
		//imshow("left_skeleton", left_skeleton);
		//cout << "countLeft is " << countLeft << endl;
		

		//�����ĸ�������50������Ϊ�ǿհ�֡,Ӧ����������Ϊ0��continue��������һ��ѭ��

		if (countLeft<50 |countRight<50)
		{
			cout << "this frame is void \n " << endl;
			frame_type=void_frame;
		}
		
		
		    
		

		/*����ֱ�������е���������������*/
		vector<Vec3f> allPointsLeft;
		vector<Vec3f> allPointsRight;
		
		int z_count_left = 0;  //��¼����z_threshold�ĵ�ĸ���
		int z_count_right = 0;  //��¼����z_threshold�ĵ�ĸ���
		
		

		int z_threshold = 20;
		
																																
		cal_3d_coor(left_skeleton, allPointsLeft, K, coeff, z_count_left, z_threshold);
		cal_3d_coor(right_skeleton, allPointsRight, K, coeff, z_count_right, z_threshold);
		
		
		cout << "number of left points that are larger than z_threshold is " << z_count_left << endl;
		cout << "the count of all left points is ��" << allPointsLeft.size() << endl;
		
		cout << "number of right points that are larger than z_threshold is " << z_count_left << endl;
		cout << "the count of all right points is ��" << allPointsLeft.size() << endl;	

		if (z_count_left<50 | z_count_right<50)
		{
			cout << "this frame is a board frame" << endl;
			frame_type=board_frame;
		}
		
		else frame_type=grain_frame;


		
		/*�����ǰ֡�ǹ���֡*/
		if(frame_type==grain_frame)
		{
		
		    
		    //�������,�ȷֱ�������������Ȼ�����,����ĵ�λ��ƽ������
		    int distance=40, board_width = 190;
		    float area_leftSide = cal_area(allPointsLeft, distance, board_width);
		    float area_rightSide = cal_area(allPointsRight, distance, board_width);
		    
		    float area=area_leftSide+area_rightSide;
		    cout << "area is " << area << endl;
		    
		    
		    
		    /*��������,�ٶ���330mm/s,��˼�������Ľ������������/s������1000,��λ����������/s��Ҳ����L/s,����ʱҪת��Ϊint����*/
		    float velocity=330.0;
		    float volume=velocity*area;
		    
		    int n_volume=(int)(volume/1000);
		    
		    
		    gettimeofday(&endTime,NULL);  
		    
		    /*ʱ�䣬�Ժ���Ϊ��λ*/
	  
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
