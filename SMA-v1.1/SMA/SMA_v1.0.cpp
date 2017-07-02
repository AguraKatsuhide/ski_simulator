// SMA.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

#include "stdafx.h"
#include <Kinect.h>
#include <math.h>
#include <Windows.h>
#include <mmsystem.h>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease) //kinect2 �޸𸮸� �����ϰ� �����ϴ� �Լ�.
{
	if (pInterfaceToRelease != NULL)
	{

		pInterfaceToRelease->Release(); // kinect2 �� interface�� release.
		pInterfaceToRelease = NULL;
	}
}

IKinectSensor*          g_pKinectSensor=0; //kinect2 ����.
IDepthFrameReader*      g_pDepthFrameReader;//���� �� �д� ���� ����.
IInfraredFrameReader*	g_pInfraredFrameReader; // ���ܼ� ���� �д� ���� ����.
IColorFrameReader*		g_pColorFrameReader;// color ���� ���� ���� ����

ICoordinateMapper*		g_pCoordinateMapper=0; // mapping �ϴ� �Լ�
ColorSpacePoint*		g_pColorCoordinates=0; // ���� ��ǥ.



#define GET2D8U(IMAGE,X,Y) (*( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) )) //unsigned 8���� �̹��� �� ���� ��ũ��.
#define GET2D8U3CH(IMAGE,X,Y) ( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + ( 3 * (X) ) )//unsigned 8���� 3ä�� �̹��� �� ���� ��ũ��.
#define GET2D8U4CH(IMAGE,X,Y) ( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + ( 4 * (X) ) )//unsigned 8���� 4ä�� �̹��� �� ���� ��ũ��.
//use :  GET2D8U3CH(IMAGE,X,Y)[0], GET2D8U3CH(IMAGE,X,Y)[1], GET2D8U3CH(IMAGE,X,Y)[2]
#define GET2D16U(IMAGE,X,Y) (*( ( (ushort*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//unsigned 16���� �̹��� �� ���� ��ũ��.
#define GET2D16S(IMAGE,X,Y) (*( ( (short*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//signed 16���� �̹��� �� ���� ��ũ��.
#define GET2D32F(IMAGE,X,Y) (*( ( (float*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//float 32���� �̹��� �� ���� ��ũ��.

#define DEPTH_WIDTH 512// ���� �̹��� ���� 512.
#define DEPTH_HEIGHT 424//���� �̹��� ���� 424.

#define POINT_IN_RECT(P,R) ( ((P).x)>=((R).x) && ((P).x)<=(((R).x)+((R).width)) && ((P).y)>=((R).y) && ((P).y)<=(((R).y)+((R).height)) )
//����Ʈ�� �簢���ȿ� �ִ��� Ȯ��.


void initProgram();//���α׷� �ʱ�ȭ.
void readSettingsFromINI(); // ini ���� ����.

void initCamera();// ī�޶� �ʱ�ȭ.
void getDepthFrame(); // ���� ������ �޴� �Լ�.
void getDepthRanged();//���� ���� �����ϴ� �Լ�.
void getBinary(); // ����ȭ ������ �޴� �Լ�.
void processFrame();// �ݺ� ���� ��ų ���μ��� �Լ�.
void exitProgram();// ���α׷� ����.
void exitCamera();// ī�޶� ����.
void processKeyEvent(int key); // keyboard �Է��� �����ϴ� �Լ�.
void centerOfMass3d (IplImage* image);// 3���� �߽��� ���ϴ� �Լ�.
void centerOfMass2d(IplImage* image); //2c���� �߽� ���ϴ� �Լ�.
int distpoints(int a, int b, int c, int d);


void InitKinectV2(); // kinect2 ī�޶� �ʱ�ȭ �Լ�.
void exitKinectV2(); // kinect2 ī�޶� ���� �Լ�.


//CV_MAT_ELEM_PTR

int g_uFrameRate=30; //frame rate�� 30���� ����.

CvMemStorage* g_memStorage; // �ӽ� ���� ���� ����.
int g_uRangeMin; // ���̰� �ּ� ���� ����.
int g_uRangeMax; // ���̰� �ִ� ���� ����.

int g_uMaxDepth=4500; //���̰� �ִ� ���� ���� ����.
int g_uMinDepth=500;  //���̰� �ּ� ���� ���� ����.

int g_uThresholdMin=0; // ����ȭ ���� �Ӱ��� �ּҰ� ����.
int g_uThresholdMax=65535;// ����ȭ ���� �Ӱ��� �ִ밪 ����.

int g_uMinInfra=405; // ���ܼ� ���� �ּҰ� ����.

int g_3dX=0;// 3���� �߰��� x ��ǥ.
int g_3dY=0;// 3���� �߰��� y��ǥ.
int g_3dZ=0;// 3���� �߰��� z��ǥ.

int g_2dX=0;// 2���� �߰��� x ��ǥ.
int g_2dY=0;// 2���� �߰��� y��ǥ.

int g_HX=0; //�Ӹ��� x ��ǥ
int g_HY=0; //�Ӹ��� y ��ǥ

int g_LX=0; //�޹� x ��ǥ
int g_LY=0; //�޹� y ��ǥ

int g_RX=0; //������ x ��ǥ
int g_RY=0; //������ y ��ǥ

int g_roiX1=0;// ��ȿ roi x��ǥ
int g_roiY1=0;// ��ȿ roi y��ǥ

int g_xmove; //89
int g_ymove; //50
int g_xresize;// color calibration vlaue x 119
int g_yresize;//color calibration vlaue y 98

int g_irAX =0;  // ���ܼ� ��Ŀ �ݻ�ü�� x�� �߽ɰ�.
int g_irAY =0;  // ���ܼ� ��Ŀ �ݻ�ü�� y�� �߽ɰ�.

int g_Under_irAX =0; //�޹� ������ ���ܼ� ��Ŀ �ݻ�ü�� x�� �߽ɰ�.
int g_Under_irAY =0; //�޹� ������ ���ܼ� ��Ŀ �ݻ�ü�� y�� �߽ɰ�.

//bool g_captureIr = false;

int g_Head3d = 0;//�Ӹ��� 3d center of mass�� �Ÿ� ��ư
int g_Leg3d = 0; //�ٸ��� 3d center of mass�� �Ÿ� ��ư
int g_HLdist = 0; //�Ӹ��� �ٸ��� ���� �ʱ�ȭ.

double g_HLdistJump = 0; //��Ű���� �Ӹ��� �ٸ��� ���� �ʱ�ȭ.

int g_init3dz = 0; //3d center of mass z ���ذ� ��ư

bool g_initvalue= false; //������ ��Ű stand ��ǥ ���� ��ư

bool g_initJumpValue= false; // ��Ű���� ���� ��ǥ ���� ��ư

bool g_soundClick_S = false; // '������' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_L = false; // '���� �����' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_R = false; // '������ �����' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_F = false; // '��ü����' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_K = false; // '��������' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_D = false; // 'Ȱ��' �Ҹ� �� ���� �鸮�� ����

bool g_soundClick_J = false; // '����' �Ҹ� �� ���� �鸮�� ����

IplImage* g_imgDepth; // ���� �̹��� ����.
IplImage* g_imgDepthRangedGray;// 0~255 ���� �̹��� ����.
IplImage* g_imgBinary; // ����ȭ ���� ����.
IplImage* g_IRimgBinary; // IR ����ȭ ���� ����.
IplImage* g_imgInfra;// ���ܼ� ���� ����.
//IplImage* g_imgInfra8Bit;// ���ܼ� ���� ����.
IplImage* g_imgColor; // color ���� ����.
//IplImage* g_imgColorCal; // color calibration��� ���� ����.
IplImage* g_imgColorPerf;
IplImage* g_imgProcess; // ���μ��� ������ ��� �̹��� ����.
//IplImage* g_imgBMask; // ��� ������ ����ũ 

//IplImage* g_imgIrFirst;// ó�� 0.5�� ���� Ir �̹��� 
IplImage* g_imgdepthFirst;// ó�� 0.5�� ���� ���� �̹��� 

IplImage* g_tempdepth;// calibration �� �̹���.
IplImage* g_colorcalspace;//calibration ��� �̹���.

//IplImage* g_imgInfraDiff;// ����ir img


CvRect g_rtROI; // ��ȿ �簢 ���� ����.

double g_p1 = 0.03; //���� �� ���� ��ȭ��
double g_p2 = 0.03; //���� �� ���� ��ȭ��
double g_Diff_v = 5; //L�� R ������ pixel �Ÿ�, ������ ��Ű�� ��Ű���� �ڼ� ���� ����.
double g_Dist_HM_a = 5; //Dist_HM�� ������
double g_Dist_MC_a = 5; //Dist_MC�� ������
double g_Dist_HC_a = 5; //Dist_HC�� ������
double g_Mz_a = 5; //M.z�� ��� ����
double g_Hx_a = 12; //H.x�� ������
double g_Mx_a = 12; //M.x�� ������
double g_Df_v = 0.7; //Dist_HC�� ���Ͽ� Ȱ���� �����ϴ� ���� ����
double g_Jp_v = 1.1; //Dist_HC�� ���Ͽ� Ȱ���� �����ϴ� ���� ����

bool g_bRun=true;  // ���α׷� ���� true�� �ʱ�ȭ.
char g_strFps[20]="0.00fps";  //fps����.
char g_strX[20]="X=0"; // 3���� �߽ɰ� X ����.
char g_strY[20]="Y=0"; // 3���� �߽ɰ� Y ����.
char g_strZ[20]="Z=0"; // 3���� �߽ɰ� Z ����.

char g_strX2[20]="X=0"; // 2���� �߽ɰ� X ����.
char g_strY2[20]="Y=0"; // 2���� �߽ɰ� Y ����.

char g_strHX[20]="X=0"; // �Ӹ� �� X ����.
char g_strHY[20]="Y=0"; // �Ӹ� �� Y ����.

char g_strLX[20]="X=0"; // �޹� �� X ����.
char g_strLY[20]="Y=0"; // �޹� �� Y ����.

char g_strRX[20]="X=0"; // ������ �� X ����.
char g_strRY[20]="Y=0"; // ������ �� Y ����.

char g_strV[50]=""; // �ڼ� ����

char g_testHLdistJ[50]=""; // �Ӹ��� �ٸ� �Ÿ� ǥ��.
char g_testHLdistJ2[50]=""; // �Ӹ��� �ٸ� �Ÿ� ǥ��.


int _tmain(int argc, _TCHAR* argv[]) //���� �Լ��� ���������� �ش� �Լ��� ����.
{
	initProgram();// ���α׷� �ʱ�ȭ.
	while(g_bRun) // ���α׷� �ݺ� ���� �κ�.
	{
		int t=GetTickCount(); // fps�� ���ϱ� ���� count.

		getDepthFrame(); //���̰� �޾ƿ��� �Լ�.
		getDepthRanged(); // �޾ƿ� ���� ���� ���� ���� �Լ�.
		//getBinary(); // �޾ƿ� ���̰��� ������� ����ȭ ���� �����.
		processFrame(); // ������ �۾� ���� �Լ�.
		processKeyEvent(cvWaitKey(1));// key event�� �ޱ� ���� �Լ�.


		t=GetTickCount()-t;// fps�� ���ϱ� ���� count�� ��.
		sprintf(g_strFps,"%.2ffps",1000.0f/t);// fps ����.
	}
	exitProgram(); // ���α׷� ������ �Լ�.

	return 0;
}
void initProgram()// ���α׷� �ʱ�ȭ.
{
	readSettingsFromINI();//ini ���� �б� .
	initCamera(); //ī�޶� �ʱ�ȭ.

	g_imgDepth=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_16U,1); // ���� �̹����� 16���� 1ä�� �̹����� ����.
	g_imgDepthRangedGray=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 0~255�� ��ȯ�� ���� �̹����� 8���� 1ä�� �̹����� ����.
	g_imgProcess=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// �۾���  �̹����� 8���� 1ä�� �̹����� ����.
	g_imgBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// ����ȭ �̹����� 8���� 1ä�� �̹����� ����.
	g_IRimgBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// ir �̹��� 16����1ä�η� ����
	//g_imgInfra8Bit=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// ���ܼ� �̹����� 16���� 1ä�� �̹����� ����.
	g_imgInfra=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// ���ܼ� �̹����� 16���� 1ä�� �̹����� ����.
	g_imgColor=cvCreateImage(cvSize(1920,1080),8,4);// color �̹��� ����.
	g_imgColorPerf=cvCreateImage(cvSize(672,378),8,4);
	//g_imgColorCal= cvCreateImage(cvSize(512,424),8,4);
	//g_imgColorCal= cvCreateImage(cvSize(1920 - g_xresize ,1080 - g_yresize),8,4);// mapping Ȥ�� calibration �� ����.
	//g_colorcalspace=cvCreateImage(cvSize(512,424),8,4);// claibration ��� ����
	g_memStorage=cvCreateMemStorage(); // �ӽ� ������� ����.
	//g_imgBMask=cvLoadImage("8Bit1ChMask.jpg",CV_LOAD_IMAGE_UNCHANGED);// ����ũ �޾ƿ��� �̹���.
	//g_imgIrFirst=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// ���ܼ� ������ ���� �̹��� 
	//g_imgInfraDiff=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1); //���ܼ� ipl�̹��� ������ ��� �̹���
	g_imgdepthFirst=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//���� �̹��� ������ ���� �̹���
	g_tempdepth=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,4);

	g_pColorCoordinates= new ColorSpacePoint[512 * 424];// color mapping ��ǥ �� �ʱ�ȭ

	//cvShowImage("8Bit1ChMask",g_imgBMask); // ������ ����ũ�� ������.

	cvNamedWindow("control",1);// ��Ʈ��â �̸� ����.
	cvResizeWindow("control",440,450);// ��Ʈ�� â ũ�� ����.450
	//cvCreateTrackbar("Confidence","control",(int*)&g_uConfidence,9999,setConfidence);//kinect2�� �ƴ� �ٸ� ī�޶󿡼� ���.
	cvCreateTrackbar("RangeMin","control",&g_uRangeMin,g_uMaxDepth,0);// ���� �̹����� �ּ� �Ÿ� ���� ��ȯ Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("RangeMax","control",&g_uRangeMax,g_uMaxDepth,0);// ���� �̹����� �ִ� �Ÿ� ���� ��ȯ Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("ThMin","control",&g_uThresholdMin,255,0);// ����ȭ �̹����� �ּ� �Ӱ谪 ��ȯ Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("ThMax","control",&g_uThresholdMax,255,0);// ����ȭ �̹����� �ִ� �Ӱ谪 ��ȯ Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("InfraMin","control",&g_uMinInfra,30000,0);// ���ܼ� �̹����� �ּ� �Ÿ� ���� ��ȯ Ʈ���ٸ� ��Ʈ�� â�� �߰�.

	cvCreateTrackbar("ROI.X","control",&g_rtROI.x,512,0);// ��ȿ ���� x0 �� ���� Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("ROI.Y","control",&g_rtROI.y,424,0);// ��ȿ ���� y0 �� ���� Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("ROI.Width","control",&g_rtROI.width,512,0);// ��ȿ ���� x1 �� ���� Ʈ���ٸ� ��Ʈ�� â�� �߰�.
	cvCreateTrackbar("ROI.Height","control",&g_rtROI.height,424,0);// ��ȿ ���� y2 �� ���� Ʈ���ٸ� ��Ʈ�� â�� �߰�.


}

void readSettingsFromINI()// ini �ʱ�ȭ.
{
	TCHAR path[512];
	GetCurrentDirectory(512,path);  //������Ʈ ���.
	wcscat(path,L"\\camera.ini");// ī�޶� ���� ini ����.
	g_uFrameRate=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("FrameRate"),30,path); //framerate ����.
	g_uMinInfra=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("MinInfrared "),30,path);//�ּ� ���ܼ� ����.
	//g_uConfidence=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("Confidence"),1,path);//kinect1����.

	GetCurrentDirectory(512,path);  //������Ʈ ���.
	wcscat(path,L"\\program.ini");// program.ini�� �б�.
	g_uRangeMin=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("RangeMin"),1,path);//�Կ� �ּ� ����.
	g_uRangeMax=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("RangeMax"),1,path);//�Կ� �ִ� ����.
	if(g_uRangeMin==-1)
		g_uRangeMin=g_uMinDepth;// �߸����� �� ���� �Ÿ� ���� ���� ���� ���� ó��.
	if(g_uRangeMax==-1)
		g_uRangeMax=g_uMaxDepth;// �߸����� �� ���� �Ÿ� ���� ���� ���� ���� ó��.

	//�����ϰ� ���� ���� �����ؼ� �ڸ��� ����.
	g_rtROI.x=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIX0"),0,path);// ������ x��ǥ.
	g_rtROI.y=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIY0"),0,path);// ������ y��ǥ.
	g_rtROI.width=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIX1"),0,path)-g_rtROI.x;// ROI ���� ����.
	g_rtROI.height=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIY1"),0,path)-g_rtROI.y;//ROI ���� ����.
	//////////////////////////////////////////////////////////////////////////////////////////////////

	g_roiX1=g_rtROI.width+g_rtROI.x;//��ȿ ROI�� ���� x
	g_roiY1=g_rtROI.height+g_rtROI.y;// ��ȿ ROI�� ���� y

	g_p1 = GetPrivateProfileInt(TEXT("Process"),TEXT("p1"),0,path);
	g_p2 = GetPrivateProfileInt(TEXT("Process"),TEXT("p2"),0,path);
	g_Diff_v = GetPrivateProfileInt(TEXT("Process"),TEXT("Diff_v"),0,path);
	g_Dist_HC_a = GetPrivateProfileInt(TEXT("Process"),TEXT("Dist_HC_a"),0,path);
	g_Dist_HM_a = GetPrivateProfileInt(TEXT("Process"),TEXT("Dist_HM_a"),0,path);
	g_Dist_MC_a = GetPrivateProfileInt(TEXT("Process"),TEXT("Dist_MC_a"),0,path);
	g_Mz_a = GetPrivateProfileInt(TEXT("Process"),TEXT("M.z_a"),0,path);
	g_Hx_a = GetPrivateProfileInt(TEXT("Process"),TEXT("H.x_a"),0,path);
	g_Mx_a = GetPrivateProfileInt(TEXT("Process"),TEXT("M.x_a"),0,path);
	g_Df_v = GetPrivateProfileInt(TEXT("Process"),TEXT("Df_v"),0,path);
	g_Jp_v = GetPrivateProfileInt(TEXT("Process"),TEXT("Jp_v"),0,path);

}

void initCamera() //ī�޶� �ʱ�ȭ.
{
	InitKinectV2();//kinect2 �ʱ�ȭ.
}

void getDepthFrame() // ���� �� �� �÷��� �ޱ�(kinect2 ���̺귯���� ����.)
{
	int SumX = 0; //���ܼ� ��Ŀ �ݻ�ü x ��ǥ ����
	int SumY = 0; //���ܼ� ��ó �ݻ�ü y ��ǥ ����.
	int num  = 0; // �ݻ�ü�� �� ����
	int irAX =0;  // ���ܼ� ��Ŀ �ݻ�ü�� x�� �߽ɰ�.
	int irAY =0;  // ���ܼ� ��Ŀ �ݻ�ü�� y�� �߽ɰ�.

	int Under_SumX = 0; //�޹� ������ ���ܼ� ��Ŀ �ݻ�ü x ��ǥ ����
	int Under_SumY = 0; //�޹� ���������ܼ� ��ó �ݻ�ü y ��ǥ ����.
	int Under_num  = 0; //�޹� ������ �ݻ�ü�� �� ����
	int Under_irAX =0; //�޹� ������ ���ܼ� ��Ŀ �ݻ�ü�� x�� �߽ɰ�.
	int Under_irAY =0; //�޹� ������ ���ܼ� ��Ŀ �ݻ�ü�� y�� �߽ɰ�.

	IDepthFrame* pDepthFrame = NULL; // ���� ������ �ʱ�ȭ
	UINT nBufferSize = 0; // ���� ������ �ʱ�ȭ

	//g_pColorCoordinates= new ColorSpacePoint[512 * 424];

	IplImage* imgDepth=cvCreateImageHeader(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_16U,1);// ���� �̹����� �ش� ����.

	HRESULT hr=g_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//�ֽ� ���� �������� �����ϱ�.
	while(FAILED(hr))// hr���� �����͸� �޴� ���� ������ ���.
		hr=g_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//�ֽ� ���� �������� �����ϱ�.
	if (SUCCEEDED(hr))//hr���� �����͸� �޴� ���� ������ ���.
	{
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16**)&imgDepth->imageData);//���ۿ��� �ٷ� �����Ͽ� ���� �����͸� �޾ƿ�.
		cvCopy(imgDepth,g_imgDepth);//�����̹��������� ���� ���� �̹����� ����.
	}

	cvReleaseImageHeader(&imgDepth); //���� �̹��� ��� �޸� ����.
	SafeRelease(pDepthFrame); // ���� ������ �����ϰ� �޸� ����.

	IInfraredFrame* pInfraredFrame = NULL; //���ܼ� ������ �ʱ�ȭ.

	IplImage* imgInfra=cvCreateImageHeader(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1); //���ܼ� ipl�̹����ش� ����.
	hr=g_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);//hr�� �ֽ� ���ܼ� ������ ���� ����.
	while(FAILED(hr))//hr���� �����͸� �޴� ���� ������ ���
		hr=g_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);//hr�� �ֽ� ���ܼ� ������ ���� ����.
	if (SUCCEEDED(hr))//hr���� �����͸� �޴� ���� ������ ���.
	{
		hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16**)&imgInfra->imageData); //���ܼ����� ���� ������ g_imgInfra�� ����
		cvCopy(imgInfra,g_imgInfra);// �����̹��������� ���� �̹��� ������ ����.
		//if(GetTickCount()>=500 && g_captureIr==false){ //0.5�� ��ٸ��� �ʱ⿡ ir�̹����� capture���� �ʾ����� ir�̹��� capture.
		//	cvCopy(imgInfra,g_imgIrFirst);//local ���ܼ� �̹����� global ���ܼ� �̹����� ����.
		//	g_captureIr=true;// ���İ� �Ǿ��ٴ� ��ư.
		//}
	}

	//cvShowImage("ir First",g_imgIrFirst);

	cvReleaseImageHeader(&imgInfra);//�پ� ipl �̹����� �޸� ����
	SafeRelease(pInfraredFrame); // ���� �޸� ����

	//cvCopy(g_imgInfra,g_imgInfraDiff);// ���ܼ� �̹����� g_imgInfraDiff�� ����.
	//cvAbsDiff(g_imgInfra,g_imgIrFirst,g_imgInfraDiff);// g_imgInfra - g_imgIrFirst =g_imgInfraDiff 

	//////////////////////////////////////////////////////////////////////////////////////////
	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //Ž���������� point ����ġ ���.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{
			if(GET2D16U(g_imgInfra,x,y)>=65535){//Ž�������� ��ü�� �ִ��� ������ Ȯ��.
				SumX = SumX+x;// x��ǥ�� ������Ŵ.
				SumY = SumY+y;// y��ǥ�� ������Ŵ.
				num= num+1; //������Ų ���� count.
			}
			else{ // �ݻ�ü�� ���� �� 
				irAX = 0;// 0���� ����ó�� 
				irAY = 0;// 0���� ����ó��
				g_HX=0;// 0���� ����ó��
				g_HY=0;// 0���� ����ó��
				g_LX=0;// 0���� ����ó��
				g_LY=0;// 0���� ����ó��
				g_RX=0;// 0���� ����ó��
				g_RY=0;// 0���� ����ó��
				num=num;// count �ֱ�ȭ.
			}

		}
	}
	if(num!=0){ //��ü�� �ִٸ�.
		irAX = SumX / num;//x��ǥ ��հ��� ����.
		irAY = SumY / num;//y��ǥ ��հ��� ����.
		g_irAX = irAX;
		g_irAY = irAY;
	}
	else{//��ü�� ���� ��Ȳ�� ����ó��.
		irAX = 0;// 0���� ����ó��
		irAY = 0;// 0���� ����ó��
		g_irAX = 0;// 0���� ����ó��
		g_irAY = 0;// 0���� ����ó��
	}

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) // �Ӹ� ��Ŀ�� ������ �� ��ġ ��Ŀ Ž��
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)//��ȿ x ��ǥ �������� Ž�� 
		{
			if(GET2D16U(g_imgInfra,x,y)>=65535&& y>irAY){ // �ݻ�ü �߽� �� y ������ �Ʒ����� Ž��.
				Under_SumX = Under_SumX+x; //��ü�� x��ǥ ����
				Under_SumY = Under_SumY+y;//��ü�� y��ǥ ����
				Under_num= Under_num+1; // ���� ������ ����
			}
			else{
				Under_irAX = 0; //0���� �ʱ�ȭ
				Under_irAY = 0; //0���� �ʱ�ȭ
				Under_num=Under_num;// ���� ���� ���� ������ �ʱ�ȭ
			}

		}
	}
	if(Under_num!=0){ //��ü�� �ִٸ�
		Under_irAX = Under_SumX / Under_num; //������ x��ǥ ������ ���� ������ ��.
		Under_irAY = Under_SumY / Under_num; //������ y��ǥ ������ ���� ������ ��.

		g_Under_irAX = Under_irAX; // ��� ���� ���� ������ ����.
		g_Under_irAY = Under_irAY; // ��� ���� ���� ������ ����.
	}
	else{//��ü�� ���� ��Ȳ�� ����ó��.
		Under_irAX = 0;// 0���� ����ó��
		Under_irAY = 0;// 0���� ����ó��
		g_Under_irAX = 0;// 0���� ����ó��
		g_Under_irAY = 0;// 0���� ����ó��
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(int y=0;y<DEPTH_HEIGHT;y++) 
	{
		for(int x=0;x<DEPTH_WIDTH;x++)
		{

			unsigned short val=GET2D16U(g_imgInfra,x,y); //ir �̹������� ���ܼ��� ��� ���� val�� ����
			unsigned short val2=GET2D16U(g_imgInfra,x,y); //ir �̹������� ���ܼ��� ��� ���� val�� ����
			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=65535 && y<irAY  ){// �˻� ��ȿ �������� val�� ���� ���� ���� ��� ������ �˻�
				g_HX=x;// �Ӹ� x ��ǥ�� ����
				g_HY=y;// �Ӹ� y ��ǥ�� ����
			}

			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=65535 && y>g_HY && y>irAY && x< Under_irAX  ){ // �� �߽� x��ǥ���� �� ������ �޹� ��ǥ.
				g_LX=x;// �޹� x ��ǥ�� ����
				g_LY=y;// �޹� y ��ǥ�� ����
			}

			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=5535&& y>g_HY && x>g_LX && y>irAY && x>=Under_irAX ){ // �� �߽� x��ǥ���� �� ũ�� ������ ��ǥ.
				g_RX=x;// ������ x ��ǥ�� ����
				g_RY=y;// ������ y ��ǥ�� ����
			}

			if(val2 <g_uMinInfra){ // g_uMinInfra ���� ��ο� ���� 0���� �ʱ�ȭ-> ��� ó��
				GET2D16U(g_imgDepth,x,y)=0;//0���� �ʱ�ȭ-> ��� ó��
			}

		}
	}

	IColorFrame* pColorFrame = NULL; // color ������ NULL�� �ʱ�ȭ
	nBufferSize = 0; //���� ������ 0���� �ʱ�ȭ

	IplImage* imgColor=cvCreateImageHeader(cvSize(1920,1080),8,4); //color �̹��� ��� ����.

	hr=g_pColorFrameReader->AcquireLatestFrame(&pColorFrame); //ī�޶󿡼� color ���� �޾ƿ�.
	while(FAILED(hr)) // hr�� �߸��� ���� ������
		hr=g_pColorFrameReader->AcquireLatestFrame(&pColorFrame); //�ٽ� color ���� ������.
	if (SUCCEEDED(hr)) // �����ϸ�
	{
		// ������ �����͸� �迭�� ��ȯ �Ͽ� ����.
		pColorFrame->CopyConvertedFrameDataToArray(1920*1080*sizeof(RGBQUAD), reinterpret_cast<BYTE*>(g_imgColor->imageData), ColorImageFormat_Bgra);
		/*hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, (BYTE**)&imgColor->imageData);
		cvCopy(imgColor,g_imgColor);*/
	}

	//cvConvertImage(g_imgColor,g_imgColor_quad,1./5);
	cvResize(g_imgColor,g_imgColorPerf,CV_INTER_CUBIC); // g_imgColor�� g_imgColorPerf�� ������ ��ȯ.

	//MapColorFrameToDepthSpace ���.
	//hr = g_pCoordinateMapper->MapDepthFrameToColorSpace(DEPTH_WIDTH  * DEPTH_HEIGHT,(UINT16*)(g_imgDepth->imageData),512*424,g_pColorCoordinates);
	//cvZero(g_imgColorCal);
	//for(int y=0;y<424;y++)
	//{
	//	for(int x=0;x<512;x++)
	//	{
	//		ColorSpacePoint p = g_pColorCoordinates[y*512+x];

	//		if (GET2D16U(g_imgDepth,x,y)>=0/*p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity()*/)
	//		{
	//			int colorX = static_cast<int>(p.X + 0.5f);
	//			int colorY = static_cast<int>(p.Y + 0.5f);

	//			if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
	//			{
	//				GET2D8U4CH(g_imgColorCal,x,y)[0] = GET2D8U4CH(g_imgColor,colorX,colorY)[0];
	//				GET2D8U4CH(g_imgColorCal,x,y)[1] = GET2D8U4CH(g_imgColor,colorX,colorY)[1];
	//				GET2D8U4CH(g_imgColorCal,x,y)[2] = GET2D8U4CH(g_imgColor,colorX,colorY)[2];
	//				GET2D8U4CH(g_imgColorCal,x,y)[3] = GET2D8U4CH(g_imgColor,colorX,colorY)[3];
	//				
	//			}
	//			
	//		}
	//	}
	//}


	cvReleaseImageHeader(&imgColor); //color ��� �޸� ����.
	SafeRelease(pColorFrame); // pColorFrame ���� ����.

}


void getDepthRanged() //0~255������ ���� ���� ����.
{
	for(int y=0;y<DEPTH_HEIGHT;y++)
	{
		for(int x=0;x<DEPTH_WIDTH;x++)
		{
			unsigned short val=GET2D16U(g_imgDepth,x,y);
			if(val && g_uRangeMin<=val &&  val<=g_uRangeMax)
			{
				GET2D8U(g_imgDepthRangedGray,x,y)=255-(int)  ( (float)(val-g_uRangeMin)/(g_uRangeMax-g_uRangeMin)*254 );
			}
			else
				GET2D8U(g_imgDepthRangedGray,x,y)=0;
		}
	}
}

void getBinary(IplImage* image) // ����ȭ ���� ���� �Լ�
{

	cvThreshold(image,g_imgBinary,g_uThresholdMax,255,CV_THRESH_TOZERO_INV);// ����ȭ ���� �ִ� �Ӱ��� ����
	cvThreshold(g_imgBinary,g_imgBinary,g_uThresholdMin,255,CV_THRESH_BINARY);// ����ȭ ���� �ּ� �Ӱ��� ����

}

void processFrame() //�������� Ʈ��ŷ ����� ���ϴ� �κ�
{

	IplImage* imgShowDepthRangedGray=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1); //���� �� ����� ���� �̹���.
	IplImage* imgShowProcess=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,3);// ���� ��ȿ�� �̹��� ����� ���� �̹���.
	IplImage* imgShowBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,3);// ����ȭ �̹��� 3channel�� ����� ���� �̹���.
	IplImage* img=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//contour�� ���� ���� �̹���.
	IplImage* imgMask=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//contour�� ������ ����ũ�� �ӽ������ϴ� �̹���.
	IplImage* img2=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// �������� �����ϱ� ���� �ӽ� �̹���.

	double Head3d=0;//���� �Ӹ��� 3d center of mass�� �Ÿ�
	double Leg3d=0; //���� �ٸ��� 3d center of mass�� �Ÿ�
	double HLdist=0; //���� �Ӹ��� �ٸ� ������ �Ÿ�

	double HLdistJump=0;//���� ��Ű ���� �Ӹ��� �ٸ� ���� �Ÿ�

	cvCopy(g_imgDepthRangedGray,g_imgProcess);//���� �̹����� �۾��� imgProcess�� ����.

	///////////////////������ ���ó��////////////////////////////////////////////
	//cvCopy(g_imgProcess,img2);//�ӽ� �̹��� img2�� g_imgProcess�� ���� ����
	//cvZero(g_imgProcess); // g_imgProcess�� 0���� �ʱ�ȭ
	//cvCopy(img2,g_imgProcess,g_imgBMask);//img2���� imgMask�� �ش��ϴ� �κ��� imgProcess�� ����--//
	//////////////////////////////////////////////////////////////////////////////////

	//cvShowImage("mask test",g_imgProcess); //����ũ �ؼ� ���ó���� ��� ������.

	//����ϴ� Ư�� ������ ����� ��� ����.
	//cvCopy(g_imgDepthRangedGray,g_imgProcess);//���� �̹����� �۾��� imgProcess�� ����.

	for(int x=0;x<DEPTH_WIDTH;x++)	//������ ���� ���θ� �˻�.
	{
		for(int y=0;y<DEPTH_HEIGHT;y++)
		{
			if(!POINT_IN_RECT(cvPoint(x,y),g_rtROI)) //program ini���� ������ ũ�⸸ŭ �̹��� �簢������ �̿��� �κ� ���������� ����.
				GET2D8U(g_imgProcess,x,y)=0;     
		}
	}//

	cvCopy(g_imgProcess,img);//-- ���� ū ���� �� ��ü�� ���� ������ ��� ����

	CvSeq* contours=0;	//�ܰ���.
	cvFindContours(g_imgProcess,g_memStorage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//�ܰ��� ����.
	CvSeq* select_contour=0;
	float max_area=0;

	while(contours)
	{
		float area=abs(cvContourArea(contours)); //����.
		if(area>max_area)		//������ �ִ� �̹��� ũ�⺸�� �˻��� ũ�Ⱑ ũ�� �ش� �̹����� �ִ� �̹����� ����.
		{
			max_area=area;   //�ִ� �̹��� ũ�� ����.
			select_contour=contours;
		}

		contours=contours->h_next;
	}

	cvZero(imgMask); //imgMask�� 0���� �ʱ�ȭ.
	cvDrawContours(imgMask,select_contour,cvScalar(255),cvScalar(255),0,CV_FILLED);//imgMask�� �߶� �̹��� ����.

	cvClearMemStorage(g_memStorage); //�ܰ��� ���� ������ ������ ����.(������ �޸� ���� ����.)

	cvZero(g_imgProcess);
	cvCopy(img,g_imgProcess,imgMask);//img���� imgMask�� �ش��ϴ� �κ��� imgProcess�� ����--//
	cvMerge(g_imgProcess,g_imgProcess,g_imgProcess,NULL,imgShowProcess);//1ä���� imgProcess�� 3ä�� imgshowprocess�� merge���� ����.

	getBinary(g_imgProcess);

	cvMerge(g_imgBinary,g_imgBinary,g_imgBinary,NULL,imgShowBinary);//1ä���� imgProcess�� 3ä�� imgshowprocess�� merge���� ����.

	cvPutText(imgShowBinary,g_strFps,cvPoint(290,50),&cvFont(3,2),cvScalar(255,255,255));//���� �̹����� fps ǥ��.

	//cvDrawCircle(imgShowProcess, cvPoint(g_3dX,g_3dY),4,CV_RGB(255,0,0),2); // 3���� �߽��� ��ǥ�� imgShowProcess�� ���������� ǥ��. 
	cvDrawRect(imgShowProcess,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1); //������� �κ��� imgShowProcess�� ǥ��.
	cvDrawRect(g_imgDepthRangedGray,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1);//������� �κ� ���� g_imgDepthRangedGray �̹����� ǥ��. 
	cvDrawRect(imgShowBinary,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1);//������� �κ� ���� imgShowBinary �̹����� ǥ��.  

	cvDrawRect(g_imgColorPerf,cvPoint(g_rtROI.x+80,g_rtROI.y-30),cvPoint(g_rtROI.x+g_rtROI.width+85,g_rtROI.y+g_rtROI.height-20),CV_RGB(255,0,0),1);//������� �κ� ���� g_imgColorPerf �̹����� ǥ��. 


	centerOfMass3d(g_imgProcess); //imgProcess�� 3���� �߽��� ��ǥ�� ����.
	centerOfMass2d(g_imgBinary); //g_imgBinary�� 2���� �߽��� ��ǥ�� ����.

	if(g_3dX!=0 || g_3dY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_3dX,g_3dY),4,CV_RGB(255,0,0),2); // 3���� �߽��� ��ǥ�� imgShowProcess�� ���������� ǥ��.
	}

	if(g_2dX!=0 || g_2dY!=0){
		cvDrawRect(imgShowBinary, cvPoint(g_2dX-5,g_2dY-5),cvPoint(g_2dX+5,g_2dY+5),CV_RGB(255,0,0),2); // 3���� �߽��� ��ǥ�� imgShowProcess�� ���������� ǥ��.
	}


	if(distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_HX!=0 && g_HY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_HX,g_HY),4,CV_RGB(230,231,87),2);// �Ӹ� ��Ŀ ������ ǥ��
	}
	if(distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_LX!=0 && g_LY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_LX,g_LY),4,CV_RGB(255,227,25),2);// �޹� ��Ŀ ������ ǥ��
	}
	if( g_RX!=0 && g_RY!=0 || distpoints(g_LX,g_LY,g_RX,g_RY)>30 && g_RX!=0 && g_RY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_RX,g_RY),4,CV_RGB(111,167,46),2);// ������ ��Ŀ ������ ǥ��
	}

	sprintf(g_strX,"3D X=%d",g_3dX);// 3���� �߽ɰ� x��ǥ ����.
	sprintf(g_strY,"3D Y=%d",g_3dY);// 3���� �߽ɰ� y��ǥ ����.
	sprintf(g_strZ,"3D Z=%d",g_3dZ);// 3���� �߽ɰ� z��ǥ ����.

	sprintf(g_strX2,"2D X=%d",g_2dX);// 2���� �߽ɰ� x��ǥ ����.
	sprintf(g_strY2,"2D Y=%d",g_2dY);// 2���� �߽ɰ� y��ǥ ����.

	sprintf(g_strHX,"Head X=%d",g_HX);// �Ӹ� ��Ŀ x��ǥ ����.
	sprintf(g_strHY,"Head Y=%d",g_HY);// �Ӹ� ��Ŀ y��ǥ ����.

	sprintf(g_strLX,"Left foot X=%d",g_LX);// �޹� ��Ŀ x��ǥ ����.
	sprintf(g_strLY,"Left foot Y=%d",g_LY);// �޹� ��Ŀ y��ǥ ����.

	sprintf(g_strRX,"Right foot X=%d",g_RX);// ������ ��Ŀ x��ǥ ����.
	sprintf(g_strRY,"Right foot Y=%d",g_RY);// ������ ��Ŀ y��ǥ ����.

	///////////////////////////////////////////////�ڼ� ����///////////////////////////////////////////////////////////////////
	if(g_HX!=0 && g_HY!=0 && g_RX!=0 && g_RY!=0 && distpoints(g_Under_irAX,g_Under_irAY,g_RX,g_RY)<=3){ //��Ű ���� �ڼ� ����
		// ���� �߰� �� �� �߽� ��ǥ�� ���Ͽ� ��Ű �ڼ� ����.
		HLdistJump = distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); // ���� �Ӹ��� �ٸ� �������� �Ÿ�.

		//sprintf(g_testHLdistJ,"prev : %0.2f",g_HLdistJump);// ���� �Ÿ�.
		//cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));// Ȱ�� �ڼ� ǥ��.

		//sprintf(g_testHLdistJ,"current: %0.2f",HLdistJump);// ���� �Ÿ�.
		//cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));// Ȱ�� �ڼ� ǥ��.

		double diffHLJump = g_HLdistJump-HLdistJump;//�Ӹ��� �ٸ� �Ÿ��� ���� �� ���� ���� ����.

		if(diffHLJump<0){ //�Ӹ��� �����ٸ��� �Ÿ��� ���� �� ���� ���� ���� ���ȭ.

			diffHLJump=diffHLJump*(-1);

		}


		if(g_initJumpValue == true && HLdistJump <= (g_HLdistJump)*(0.7) || g_initJumpValue == true && HLdistJump<=100){ //HLdistJump<=120
			//���� ���� ���� 0.7���� �پ� ��ų� ��ȭ�� ���� ��.

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//���ذ� ����.

				if(g_soundClick_D != true){

						sndPlaySoundW(L"sound\\J01_Downfall.wav",SND_ASYNC);

						g_soundClick_L = false;
						g_soundClick_S = false;
						g_soundClick_R = false;
						g_soundClick_F = false;
						g_soundClick_K = false;
						g_soundClick_D = true;
						g_soundClick_J = false;
						
					
					}

			sprintf(g_strV,"Current Pose : Downhill (1)");// Ȱ�� �ڼ�.
			cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// Ȱ�� �ڼ� ǥ��.

			//g_initJumpValue = false;

		}

		if(g_initJumpValue == true && HLdistJump >= (g_HLdistJump)*(1.1) || g_initJumpValue == true && HLdistJump>100){  //HLdistJump>120
			//���� ���� ���� 1.1�� �پ� ��ų� ��ȭ�� ���� ��.

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//���ذ� ����.

					if(g_soundClick_J != true){

						sndPlaySoundW(L"sound\\J02_Jump.wav",SND_ASYNC);

						g_soundClick_L = false;
						g_soundClick_S = false;
						g_soundClick_R = false;
						g_soundClick_F = false;
						g_soundClick_K = false;
						g_soundClick_D = false;
						g_soundClick_J = true;
						
					
					}

			sprintf(g_strV,"Current Pose : Jump (2)");// ��Ű���� �ڼ�.
			cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));//���� �ڼ� ǥ��.

			//g_initJumpValue = false;

		}

		if(g_initJumpValue== false && HLdistJump>140 ){ // ��Ű ���� �ʱ�ȭ

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//���ذ� ����.

			//sprintf(g_strV,"Current Pose : Downhill Ready ");// Ȱ�� �غ�.
			//cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �غ� �ڼ� ǥ��.

			g_initJumpValue = true;

		}
	}
	else if(distpoints(g_Under_irAX,g_Under_irAY,g_RX,g_RY)>5 && g_LY!=0 && g_RY!=0 ){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		        
		        double t_HXC;
				double t_3dXC;
		
		if(g_Under_irAX<256){
			

			    g_HX = g_HX+(42*(256-g_Under_irAX)/256); // ȭ���� ���� x �� ���� ����.
				g_3dX =g_3dX+(22*(256-g_Under_irAX)/256);
				
			
				
				t_HXC = g_HX;
				t_3dXC = g_3dX;

				sprintf(g_testHLdistJ,"Head.x : %0.2f",t_HXC);// �Ӹ� ���� ��.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));

		        sprintf(g_testHLdistJ,"Center.x: %0.2f",t_3dXC);// 3D ���� ��.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));

			}
			if(g_Under_irAX>256){
			
				g_HX = g_HX-(42*(g_Under_irAX-256)/256); // ȭ���� ���� x �� ���� ����.
				g_3dX =g_3dX-(22*(g_Under_irAX-256)/256);

				t_HXC = g_HX; // ȭ���� ���� x �� ���� ����.
				t_3dXC = g_3dX;

				sprintf(g_testHLdistJ,"Head.x : %0.2f",t_HXC);// �Ӹ� ���� ��.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));

		        sprintf(g_testHLdistJ,"Center.x: %0.2f",t_3dXC);// 3D ���� ��.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));

			}
		
		if(g_initvalue==true && distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_3dZ!=0)
		{// stand �ڼ� �� ���� �� �ٸ� �ڼ� ����.

			Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// ���� �Ӹ��� 3d ���� ���� 
			Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// ���� �ٸ��� 3d ���� ����
			HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);// ���� �Ӹ��� �ٸ� ����
			

			double diffdistH=g_Head3d - Head3d;// stand �� ���� Head3d �Ÿ� ���� ���ϱ�
			diffdistH=fabs(diffdistH);

			double diffdistL=g_Leg3d - Leg3d;// ���ذ� �� ������ Leg3d  �Ÿ� ���� ���ϱ�
			diffdistL=fabs(diffdistL);

			double diff3dz=g_init3dz - g_3dZ;// stand �� ���� 3dz �Ÿ� ���� ���ϱ�
			diff3dz=fabs(diff3dz);

			double diffHead3dx=g_3dX - g_HX;//stand�� �Ӹ��� 3d center of mass x ��ǥ ����. 
			diffHead3dx=fabs(diffHead3dx);

			double diffHLdist=g_HLdist - HLdist;// �Ӹ��� �ٸ� ��ǥ ���ذ� ������ ����. 
			diffHLdist=fabs(diffHLdist);

			double L3d_x = g_Under_irAX-g_3dX; // ��, �� ����Ӱ� ������ �α� ���� ���� ����.
			L3d_x = fabs(L3d_x);

			if(g_Under_irAX<256){

				diffdistH = diffdistH*(1-(g_p1/256)*(256-g_Under_irAX)); //ȭ���� ���� y �� ���� ����.
				diffdistL = diffdistL*(1-(g_p1/256)*(256-g_Under_irAX)); //ȭ���� ���� y �� ���� ����.
				diffHLdist = diffHLdist*(1-(g_p1/256)*(256-g_Under_irAX)); //ȭ���� ���� y �� ���� ����.

				
			}
			if(g_Under_irAX>256){
			
				diffdistH = diffdistH*(1-(g_p2/256)*(g_Under_irAX-256)); //ȭ���� ���� y �� ���� ����.
				diffdistL = diffdistL*(1-(g_p2/256)*(g_Under_irAX-256)); //ȭ���� ���� y �� ���� ����.
				diffHLdist = diffHLdist*(1-(g_p2/256)*(g_Under_irAX-256)); //ȭ���� ���� y �� ���� ����.

			}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
			
			
			if(  g_initvalue == true  && Head3d >= (g_Head3d*1.1)&& Leg3d >= (g_Leg3d*1.1) && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true  && HLdist>100 && Head3d>30 && Leg3d>70 && L3d_x<=12 && g_HX>g_LX &&g_HX<g_RX){

				g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand�� �Ӹ��� �ٸ��� ���� �Ÿ� �ʱ�ȭ
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand�� �Ӹ��� 3d center of mass �Ÿ� �ʱ�ȭ.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand�� �ٸ��� 3d center of mass �Ÿ� �ʱ�ȭ
				g_init3dz=g_3dZ;

				if(g_soundClick_S != true){

						sndPlaySoundW(L"sound\\A01_Stand.wav",SND_ASYNC);

						g_soundClick_L = false;
						g_soundClick_S = true;
						g_soundClick_R = false;
						g_soundClick_F = false;
						g_soundClick_K = false;
						g_soundClick_D = false;
						g_soundClick_J = false;
						
					
					}
				sprintf(g_strV,"Current Pose : Stand (1)");// ������.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �ڼ� ǥ��.

			}

			if( g_HLdist > HLdist && Head3d <= (g_Head3d*0.7)  && Leg3d >= (g_Leg3d*1.1)  && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true && HLdist<=100 && Head3d<=30 && Leg3d>70 && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX){

		     	g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand�� �Ӹ��� �ٸ��� ���� �Ÿ� �ʱ�ȭ
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand�� �Ӹ��� 3d center of mass �Ÿ� �ʱ�ȭ.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand�� �ٸ��� 3d center of mass �Ÿ� �ʱ�ȭ
				g_init3dz=g_3dZ;

				if(g_soundClick_F != true){

						sndPlaySoundW(L"sound\\A02_BendingForward.wav",SND_ASYNC);

						g_soundClick_L = false;
						g_soundClick_S = false;
						g_soundClick_R = false;
						g_soundClick_F = true;
						g_soundClick_K = false;
						g_soundClick_D = false;
						g_soundClick_J = false;
						
					
					}
				sprintf(g_strV,"Current Pose : Bending Forward (2)");// ��ü����.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �ڼ� ǥ��.

			}

			if( g_HLdist > HLdist && Head3d >= (g_Head3d*1.1)  && Leg3d <= (g_Leg3d*0.7) && L3d_x<=12 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true && HLdist<=100 && Head3d>30 && Leg3d<=70  && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX){

				g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand�� �Ӹ��� �ٸ��� ���� �Ÿ� �ʱ�ȭ
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand�� �Ӹ��� 3d center of mass �Ÿ� �ʱ�ȭ.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand�� �ٸ��� 3d center of mass �Ÿ� �ʱ�ȭ
				g_init3dz=g_3dZ;

				if(g_soundClick_K != true){

						sndPlaySoundW(L"sound\\A03_Bendingknees.wav",SND_ASYNC);

						g_soundClick_L = false;
						g_soundClick_S = false;
						g_soundClick_R = false;
						g_soundClick_F = false;
						g_soundClick_K = true;
						g_soundClick_D = false;
						g_soundClick_J = false;
						
					
					}
				sprintf(g_strV,"Current Pose : Bending Knees (3)");// �������θ�.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �ڼ� ǥ��.

			}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if( g_HY!=0 && g_LY!=0 && g_RY!=0 && g_3dX<g_Under_irAX && (g_Under_irAX-g_3dX)>14 && g_HX<g_LX || // 3���� �߽����� ��� ���� ��.
				g_HY!=0 && g_LY!=0 && g_RY!=0 && g_HX<g_LX && (g_3dX-g_HX)>12){ //�Ӹ��� 3���� �߽��� ��.

					if(g_soundClick_L != true){

						sndPlaySoundW(L"sound\\A05_TiltLeft.wav",SND_ASYNC);

						g_soundClick_L = true;
						g_soundClick_S = false;
						g_soundClick_R = false;
						g_soundClick_F = false;
						g_soundClick_K = false;
						g_soundClick_D = false;
						g_soundClick_J = false;
						
					
					}
					
					sprintf(g_strV,"Current Pose : Tilt Left (1)");// ���� ����.
					cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �ڼ� ǥ��.

			}
			if(g_HY!=0 && g_LY!=0 && g_RY!=0 && g_3dX>g_Under_irAX && (g_3dX-g_Under_irAX)>14 && g_HX>g_RX ||  // �߽����� ��� ���� ��.
				g_HY!=0 && g_LY!=0 && g_RY!=0 && g_HX>g_RX && (g_HX-g_3dX)>12){ //�Ӹ��� 3���� �߽��� ��.

					if(g_soundClick_R != true){

						sndPlaySoundW(L"sound\\A04_TiltRight.wav",SND_ASYNC);

						g_soundClick_R = true;
						g_soundClick_L = false;
						g_soundClick_S = false;
						g_soundClick_F = false;
						g_soundClick_K = false;
						g_soundClick_D = false;
						g_soundClick_J = false;
					}

					
					sprintf(g_strV,"Current Pose : Tilt Right (2)");// ������ ����.
					cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// �ڼ� ǥ��.


			}
		}


		if(g_HX!=0&& g_HY!=0 && distpoints(g_Under_irAX,g_Under_irAY,g_HX,g_HY)>140 && g_3dZ!=0 && g_initvalue==false && g_3dZ<=170 ){
			//Stand �ڼ� ���� �� ����.
			g_init3dz=g_3dZ; //3d z �� �ʱ�ȭ.
			g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand�� �Ӹ��� 3d center of mass �Ÿ� �ʱ�ȭ.
			g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand�� �ٸ��� 3d center of mass �Ÿ� �ʱ�ȭ
			g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand�� �Ӹ��� �ٸ��� ���� �Ÿ� �ʱ�ȭ
			g_initvalue= true; //���� flag

		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cvPutText(imgShowProcess,g_strX,cvPoint(10,370),&cvFont(1,1),cvScalar(255,255,255));// 3���� �߽ɰ� x��ǥ ǥ��.
	cvPutText(imgShowProcess,g_strY,cvPoint(10,385),&cvFont(1,1),cvScalar(255,255,255));// 3���� �߽ɰ� y��ǥ ǥ��.
	cvPutText(imgShowProcess,g_strZ,cvPoint(10,400),&cvFont(1,1),cvScalar(255,255,255));// 3���� �߽ɰ� z��ǥ ǥ��.

	cvPutText(imgShowBinary,g_strX2,cvPoint(10,370),&cvFont(1,1),cvScalar(255,255,255));// 2���� �߽ɰ� x��ǥ ǥ��.
	cvPutText(imgShowBinary,g_strY2,cvPoint(10,385),&cvFont(1,1),cvScalar(255,255,255));// 2���� �߽ɰ� y��ǥ ǥ��.

	cvPutText(imgShowProcess,g_strHX,cvPoint(100,370),&cvFont(1,1),cvScalar(255,255,255));//�Ӹ� ��Ŀ x��ǥ ǥ��
	cvPutText(imgShowProcess,g_strHY,cvPoint(100,385),&cvFont(1,1),cvScalar(255,255,255));//�Ӹ� ��Ŀ y��ǥ ǥ��

	cvPutText(imgShowProcess,g_strLX,cvPoint(210,370),&cvFont(1,1),cvScalar(255,255,255));//�޹� ��Ŀ x��ǥ ǥ��
	cvPutText(imgShowProcess,g_strLY,cvPoint(210,385),&cvFont(1,1),cvScalar(255,255,255));//�޹� ��Ŀ y��ǥ ǥ��

	cvPutText(imgShowProcess,g_strRX,cvPoint(355,370),&cvFont(1,1),cvScalar(255,255,255));//������ ��Ŀ x��ǥ ǥ��
	cvPutText(imgShowProcess,g_strRY,cvPoint(355,385),&cvFont(1,1),cvScalar(255,255,255));//������ ��Ŀ y��ǥ ǥ��




	cvShowImage("depth",g_imgDepthRangedGray);//���� ���� ��� �󺧸�.
	cvShowImage("binary",imgShowBinary); // ����ȭ ���� ���.

	//cvShowImage("IR binary",g_IRimgBinary);//IR �̹��� ����ȭ����

	cvShowImage("IR",g_imgInfra); //���ܼ� �̹��� ���.
	cvShowImage("�ڼ� ���� ȭ��",imgShowProcess); //���� ��� ������.

	//cvShowImage("Color_calibration mapping",g_imgColorCal);// color ����
	cvShowImage("Color",g_imgColorPerf);// color ����

	cvReleaseImage(&imgShowDepthRangedGray);//�̹��� ���� �޸� ���� .
	cvReleaseImage(&imgShowBinary);//�̹��� ���� �޸� ����.
	cvReleaseImage(&imgShowProcess);//�̹��� ���� �޸� ����.
	cvReleaseImage(&img);//�̹��� ���� �޸� ����.
	cvReleaseImage(&imgMask);//�̹��� ���� �޸� ����.
	cvReleaseImage(&img2);//�̹��� ���� �޸� ����.


}
void centerOfMass3d (IplImage* image) // 3���� �߽����� ���ϴ� �Լ�.
{
	int SumX = 0;// 0���� �ʱ�ȭ
	int SumY = 0;// 0���� �ʱ�ȭ
	int SumZ = 0;// 0���� �ʱ�ȭ
	int num  = 0; // 0���� �ʱ�ȭ

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //Ž���������� point ����ġ ���.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{

			if(GET2D8U(image,x,y)!=0){//Ž�������� ��ü�� �ִ��� ������ Ȯ��.
				SumX = SumX+x;
				SumY = SumY+y;
				SumZ = SumZ + GET2D8U(image,x,y);
				num= num+1;
			}
			else{
				g_3dX = 0;//0���� �ʱ�ȭ
				g_3dY = 0;//0���� �ʱ�ȭ
				g_3dZ = 0;//0���� �ʱ�ȭ
				num=num;
			}

		}
	}
	if(num!=0){ //��ü�� ������ 3���� �߽ɰ��� ���ϰ�
		g_3dX = SumX / num;
		g_3dY = SumY / num;
		g_3dZ = SumZ / num;
	}
	else{// ������ 0���� �ʱ�ȭ
		g_3dX = 0;
		g_3dY = 0;
		g_3dZ = 0;
	}

	// The coordinate (SumX,SumY) is the center of the image mass.
}

void centerOfMass2d (IplImage* image) // 2���� �߽����� ���ϴ� �Լ�.
{
	int SumX = 0;
	int SumY = 0;
	int num  = 0; 

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //Ž���������� point ����ġ ���.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{

			if(GET2D8U(image,x,y)!=0){//Ž�������� ��ü�� �ִ��� ������ Ȯ��.
				SumX = SumX+x;
				SumY = SumY+y;
				num= num+1;
			}
			else{
				g_2dX = 0;
				g_2dY = 0;
				num=num;
			}

		}
	}
	if(num!=0){ //��ü�� ���� ��Ȳ�� ����ó��.
		g_2dX = SumX / num;
		g_2dY = SumY / num;
	}
	else{
		g_2dX = 0;
		g_2dY = 0;
	}// The coordinate (SumX,SumY) is the center of the image mass.
}

int distpoints(int x0, int y0 , int x1, int y1){

	return  sqrt((x0-x1)*(x0-x1)+(y0-y1)*(y0-y1));

}
void exitProgram()// ���α׷� ����� �޸� ����.
{
	exitCamera(); // ī�޶� �޸� ���� �Լ� ȣ��.
	cvDestroyAllWindows(); // ���� ���� �����츦 ��� ����.
	cvReleaseImage(&g_imgDepth);// ���� �̹��� �̹��� �޸� ����.
	cvReleaseImage(&g_imgDepthRangedGray);//g_imgDepthRangedGray �޸� ����
	cvReleaseImage(&g_imgBinary);//g_imgBinary ����ȭ�� �޸� ����

	cvReleaseImage(&g_IRimgBinary);//g_IRimgBinary ����ȭ�� �޸� ����
	//cvReleaseImage(&g_imgInfra8Bit);//g_IRimgBinary ����ȭ�� �޸� ����

	cvReleaseImage(&g_imgInfra);// g_imgInfra �޸� ����
	cvReleaseImage(&g_imgProcess);// g_imgProcess �޸� ����
	cvReleaseMemStorage(&g_memStorage);// g_memStorage �޸� ����
	//cvReleaseImage(&g_imgBMask);// g_imgBMask �޸� ����
	//cvReleaseImage(&g_imgIrFirst);// g_imgIrFirst �޸� ����
	//cvReleaseImage(&g_imgInfraDiff);// g_imgInfraDiff �޸� ����

	cvReleaseImage(&g_tempdepth);// calibration �� �̹��� �޸� ����

	cvReleaseImage(&g_imgColor);// g_imgColor �޸� ����
	//cvReleaseImage(&g_imgColorCal);// g_imgColorCal �޸� ����
	cvReleaseImage(&g_imgColorPerf);// g_imgColorPerf �޸� ����
	//cvReleaseImage(&g_colorcalspace);

	cvReleaseImage(&g_imgdepthFirst);// g_imgdepthFirst �޸� ����
}

void exitCamera() //ī�޶� ���� �Լ�.
{
	exitKinectV2(); // Ű��Ʈ2 ����.
}

void processKeyEvent(int key) //Ű���� �̺�Ʈ ����.
{
	switch(key)
	{
	case VK_ESCAPE: // esc Ű�� �޾��� �� ���α׷� ����.
		g_bRun=false;
	default:
		break;
	}
}


void InitKinectV2() //kinect2 �ʱ�ȭ.
{
	HRESULT hr;// ��� hr ����

	hr = GetDefaultKinectSensor(&g_pKinectSensor); // Ű��Ʈ ������ �⺻������ �ʱ�ȭ�Ѵ�.


	if (g_pKinectSensor) // kinect ������ ready �������� Ȯ��.
	{
		// Initialize the Kinect and get the depth reader.
		IDepthFrameSource* pDepthFrameSource = NULL;  // ���� �������� NULL�� �ʱ�ȭ.
		IInfraredFrameSource* pInfraredFrameSource = NULL; // ���ܼ� �������� NULL�� �ʱ�ȭ.
		IColorFrameSource* pColorFrameSource = NULL; // Color ������ �ҽ� NULL�� �ʱ�ȭ.

		hr = g_pKinectSensor->Open(); // kinect�� �۵���Ŵ.

		if (SUCCEEDED(hr)) // kinect�� �۵��� ���� Ȯ��.
		{
			hr = g_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource); //���������Ӽҽ��� hr�� ����.
			hr = g_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);// ���ܼ� �ҽ��� hr�� ����.
			hr = g_pKinectSensor->get_ColorFrameSource(&pColorFrameSource); // color �ҽ��� hr�� ����.
			hr = g_pKinectSensor->get_CoordinateMapper(&g_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))// kinect�� �۵��� ���� Ȯ��
		{
			hr = pDepthFrameSource->OpenReader(&g_pDepthFrameReader); //���� �����Ӽҽ��� OpenReader�� hr�� ����.
			hr = pInfraredFrameSource->OpenReader(&g_pInfraredFrameReader);//���ܼ� �����Ӽҽ��� OpenReader�� hr�� ����.
			hr = pColorFrameSource->OpenReader(&g_pColorFrameReader);// color �����ӿ��� OpenReader�� hr�� ����.
		}

		SafeRelease(pDepthFrameSource); //���������Ӽҽ��� �����ϰ� �޸� ����.
	}

	if (!g_pKinectSensor || FAILED(hr)) //kinect ������ ready ���°� �ƴ�.
	{
		printf("No ready Kinect found!");
	}
}

void exitKinectV2() //kinect2 ī�޶� ����.
{
	// done with depth frame reader.
	SafeRelease(g_pDepthFrameReader);//������������ �о���� �޸� ����
	SafeRelease(g_pInfraredFrameReader);//���ܼ��������� �о���� �޸� ����
	SafeRelease(g_pColorFrameReader);// color �̹��� �о���� �޸� ����
	SafeRelease(g_pCoordinateMapper);

	if (g_pColorCoordinates)
	{
		delete[] g_pColorCoordinates;
		g_pColorCoordinates = NULL;
	}


	// close the Kinect Sensor.
	if (g_pKinectSensor)
	{
		g_pKinectSensor->Close(); //kinect ���� ���� ��.
	}

	SafeRelease(g_pKinectSensor); //kinet ���� �޸� ����
}
