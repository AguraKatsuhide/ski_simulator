// SMA.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <Kinect.h>
#include <math.h>
#include <Windows.h>
#include <mmsystem.h>

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease) //kinect2 메모리를 안전하게 해제하는 함수.
{
	if (pInterfaceToRelease != NULL)
	{

		pInterfaceToRelease->Release(); // kinect2 내 interface를 release.
		pInterfaceToRelease = NULL;
	}
}

IKinectSensor*          g_pKinectSensor=0; //kinect2 센서.
IDepthFrameReader*      g_pDepthFrameReader;//깊이 값 읽는 변수 선언.
IInfraredFrameReader*	g_pInfraredFrameReader; // 적외선 값을 읽는 변수 선언.
IColorFrameReader*		g_pColorFrameReader;// color 값을 읽은 변수 선언

ICoordinateMapper*		g_pCoordinateMapper=0; // mapping 하는 함수
ColorSpacePoint*		g_pColorCoordinates=0; // 깊이 좌표.



#define GET2D8U(IMAGE,X,Y) (*( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) )) //unsigned 8진수 이미지 값 추출 메크로.
#define GET2D8U3CH(IMAGE,X,Y) ( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + ( 3 * (X) ) )//unsigned 8진수 3채널 이미지 값 추출 메크로.
#define GET2D8U4CH(IMAGE,X,Y) ( ( (uchar*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + ( 4 * (X) ) )//unsigned 8진수 4채널 이미지 값 추출 메크로.
//use :  GET2D8U3CH(IMAGE,X,Y)[0], GET2D8U3CH(IMAGE,X,Y)[1], GET2D8U3CH(IMAGE,X,Y)[2]
#define GET2D16U(IMAGE,X,Y) (*( ( (ushort*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//unsigned 16진수 이미지 값 추출 메크로.
#define GET2D16S(IMAGE,X,Y) (*( ( (short*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//signed 16진수 이미지 값 추출 메크로.
#define GET2D32F(IMAGE,X,Y) (*( ( (float*)( ( (IMAGE) -> imageData ) + (Y) * ( (IMAGE) -> widthStep ) ) ) + (X) ))//float 32진수 이미지 값 추출 메크로.

#define DEPTH_WIDTH 512// 깊이 이미지 가로 512.
#define DEPTH_HEIGHT 424//깊이 이미지 세로 424.

#define POINT_IN_RECT(P,R) ( ((P).x)>=((R).x) && ((P).x)<=(((R).x)+((R).width)) && ((P).y)>=((R).y) && ((P).y)<=(((R).y)+((R).height)) )
//포인트가 사각형안에 있는지 확인.


void initProgram();//프로그램 초기화.
void readSettingsFromINI(); // ini 파일 셋팅.

void initCamera();// 카메라 초기화.
void getDepthFrame(); // 깊이 프래임 받는 함수.
void getDepthRanged();//깊이 범위 조절하는 함수.
void getBinary(); // 이진화 영상을 받는 함수.
void processFrame();// 반복 실행 시킬 프로세스 함수.
void exitProgram();// 프로그램 종료.
void exitCamera();// 카메라 종료.
void processKeyEvent(int key); // keyboard 입력을 제어하는 함수.
void centerOfMass3d (IplImage* image);// 3차원 중심을 구하는 함수.
void centerOfMass2d(IplImage* image); //2c차원 중심 구하는 함수.
int distpoints(int a, int b, int c, int d);


void InitKinectV2(); // kinect2 카메라 초기화 함수.
void exitKinectV2(); // kinect2 카메라 해제 함수.


//CV_MAT_ELEM_PTR

int g_uFrameRate=30; //frame rate을 30으로 조절.

CvMemStorage* g_memStorage; // 임시 저장 변수 선언.
int g_uRangeMin; // 깊이값 최소 측정 범위.
int g_uRangeMax; // 깊이값 최대 측정 범위.

int g_uMaxDepth=4500; //깊이값 최대 측정 범위 정의.
int g_uMinDepth=500;  //깊이값 최소 측정 범위 정의.

int g_uThresholdMin=0; // 이진화 영상 임계점 최소값 정의.
int g_uThresholdMax=65535;// 이진화 영상 임계점 최대값 정의.

int g_uMinInfra=405; // 적외선 측정 최소값 정의.

int g_3dX=0;// 3차원 중간값 x 좌표.
int g_3dY=0;// 3차원 중간값 y좌표.
int g_3dZ=0;// 3차원 중간값 z좌표.

int g_2dX=0;// 2차원 중간값 x 좌표.
int g_2dY=0;// 2차원 중간값 y좌표.

int g_HX=0; //머리점 x 좌표
int g_HY=0; //머리점 y 좌표

int g_LX=0; //왼발 x 좌표
int g_LY=0; //왼발 y 좌표

int g_RX=0; //오른발 x 좌표
int g_RY=0; //오른발 y 좌표

int g_roiX1=0;// 유효 roi x좌표
int g_roiY1=0;// 유효 roi y좌표

int g_xmove; //89
int g_ymove; //50
int g_xresize;// color calibration vlaue x 119
int g_yresize;//color calibration vlaue y 98

int g_irAX =0;  // 적외선 마커 반사체의 x의 중심값.
int g_irAY =0;  // 적외선 마커 반사체의 y의 중심값.

int g_Under_irAX =0; //왼발 오른발 적외선 마커 반사체의 x의 중심값.
int g_Under_irAY =0; //왼발 오른발 적외선 마커 반사체의 y의 중심값.

//bool g_captureIr = false;

int g_Head3d = 0;//머리와 3d center of mass의 거리 버튼
int g_Leg3d = 0; //다리와 3d center of mass의 거리 버튼
int g_HLdist = 0; //머리와 다리의 길이 초기화.

double g_HLdistJump = 0; //스키점프 머리와 다리의 길이 초기화.

int g_init3dz = 0; //3d center of mass z 기준값 버튼

bool g_initvalue= false; //알파인 스키 stand 좌표 저장 버튼

bool g_initJumpValue= false; // 스키점프 이전 좌표 저장 버튼

bool g_soundClick_S = false; // '서있음' 소리 한 번만 들리게 제어

bool g_soundClick_L = false; // '왼쪽 기울임' 소리 한 번만 들리게 제어

bool g_soundClick_R = false; // '오른쪽 기울임' 소리 한 번만 들리게 제어

bool g_soundClick_F = false; // '상체굽힘' 소리 한 번만 들리게 제어

bool g_soundClick_K = false; // '무릎굽힘' 소리 한 번만 들리게 제어

bool g_soundClick_D = false; // '활강' 소리 한 번만 들리게 제어

bool g_soundClick_J = false; // '점프' 소리 한 번만 들리게 제어

IplImage* g_imgDepth; // 깊이 이미지 정의.
IplImage* g_imgDepthRangedGray;// 0~255 깊이 이미지 정의.
IplImage* g_imgBinary; // 이진화 영상 정의.
IplImage* g_IRimgBinary; // IR 이진화 영상 정의.
IplImage* g_imgInfra;// 적외선 영상 정의.
//IplImage* g_imgInfra8Bit;// 적외선 영상 정의.
IplImage* g_imgColor; // color 영상 정의.
//IplImage* g_imgColorCal; // color calibration결과 영상 정의.
IplImage* g_imgColorPerf;
IplImage* g_imgProcess; // 프로세스 진행중 사용 이미지 정의.
//IplImage* g_imgBMask; // 배경 차영상 마스크 

//IplImage* g_imgIrFirst;// 처음 0.5초 후의 Ir 이미지 
IplImage* g_imgdepthFirst;// 처음 0.5초 후의 깊이 이미지 

IplImage* g_tempdepth;// calibration 비교 이미지.
IplImage* g_colorcalspace;//calibration 결과 이미지.

//IplImage* g_imgInfraDiff;// 최종ir img


CvRect g_rtROI; // 유효 사각 구간 정의.

double g_p1 = 0.03; //왼쪽 끝 실제 변화량
double g_p2 = 0.03; //왼쪽 끝 실제 변화량
double g_Diff_v = 5; //L과 R 사이의 pixel 거리, 알파인 스키와 스키점프 자세 구별 기준.
double g_Dist_HM_a = 5; //Dist_HM의 허용범위
double g_Dist_MC_a = 5; //Dist_MC의 허용범위
double g_Dist_HC_a = 5; //Dist_HC의 허용범위
double g_Mz_a = 5; //M.z의 허용 범위
double g_Hx_a = 12; //H.x의 허용범위
double g_Mx_a = 12; //M.x의 허용범위
double g_Df_v = 0.7; //Dist_HC에 곱하여 활강을 정의하는 기준 변수
double g_Jp_v = 1.1; //Dist_HC에 곱하여 활강을 정의하는 기준 변수

bool g_bRun=true;  // 프로그램 구동 true로 초기화.
char g_strFps[20]="0.00fps";  //fps정의.
char g_strX[20]="X=0"; // 3차원 중심값 X 정의.
char g_strY[20]="Y=0"; // 3차원 중심값 Y 정의.
char g_strZ[20]="Z=0"; // 3차원 중심값 Z 정의.

char g_strX2[20]="X=0"; // 2차원 중심값 X 정의.
char g_strY2[20]="Y=0"; // 2차원 중심값 Y 정의.

char g_strHX[20]="X=0"; // 머리 점 X 정의.
char g_strHY[20]="Y=0"; // 머리 점 Y 정의.

char g_strLX[20]="X=0"; // 왼발 점 X 정의.
char g_strLY[20]="Y=0"; // 왼발 점 Y 정의.

char g_strRX[20]="X=0"; // 오른발 점 X 정의.
char g_strRY[20]="Y=0"; // 오른발 점 Y 정의.

char g_strV[50]=""; // 자세 정의

char g_testHLdistJ[50]=""; // 머리와 다리 거리 표시.
char g_testHLdistJ2[50]=""; // 머리와 다리 거리 표시.


int _tmain(int argc, _TCHAR* argv[]) //메인 함수로 순차적으로 해당 함수를 실행.
{
	initProgram();// 프로그램 초기화.
	while(g_bRun) // 프로그램 반복 실행 부분.
	{
		int t=GetTickCount(); // fps를 구하기 위한 count.

		getDepthFrame(); //깊이값 받아오는 함수.
		getDepthRanged(); // 받아올 깊이 값의 범위 정의 함수.
		//getBinary(); // 받아온 깊이값을 기반으로 이진화 영상 만들기.
		processFrame(); // 실행할 작업 진행 함수.
		processKeyEvent(cvWaitKey(1));// key event를 받기 위한 함수.


		t=GetTickCount()-t;// fps를 구하기 위한 count의 끝.
		sprintf(g_strFps,"%.2ffps",1000.0f/t);// fps 저장.
	}
	exitProgram(); // 프로그램 끝내는 함수.

	return 0;
}
void initProgram()// 프로그램 초기화.
{
	readSettingsFromINI();//ini 셋팅 읽기 .
	initCamera(); //카메라 초기화.

	g_imgDepth=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_16U,1); // 깊이 이미지를 16진수 1채널 이미지로 정의.
	g_imgDepthRangedGray=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 0~255로 변환한 깊이 이미지를 8진수 1채널 이미지로 정의.
	g_imgProcess=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 작업할  이미지를 8진수 1채널 이미지로 정의.
	g_imgBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 이진화 이미지를 8진수 1채널 이미지로 정의.
	g_IRimgBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// ir 이미지 16진수1채널로 정의
	//g_imgInfra8Bit=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 적외선 이미지를 16진수 1채널 이미지로 정의.
	g_imgInfra=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// 적외선 이미지를 16진수 1채널 이미지로 정의.
	g_imgColor=cvCreateImage(cvSize(1920,1080),8,4);// color 이미지 영상.
	g_imgColorPerf=cvCreateImage(cvSize(672,378),8,4);
	//g_imgColorCal= cvCreateImage(cvSize(512,424),8,4);
	//g_imgColorCal= cvCreateImage(cvSize(1920 - g_xresize ,1080 - g_yresize),8,4);// mapping 혹은 calibration 한 영상.
	//g_colorcalspace=cvCreateImage(cvSize(512,424),8,4);// claibration 결과 영상
	g_memStorage=cvCreateMemStorage(); // 임시 저장공간 정의.
	//g_imgBMask=cvLoadImage("8Bit1ChMask.jpg",CV_LOAD_IMAGE_UNCHANGED);// 마스크 받아오는 이미지.
	//g_imgIrFirst=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1);// 적외선 차영상에 쓰는 이미지 
	//g_imgInfraDiff=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1); //적외선 ipl이미지 차영상 결과 이미지
	g_imgdepthFirst=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//깊이 이미지 차영상에 쓰는 이미지
	g_tempdepth=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,4);

	g_pColorCoordinates= new ColorSpacePoint[512 * 424];// color mapping 좌표 값 초기화

	//cvShowImage("8Bit1ChMask",g_imgBMask); // 제작한 마스크를 보여줌.

	cvNamedWindow("control",1);// 컨트롤창 이름 정의.
	cvResizeWindow("control",440,450);// 컨트롤 창 크기 지정.450
	//cvCreateTrackbar("Confidence","control",(int*)&g_uConfidence,9999,setConfidence);//kinect2가 아닌 다른 카메라에서 사용.
	cvCreateTrackbar("RangeMin","control",&g_uRangeMin,g_uMaxDepth,0);// 깊이 이미지의 최소 거리 측정 변환 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("RangeMax","control",&g_uRangeMax,g_uMaxDepth,0);// 깊이 이미지의 최대 거리 측정 변환 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("ThMin","control",&g_uThresholdMin,255,0);// 이진화 이미지의 최소 임계값 변환 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("ThMax","control",&g_uThresholdMax,255,0);// 이진화 이미지의 최대 임계값 변환 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("InfraMin","control",&g_uMinInfra,30000,0);// 적외선 이미지의 최소 거리 측정 변환 트랙바를 컨트롤 창에 추가.

	cvCreateTrackbar("ROI.X","control",&g_rtROI.x,512,0);// 유효 범위 x0 값 정의 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("ROI.Y","control",&g_rtROI.y,424,0);// 유효 범위 y0 값 정의 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("ROI.Width","control",&g_rtROI.width,512,0);// 유효 범위 x1 값 정의 트랙바를 컨트롤 창에 추가.
	cvCreateTrackbar("ROI.Height","control",&g_rtROI.height,424,0);// 유효 범위 y2 값 정의 트랙바를 컨트롤 창에 추가.


}

void readSettingsFromINI()// ini 초기화.
{
	TCHAR path[512];
	GetCurrentDirectory(512,path);  //프로젝트 경로.
	wcscat(path,L"\\camera.ini");// 카메라 관련 ini 설정.
	g_uFrameRate=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("FrameRate"),30,path); //framerate 설정.
	g_uMinInfra=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("MinInfrared "),30,path);//최소 적외선 설정.
	//g_uConfidence=GetPrivateProfileInt(TEXT("CameraSetting"),TEXT("Confidence"),1,path);//kinect1셋팅.

	GetCurrentDirectory(512,path);  //프로젝트 경로.
	wcscat(path,L"\\program.ini");// program.ini를 읽기.
	g_uRangeMin=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("RangeMin"),1,path);//촬영 최소 범위.
	g_uRangeMax=GetPrivateProfileInt(TEXT("DepthRange"),TEXT("RangeMax"),1,path);//촬영 최대 범위.
	if(g_uRangeMin==-1)
		g_uRangeMin=g_uMinDepth;// 잘못정의 된 깊이 거리 범위 값에 대한 예외 처리.
	if(g_uRangeMax==-1)
		g_uRangeMax=g_uMaxDepth;// 잘못정의 된 깊이 거리 범위 값에 대한 예외 처리.

	//측정하고 싶은 구간 지정해서 자르는 설정.
	g_rtROI.x=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIX0"),0,path);// 시작점 x좌표.
	g_rtROI.y=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIY0"),0,path);// 시작점 y좌표.
	g_rtROI.width=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIX1"),0,path)-g_rtROI.x;// ROI 가로 길이.
	g_rtROI.height=GetPrivateProfileInt(TEXT("Process"),TEXT("ROIY1"),0,path)-g_rtROI.y;//ROI 세로 길이.
	//////////////////////////////////////////////////////////////////////////////////////////////////

	g_roiX1=g_rtROI.width+g_rtROI.x;//유효 ROI의 끝점 x
	g_roiY1=g_rtROI.height+g_rtROI.y;// 유효 ROI의 끝점 y

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

void initCamera() //카메라 초기화.
{
	InitKinectV2();//kinect2 초기화.
}

void getDepthFrame() // 깊이 값 및 컬러값 받기(kinect2 라이브러리를 따름.)
{
	int SumX = 0; //적외선 마커 반사체 x 좌표 총합
	int SumY = 0; //적외선 마처 반사체 y 좌표 총합.
	int num  = 0; // 반사체의 총 갯수
	int irAX =0;  // 적외선 마커 반사체의 x의 중심값.
	int irAY =0;  // 적외선 마커 반사체의 y의 중심값.

	int Under_SumX = 0; //왼발 오른발 적외선 마커 반사체 x 좌표 총합
	int Under_SumY = 0; //왼발 오른발적외선 마처 반사체 y 좌표 총합.
	int Under_num  = 0; //왼발 오른발 반사체의 총 갯수
	int Under_irAX =0; //왼발 오른발 적외선 마커 반사체의 x의 중심값.
	int Under_irAY =0; //왼발 오른발 적외선 마커 반사체의 y의 중심값.

	IDepthFrame* pDepthFrame = NULL; // 깊이 프래임 초기화
	UINT nBufferSize = 0; // 버퍼 사이즈 초기화

	//g_pColorCoordinates= new ColorSpacePoint[512 * 424];

	IplImage* imgDepth=cvCreateImageHeader(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_16U,1);// 깊이 이미지의 해더 정의.

	HRESULT hr=g_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//최신 깊이 프래임을 저장하기.
	while(FAILED(hr))// hr에서 데이터를 받는 것이 실패할 경우.
		hr=g_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//최신 깊이 프래임을 저장하기.
	if (SUCCEEDED(hr))//hr에서 데이터를 받는 것을 성공할 경우.
	{
		hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16**)&imgDepth->imageData);//버퍼에서 바로 접근하여 깊이 데이터를 받아옴.
		cvCopy(imgDepth,g_imgDepth);//지역이미지변수를 전역 변수 이미지에 저장.
	}

	cvReleaseImageHeader(&imgDepth); //깊이 이미지 헤더 메모리 해제.
	SafeRelease(pDepthFrame); // 깊이 프래임 안전하게 메모리 해제.

	IInfraredFrame* pInfraredFrame = NULL; //적외선 프래임 초기화.

	IplImage* imgInfra=cvCreateImageHeader(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),16,1); //적외선 ipl이미지해더 생성.
	hr=g_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);//hr에 최신 적외선 프레임 값을 저장.
	while(FAILED(hr))//hr에서 데이터를 받는 것을 실패할 경우
		hr=g_pInfraredFrameReader->AcquireLatestFrame(&pInfraredFrame);//hr에 최신 적외선 프레임 값을 저장.
	if (SUCCEEDED(hr))//hr에서 데이터를 받는 것을 성공할 경우.
	{
		hr = pInfraredFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16**)&imgInfra->imageData); //적외선으로 받은 데이터 g_imgInfra에 저장
		cvCopy(imgInfra,g_imgInfra);// 지역이미지변수를 전역 이미지 변수에 저장.
		//if(GetTickCount()>=500 && g_captureIr==false){ //0.5를 기다리고 초기에 ir이미지를 capture하지 않았으면 ir이미지 capture.
		//	cvCopy(imgInfra,g_imgIrFirst);//local 적외선 이미지를 global 적외선 이미지에 저장.
		//	g_captureIr=true;// 켑쳐가 되었다는 버튼.
		//}
	}

	//cvShowImage("ir First",g_imgIrFirst);

	cvReleaseImageHeader(&imgInfra);//다쓴 ipl 이미지는 메모리 해제
	SafeRelease(pInfraredFrame); // 안전 메모리 해제

	//cvCopy(g_imgInfra,g_imgInfraDiff);// 적외선 이미지를 g_imgInfraDiff에 저장.
	//cvAbsDiff(g_imgInfra,g_imgIrFirst,g_imgInfraDiff);// g_imgInfra - g_imgIrFirst =g_imgInfraDiff 

	//////////////////////////////////////////////////////////////////////////////////////////
	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //탐색공간에서 point 가중치 계산.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{
			if(GET2D16U(g_imgInfra,x,y)>=65535){//탐색공간에 물체가 있는지 없는지 확인.
				SumX = SumX+x;// x좌표를 누적시킴.
				SumY = SumY+y;// y좌표를 누적시킴.
				num= num+1; //누적시킨 갯수 count.
			}
			else{ // 반사체가 없을 때 
				irAX = 0;// 0으로 예외처리 
				irAY = 0;// 0으로 예외처리
				g_HX=0;// 0으로 예외처리
				g_HY=0;// 0으로 예외처리
				g_LX=0;// 0으로 예외처리
				g_LY=0;// 0으로 예외처리
				g_RX=0;// 0으로 예외처리
				g_RY=0;// 0으로 예외처리
				num=num;// count 최기화.
			}

		}
	}
	if(num!=0){ //물체가 있다면.
		irAX = SumX / num;//x좌표 평균값을 구함.
		irAY = SumY / num;//y좌표 평균값을 구함.
		g_irAX = irAX;
		g_irAY = irAY;
	}
	else{//물체가 없을 상황의 예외처리.
		irAX = 0;// 0으로 예외처리
		irAY = 0;// 0으로 예외처리
		g_irAX = 0;// 0으로 예외처리
		g_irAY = 0;// 0으로 예외처리
	}

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) // 머리 마커를 제외한 발 위치 마커 탐색
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)//유효 x 좌표 범위에서 탐색 
		{
			if(GET2D16U(g_imgInfra,x,y)>=65535&& y>irAY){ // 반사체 중심 점 y 값보다 아래에서 탐색.
				Under_SumX = Under_SumX+x; //물체의 x좌표 누적
				Under_SumY = Under_SumY+y;//물체의 y좌표 누적
				Under_num= Under_num+1; // 누적 갯수를 저장
			}
			else{
				Under_irAX = 0; //0으로 초기화
				Under_irAY = 0; //0으로 초기화
				Under_num=Under_num;// 누적 개수 이전 값으로 초기화
			}

		}
	}
	if(Under_num!=0){ //물체가 있다면
		Under_irAX = Under_SumX / Under_num; //누적된 x좌표 나누기 현재 누적된 값.
		Under_irAY = Under_SumY / Under_num; //누적된 y좌표 나누기 현재 누적된 값.

		g_Under_irAX = Under_irAX; // 결과 값을 전역 변수에 저장.
		g_Under_irAY = Under_irAY; // 결과 값을 전역 변수에 저장.
	}
	else{//물체가 없을 상황의 예외처리.
		Under_irAX = 0;// 0으로 예외처리
		Under_irAY = 0;// 0으로 예외처리
		g_Under_irAX = 0;// 0으로 예외처리
		g_Under_irAY = 0;// 0으로 예외처리
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
	for(int y=0;y<DEPTH_HEIGHT;y++) 
	{
		for(int x=0;x<DEPTH_WIDTH;x++)
		{

			unsigned short val=GET2D16U(g_imgInfra,x,y); //ir 이미지에서 적외선의 밝기 값을 val에 저장
			unsigned short val2=GET2D16U(g_imgInfra,x,y); //ir 이미지에서 적외선의 밝기 값을 val에 저장
			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=65535 && y<irAY  ){// 검색 유효 영역에서 val의 값이 가장 높은 밝기 값인지 검사
				g_HX=x;// 머리 x 좌표를 저장
				g_HY=y;// 머리 y 좌표를 저장
			}

			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=65535 && y>g_HY && y>irAY && x< Under_irAX  ){ // 발 중심 x좌표보다 더 작으면 왼발 좌표.
				g_LX=x;// 왼발 x 좌표를 저장
				g_LY=y;// 왼발 y 좌표를 저장
			}

			if(x>=g_rtROI.x && x<=g_roiX1 && y>=g_rtROI.y && y<=g_roiY1 && val>=5535&& y>g_HY && x>g_LX && y>irAY && x>=Under_irAX ){ // 발 중심 x좌표보다 더 크면 오른발 좌표.
				g_RX=x;// 오른발 x 좌표를 저장
				g_RY=y;// 오른발 y 좌표를 저장
			}

			if(val2 <g_uMinInfra){ // g_uMinInfra 보다 어두운 값은 0으로 초기화-> 배경 처리
				GET2D16U(g_imgDepth,x,y)=0;//0으로 초기화-> 배경 처리
			}

		}
	}

	IColorFrame* pColorFrame = NULL; // color 프래임 NULL로 초기화
	nBufferSize = 0; //버프 사이즈 0으로 초기화

	IplImage* imgColor=cvCreateImageHeader(cvSize(1920,1080),8,4); //color 이미지 헤더 생성.

	hr=g_pColorFrameReader->AcquireLatestFrame(&pColorFrame); //카메라에서 color 값을 받아옴.
	while(FAILED(hr)) // hr에 잘못된 값이 들어오면
		hr=g_pColorFrameReader->AcquireLatestFrame(&pColorFrame); //다시 color 값을 갱신함.
	if (SUCCEEDED(hr)) // 성공하면
	{
		// 프래임 데이터를 배열에 변환 하여 저장.
		pColorFrame->CopyConvertedFrameDataToArray(1920*1080*sizeof(RGBQUAD), reinterpret_cast<BYTE*>(g_imgColor->imageData), ColorImageFormat_Bgra);
		/*hr = pColorFrame->AccessRawUnderlyingBuffer(&nBufferSize, (BYTE**)&imgColor->imageData);
		cvCopy(imgColor,g_imgColor);*/
	}

	//cvConvertImage(g_imgColor,g_imgColor_quad,1./5);
	cvResize(g_imgColor,g_imgColorPerf,CV_INTER_CUBIC); // g_imgColor를 g_imgColorPerf로 사이즈 변환.

	//MapColorFrameToDepthSpace 사용.
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


	cvReleaseImageHeader(&imgColor); //color 헤더 메모리 해제.
	SafeRelease(pColorFrame); // pColorFrame 안전 해제.

}


void getDepthRanged() //0~255값으로 깊이 값을 정렬.
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

void getBinary(IplImage* image) // 이진화 영상 생성 함수
{

	cvThreshold(image,g_imgBinary,g_uThresholdMax,255,CV_THRESH_TOZERO_INV);// 이진화 영상 최대 임계점 설정
	cvThreshold(g_imgBinary,g_imgBinary,g_uThresholdMin,255,CV_THRESH_BINARY);// 이진화 영상 최소 임계점 설정

}

void processFrame() //실질적인 트레킹 기능을 행하는 부분
{

	IplImage* imgShowDepthRangedGray=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1); //깊이 값 출력을 위한 이미지.
	IplImage* imgShowProcess=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,3);// 측정 유효한 이미지 출력을 위한 이미지.
	IplImage* imgShowBinary=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,3);// 이진화 이미지 3channel로 출력을 위한 이미지.
	IplImage* img=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//contour을 따기 위한 이미지.
	IplImage* imgMask=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);//contour를 적용한 마스크를 임시저장하는 이미지.
	IplImage* img2=cvCreateImage(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),8,1);// 차영상을 저장하기 위한 임시 이미지.

	double Head3d=0;//현재 머리와 3d center of mass의 거리
	double Leg3d=0; //현재 다리와 3d center of mass의 거리
	double HLdist=0; //현재 머리와 다리 사이의 거리

	double HLdistJump=0;//현재 스키 점프 머리와 다리 사이 거리

	cvCopy(g_imgDepthRangedGray,g_imgProcess);//깊이 이미지를 작업할 imgProcess에 복사.

	///////////////////차영상 배경처리////////////////////////////////////////////
	//cvCopy(g_imgProcess,img2);//임시 이미지 img2에 g_imgProcess의 값을 복사
	//cvZero(g_imgProcess); // g_imgProcess를 0으로 초기화
	//cvCopy(img2,g_imgProcess,g_imgBMask);//img2에서 imgMask에 해당하는 부분을 imgProcess에 저장--//
	//////////////////////////////////////////////////////////////////////////////////

	//cvShowImage("mask test",g_imgProcess); //마스크 해서 배경처리된 결과 보여줌.

	//사용하는 특정 구간을 빼고는 모두 제외.
	//cvCopy(g_imgDepthRangedGray,g_imgProcess);//깊이 이미지를 작업할 imgProcess에 복사.

	for(int x=0;x<DEPTH_WIDTH;x++)	//영상의 가로 세로를 검색.
	{
		for(int y=0;y<DEPTH_HEIGHT;y++)
		{
			if(!POINT_IN_RECT(cvPoint(x,y),g_rtROI)) //program ini에서 지정된 크기만큼 이미지 사각형영역 이외의 부분 검은색으로 설정.
				GET2D8U(g_imgProcess,x,y)=0;     
		}
	}//

	cvCopy(g_imgProcess,img);//-- 가장 큰 깊이 값 물체를 빼고 나머지 모두 제외

	CvSeq* contours=0;	//외곽선.
	cvFindContours(g_imgProcess,g_memStorage,&contours,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);//외곽선 검출.
	CvSeq* select_contour=0;
	float max_area=0;

	while(contours)
	{
		float area=abs(cvContourArea(contours)); //넓이.
		if(area>max_area)		//기존의 최대 이미지 크기보다 검색한 크기가 크면 해당 이미지를 최대 이미지로 저장.
		{
			max_area=area;   //최대 이미지 크기 설정.
			select_contour=contours;
		}

		contours=contours->h_next;
	}

	cvZero(imgMask); //imgMask를 0으로 초기화.
	cvDrawContours(imgMask,select_contour,cvScalar(255),cvScalar(255),0,CV_FILLED);//imgMask에 잘라낸 이미지 저장.

	cvClearMemStorage(g_memStorage); //외곽선 정보 저장한 데이터 해제.(없으면 메모리 누수 생김.)

	cvZero(g_imgProcess);
	cvCopy(img,g_imgProcess,imgMask);//img에서 imgMask에 해당하는 부분을 imgProcess에 저장--//
	cvMerge(g_imgProcess,g_imgProcess,g_imgProcess,NULL,imgShowProcess);//1채널의 imgProcess를 3채널 imgshowprocess에 merge시켜 저장.

	getBinary(g_imgProcess);

	cvMerge(g_imgBinary,g_imgBinary,g_imgBinary,NULL,imgShowBinary);//1채널의 imgProcess를 3채널 imgshowprocess에 merge시켜 저장.

	cvPutText(imgShowBinary,g_strFps,cvPoint(290,50),&cvFont(3,2),cvScalar(255,255,255));//깊이 이미지에 fps 표시.

	//cvDrawCircle(imgShowProcess, cvPoint(g_3dX,g_3dY),4,CV_RGB(255,0,0),2); // 3차원 중심점 좌표를 imgShowProcess에 빨간원으로 표시. 
	cvDrawRect(imgShowProcess,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1); //모션측정 부분을 imgShowProcess에 표시.
	cvDrawRect(g_imgDepthRangedGray,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1);//모션측정 부분 깊이 g_imgDepthRangedGray 이미지에 표시. 
	cvDrawRect(imgShowBinary,cvPoint(g_rtROI.x,g_rtROI.y),cvPoint(g_rtROI.x+g_rtROI.width-1,g_rtROI.y+g_rtROI.height),CV_RGB(255,255,255),1);//모션측정 부분 깊이 imgShowBinary 이미지에 표시.  

	cvDrawRect(g_imgColorPerf,cvPoint(g_rtROI.x+80,g_rtROI.y-30),cvPoint(g_rtROI.x+g_rtROI.width+85,g_rtROI.y+g_rtROI.height-20),CV_RGB(255,0,0),1);//모션측정 부분 깊이 g_imgColorPerf 이미지에 표시. 


	centerOfMass3d(g_imgProcess); //imgProcess의 3차원 중심점 좌표를 생성.
	centerOfMass2d(g_imgBinary); //g_imgBinary의 2차원 중심점 좌표를 생성.

	if(g_3dX!=0 || g_3dY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_3dX,g_3dY),4,CV_RGB(255,0,0),2); // 3차원 중심점 좌표를 imgShowProcess에 빨간원으로 표시.
	}

	if(g_2dX!=0 || g_2dY!=0){
		cvDrawRect(imgShowBinary, cvPoint(g_2dX-5,g_2dY-5),cvPoint(g_2dX+5,g_2dY+5),CV_RGB(255,0,0),2); // 3차원 중심점 좌표를 imgShowProcess에 빨간원으로 표시.
	}


	if(distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_HX!=0 && g_HY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_HX,g_HY),4,CV_RGB(230,231,87),2);// 머리 마커 원으로 표시
	}
	if(distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_LX!=0 && g_LY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_LX,g_LY),4,CV_RGB(255,227,25),2);// 왼발 마커 원으로 표시
	}
	if( g_RX!=0 && g_RY!=0 || distpoints(g_LX,g_LY,g_RX,g_RY)>30 && g_RX!=0 && g_RY!=0){
		cvDrawCircle(imgShowProcess, cvPoint(g_RX,g_RY),4,CV_RGB(111,167,46),2);// 오른발 마커 원으로 표시
	}

	sprintf(g_strX,"3D X=%d",g_3dX);// 3차원 중심값 x좌표 저장.
	sprintf(g_strY,"3D Y=%d",g_3dY);// 3차원 중심값 y좌표 저장.
	sprintf(g_strZ,"3D Z=%d",g_3dZ);// 3차원 중심값 z좌표 저장.

	sprintf(g_strX2,"2D X=%d",g_2dX);// 2차원 중심값 x좌표 저장.
	sprintf(g_strY2,"2D Y=%d",g_2dY);// 2차원 중심값 y좌표 저장.

	sprintf(g_strHX,"Head X=%d",g_HX);// 머리 마커 x좌표 저장.
	sprintf(g_strHY,"Head Y=%d",g_HY);// 머리 마커 y좌표 저장.

	sprintf(g_strLX,"Left foot X=%d",g_LX);// 왼발 마커 x좌표 저장.
	sprintf(g_strLY,"Left foot Y=%d",g_LY);// 왼발 마커 y좌표 저장.

	sprintf(g_strRX,"Right foot X=%d",g_RX);// 오른발 마커 x좌표 저장.
	sprintf(g_strRY,"Right foot Y=%d",g_RY);// 오른발 마커 y좌표 저장.

	///////////////////////////////////////////////자세 정의///////////////////////////////////////////////////////////////////
	if(g_HX!=0 && g_HY!=0 && g_RX!=0 && g_RY!=0 && distpoints(g_Under_irAX,g_Under_irAY,g_RX,g_RY)<=3){ //스키 점프 자세 정의
		// 오른 발과 양 발 중심 좌표랑 비교하여 스키 자세 정의.
		HLdistJump = distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); // 현재 머리와 다리 중점과의 거리.

		//sprintf(g_testHLdistJ,"prev : %0.2f",g_HLdistJump);// 기준 거리.
		//cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));// 활강 자세 표시.

		//sprintf(g_testHLdistJ,"current: %0.2f",HLdistJump);// 현재 거리.
		//cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));// 활강 자세 표시.

		double diffHLJump = g_HLdistJump-HLdistJump;//머리와 다리 거리의 기준 과 현재 값의 차이.

		if(diffHLJump<0){ //머리와 오른다리의 거리의 기준 과 현재 값의 차이 양수화.

			diffHLJump=diffHLJump*(-1);

		}


		if(g_initJumpValue == true && HLdistJump <= (g_HLdistJump)*(0.7) || g_initJumpValue == true && HLdistJump<=100){ //HLdistJump<=120
			//기존 길이 보다 0.7으로 줄어 들거나 변화가 적은 것.

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//기준값 저장.

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

			sprintf(g_strV,"Current Pose : Downhill (1)");// 활강 자세.
			cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 활강 자세 표시.

			//g_initJumpValue = false;

		}

		if(g_initJumpValue == true && HLdistJump >= (g_HLdistJump)*(1.1) || g_initJumpValue == true && HLdistJump>100){  //HLdistJump>120
			//기존 길이 보다 1.1로 줄어 들거나 변화가 적은 것.

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//기준값 저장.

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

			sprintf(g_strV,"Current Pose : Jump (2)");// 스키점프 자세.
			cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));//점프 자세 표시.

			//g_initJumpValue = false;

		}

		if(g_initJumpValue== false && HLdistJump>140 ){ // 스키 점프 초기화

			g_HLdistJump=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);//기준값 저장.

			//sprintf(g_strV,"Current Pose : Downhill Ready ");// 활강 준비.
			//cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 준비 자세 표시.

			g_initJumpValue = true;

		}
	}
	else if(distpoints(g_Under_irAX,g_Under_irAY,g_RX,g_RY)>5 && g_LY!=0 && g_RY!=0 ){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		        
		        double t_HXC;
				double t_3dXC;
		
		if(g_Under_irAX<256){
			

			    g_HX = g_HX+(42*(256-g_Under_irAX)/256); // 화각에 맞춰 x 축 길이 조절.
				g_3dX =g_3dX+(22*(256-g_Under_irAX)/256);
				
			
				
				t_HXC = g_HX;
				t_3dXC = g_3dX;

				sprintf(g_testHLdistJ,"Head.x : %0.2f",t_HXC);// 머리 보정 값.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));

		        sprintf(g_testHLdistJ,"Center.x: %0.2f",t_3dXC);// 3D 보정 값.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));

			}
			if(g_Under_irAX>256){
			
				g_HX = g_HX-(42*(g_Under_irAX-256)/256); // 화각에 맞춰 x 축 길이 조절.
				g_3dX =g_3dX-(22*(g_Under_irAX-256)/256);

				t_HXC = g_HX; // 화각에 맞춰 x 축 길이 조절.
				t_3dXC = g_3dX;

				sprintf(g_testHLdistJ,"Head.x : %0.2f",t_HXC);// 머리 보정 값.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(210,400),&cvFont(1,1),cvScalar(255,255,255));

		        sprintf(g_testHLdistJ,"Center.x: %0.2f",t_3dXC);// 3D 보정 값.
		        cvPutText(imgShowProcess,g_testHLdistJ,cvPoint(355,400),&cvFont(1,1),cvScalar(255,255,255));

			}
		
		if(g_initvalue==true && distpoints(g_irAX,g_irAY,g_HX,g_HY)>10 && g_3dZ!=0)
		{// stand 자세 값 저장 후 다른 자세 구분.

			Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// 현재 머리와 3d 중점 길이 
			Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// 현재 다리와 3d 중점 길이
			HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY);// 현재 머리와 다리 길이
			

			double diffdistH=g_Head3d - Head3d;// stand 와 현재 Head3d 거리 차이 구하기
			diffdistH=fabs(diffdistH);

			double diffdistL=g_Leg3d - Leg3d;// 기준과 와 현재의 Leg3d  거리 차이 구하기
			diffdistL=fabs(diffdistL);

			double diff3dz=g_init3dz - g_3dZ;// stand 와 현재 3dz 거리 차이 구하기
			diff3dz=fabs(diff3dz);

			double diffHead3dx=g_3dX - g_HX;//stand의 머리와 3d center of mass x 좌표 차이. 
			diffHead3dx=fabs(diffHead3dx);

			double diffHLdist=g_HLdist - HLdist;// 머리와 다리 좌표 기준과 현재의 차이. 
			diffHLdist=fabs(diffHLdist);

			double L3d_x = g_Under_irAX-g_3dX; // 좌, 우 기울임과 차별을 두기 위한 길이 변수.
			L3d_x = fabs(L3d_x);

			if(g_Under_irAX<256){

				diffdistH = diffdistH*(1-(g_p1/256)*(256-g_Under_irAX)); //화각에 맞춰 y 축 길이 조절.
				diffdistL = diffdistL*(1-(g_p1/256)*(256-g_Under_irAX)); //화각에 맞춰 y 축 길이 조절.
				diffHLdist = diffHLdist*(1-(g_p1/256)*(256-g_Under_irAX)); //화각에 맞춰 y 축 길이 조절.

				
			}
			if(g_Under_irAX>256){
			
				diffdistH = diffdistH*(1-(g_p2/256)*(g_Under_irAX-256)); //화각에 맞춰 y 축 길이 조절.
				diffdistL = diffdistL*(1-(g_p2/256)*(g_Under_irAX-256)); //화각에 맞춰 y 축 길이 조절.
				diffHLdist = diffHLdist*(1-(g_p2/256)*(g_Under_irAX-256)); //화각에 맞춰 y 축 길이 조절.

			}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
			
			
			if(  g_initvalue == true  && Head3d >= (g_Head3d*1.1)&& Leg3d >= (g_Leg3d*1.1) && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true  && HLdist>100 && Head3d>30 && Leg3d>70 && L3d_x<=12 && g_HX>g_LX &&g_HX<g_RX){

				g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand의 머리와 다리의 사이 거리 초기화
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand의 머리와 3d center of mass 거리 초기화.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand의 다리와 3d center of mass 거리 초기화
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
				sprintf(g_strV,"Current Pose : Stand (1)");// 서있음.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 자세 표시.

			}

			if( g_HLdist > HLdist && Head3d <= (g_Head3d*0.7)  && Leg3d >= (g_Leg3d*1.1)  && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true && HLdist<=100 && Head3d<=30 && Leg3d>70 && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX){

		     	g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand의 머리와 다리의 사이 거리 초기화
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand의 머리와 3d center of mass 거리 초기화.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand의 다리와 3d center of mass 거리 초기화
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
				sprintf(g_strV,"Current Pose : Bending Forward (2)");// 상체숙임.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 자세 표시.

			}

			if( g_HLdist > HLdist && Head3d >= (g_Head3d*1.1)  && Leg3d <= (g_Leg3d*0.7) && L3d_x<=12 && g_HX>g_LX &&g_HX<g_RX
				|| g_initvalue == true && HLdist<=100 && Head3d>30 && Leg3d<=70  && L3d_x<=14 && g_HX>g_LX &&g_HX<g_RX){

				g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand의 머리와 다리의 사이 거리 초기화
				g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand의 머리와 3d center of mass 거리 초기화.
			    g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand의 다리와 3d center of mass 거리 초기화
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
				sprintf(g_strV,"Current Pose : Bending Knees (3)");// 무릎구부림.
				cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 자세 표시.

			}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			if( g_HY!=0 && g_LY!=0 && g_RY!=0 && g_3dX<g_Under_irAX && (g_Under_irAX-g_3dX)>14 && g_HX<g_LX || // 3차원 중심점과 양발 중점 비교.
				g_HY!=0 && g_LY!=0 && g_RY!=0 && g_HX<g_LX && (g_3dX-g_HX)>12){ //머리와 3차원 중심점 비교.

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
					
					sprintf(g_strV,"Current Pose : Tilt Left (1)");// 왼쪽 숙임.
					cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 자세 표시.

			}
			if(g_HY!=0 && g_LY!=0 && g_RY!=0 && g_3dX>g_Under_irAX && (g_3dX-g_Under_irAX)>14 && g_HX>g_RX ||  // 중심점과 양발 중점 비교.
				g_HY!=0 && g_LY!=0 && g_RY!=0 && g_HX>g_RX && (g_HX-g_3dX)>12){ //머리와 3차원 중심점 비교.

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

					
					sprintf(g_strV,"Current Pose : Tilt Right (2)");// 오른쪽 숙임.
					cvPutText(imgShowProcess,g_strV,cvPoint(0,50),&cvFont(1.5,2),cvScalar(255,255,255));// 자세 표시.


			}
		}


		if(g_HX!=0&& g_HY!=0 && distpoints(g_Under_irAX,g_Under_irAY,g_HX,g_HY)>140 && g_3dZ!=0 && g_initvalue==false && g_3dZ<=170 ){
			//Stand 자세 기준 값 저장.
			g_init3dz=g_3dZ; //3d z 값 초기화.
			g_Head3d=distpoints(g_HX,g_HY,g_3dX,g_3dY);// stand의 머리와 3d center of mass 거리 초기화.
			g_Leg3d=distpoints(g_Under_irAX,g_Under_irAY,g_3dX,g_3dY);// stand의 다리와 3d center of mass 거리 초기화
			g_HLdist=distpoints(g_HX,g_HY,g_Under_irAX,g_Under_irAY); //stand의 머리와 다리의 사이 거리 초기화
			g_initvalue= true; //저장 flag

		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cvPutText(imgShowProcess,g_strX,cvPoint(10,370),&cvFont(1,1),cvScalar(255,255,255));// 3차원 중심값 x좌표 표시.
	cvPutText(imgShowProcess,g_strY,cvPoint(10,385),&cvFont(1,1),cvScalar(255,255,255));// 3차원 중심값 y좌표 표시.
	cvPutText(imgShowProcess,g_strZ,cvPoint(10,400),&cvFont(1,1),cvScalar(255,255,255));// 3차원 중심값 z좌표 표시.

	cvPutText(imgShowBinary,g_strX2,cvPoint(10,370),&cvFont(1,1),cvScalar(255,255,255));// 2차원 중심값 x좌표 표시.
	cvPutText(imgShowBinary,g_strY2,cvPoint(10,385),&cvFont(1,1),cvScalar(255,255,255));// 2차원 중심값 y좌표 표시.

	cvPutText(imgShowProcess,g_strHX,cvPoint(100,370),&cvFont(1,1),cvScalar(255,255,255));//머리 마커 x좌표 표시
	cvPutText(imgShowProcess,g_strHY,cvPoint(100,385),&cvFont(1,1),cvScalar(255,255,255));//머리 마커 y좌표 표시

	cvPutText(imgShowProcess,g_strLX,cvPoint(210,370),&cvFont(1,1),cvScalar(255,255,255));//왼발 마커 x좌표 표시
	cvPutText(imgShowProcess,g_strLY,cvPoint(210,385),&cvFont(1,1),cvScalar(255,255,255));//왼발 마커 y좌표 표시

	cvPutText(imgShowProcess,g_strRX,cvPoint(355,370),&cvFont(1,1),cvScalar(255,255,255));//오른발 마커 x좌표 표시
	cvPutText(imgShowProcess,g_strRY,cvPoint(355,385),&cvFont(1,1),cvScalar(255,255,255));//오른발 마커 y좌표 표시




	cvShowImage("depth",g_imgDepthRangedGray);//깊이 영상 출력 라벨링.
	cvShowImage("binary",imgShowBinary); // 이진화 영상 출력.

	//cvShowImage("IR binary",g_IRimgBinary);//IR 이미지 이진화영상

	cvShowImage("IR",g_imgInfra); //적외선 이미지 출력.
	cvShowImage("자세 측정 화면",imgShowProcess); //측정 결과 보여줌.

	//cvShowImage("Color_calibration mapping",g_imgColorCal);// color 영상
	cvShowImage("Color",g_imgColorPerf);// color 영상

	cvReleaseImage(&imgShowDepthRangedGray);//이미지 파일 메모리 해제 .
	cvReleaseImage(&imgShowBinary);//이미지 파일 메모리 해제.
	cvReleaseImage(&imgShowProcess);//이미지 파일 메모리 해제.
	cvReleaseImage(&img);//이미지 파일 메모리 해제.
	cvReleaseImage(&imgMask);//이미지 파일 메모리 해제.
	cvReleaseImage(&img2);//이미지 파일 메모리 해제.


}
void centerOfMass3d (IplImage* image) // 3차원 중심점을 구하는 함수.
{
	int SumX = 0;// 0으로 초기화
	int SumY = 0;// 0으로 초기화
	int SumZ = 0;// 0으로 초기화
	int num  = 0; // 0으로 초기화

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //탐색공간에서 point 가중치 계산.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{

			if(GET2D8U(image,x,y)!=0){//탐색공간에 물체가 있는지 없는지 확인.
				SumX = SumX+x;
				SumY = SumY+y;
				SumZ = SumZ + GET2D8U(image,x,y);
				num= num+1;
			}
			else{
				g_3dX = 0;//0으로 초기화
				g_3dY = 0;//0으로 초기화
				g_3dZ = 0;//0으로 초기화
				num=num;
			}

		}
	}
	if(num!=0){ //물체가 있으면 3차원 중심값을 구하고
		g_3dX = SumX / num;
		g_3dY = SumY / num;
		g_3dZ = SumZ / num;
	}
	else{// 없으면 0으로 초기화
		g_3dX = 0;
		g_3dY = 0;
		g_3dZ = 0;
	}

	// The coordinate (SumX,SumY) is the center of the image mass.
}

void centerOfMass2d (IplImage* image) // 2차원 중심점을 구하는 함수.
{
	int SumX = 0;
	int SumY = 0;
	int num  = 0; 

	for (int y=g_rtROI.y; y<(g_rtROI.y+g_rtROI.height); y++) //탐색공간에서 point 가중치 계산.
	{
		for (int x=g_rtROI.x; x<g_rtROI.width; x++)
		{

			if(GET2D8U(image,x,y)!=0){//탐색공간에 물체가 있는지 없는지 확인.
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
	if(num!=0){ //물체가 없을 상황의 예외처리.
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
void exitProgram()// 프로그램 종료시 메모리 해제.
{
	exitCamera(); // 카메라 메모리 끄기 함수 호출.
	cvDestroyAllWindows(); // 현재 켜진 윈도우를 모두 끄기.
	cvReleaseImage(&g_imgDepth);// 깊이 이미지 이미지 메모리 해제.
	cvReleaseImage(&g_imgDepthRangedGray);//g_imgDepthRangedGray 메모리 해제
	cvReleaseImage(&g_imgBinary);//g_imgBinary 이진화된 메모리 해제

	cvReleaseImage(&g_IRimgBinary);//g_IRimgBinary 이진화된 메모리 해제
	//cvReleaseImage(&g_imgInfra8Bit);//g_IRimgBinary 이진화된 메모리 해제

	cvReleaseImage(&g_imgInfra);// g_imgInfra 메모리 해제
	cvReleaseImage(&g_imgProcess);// g_imgProcess 메모리 해제
	cvReleaseMemStorage(&g_memStorage);// g_memStorage 메모리 해제
	//cvReleaseImage(&g_imgBMask);// g_imgBMask 메모리 해제
	//cvReleaseImage(&g_imgIrFirst);// g_imgIrFirst 메모리 해제
	//cvReleaseImage(&g_imgInfraDiff);// g_imgInfraDiff 메모리 해제

	cvReleaseImage(&g_tempdepth);// calibration 비교 이미지 메모리 해제

	cvReleaseImage(&g_imgColor);// g_imgColor 메모리 해제
	//cvReleaseImage(&g_imgColorCal);// g_imgColorCal 메모리 해제
	cvReleaseImage(&g_imgColorPerf);// g_imgColorPerf 메모리 해제
	//cvReleaseImage(&g_colorcalspace);

	cvReleaseImage(&g_imgdepthFirst);// g_imgdepthFirst 메모리 해제
}

void exitCamera() //카메라 끄기 함수.
{
	exitKinectV2(); // 키넥트2 끄기.
}

void processKeyEvent(int key) //키보드 이벤트 설정.
{
	switch(key)
	{
	case VK_ESCAPE: // esc 키를 받았을 때 프로그램 종료.
		g_bRun=false;
	default:
		break;
	}
}


void InitKinectV2() //kinect2 초기화.
{
	HRESULT hr;// 결과 hr 선언

	hr = GetDefaultKinectSensor(&g_pKinectSensor); // 키넥트 센서의 기본값으로 초기화한다.


	if (g_pKinectSensor) // kinect 센서가 ready 상태인지 확인.
	{
		// Initialize the Kinect and get the depth reader.
		IDepthFrameSource* pDepthFrameSource = NULL;  // 깊이 프래임을 NULL로 초기화.
		IInfraredFrameSource* pInfraredFrameSource = NULL; // 적외선 프래임을 NULL로 초기화.
		IColorFrameSource* pColorFrameSource = NULL; // Color 프래임 소스 NULL로 초기화.

		hr = g_pKinectSensor->Open(); // kinect를 작동시킴.

		if (SUCCEEDED(hr)) // kinect가 작동이 된지 확인.
		{
			hr = g_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource); //깊이프레임소스를 hr에 저장.
			hr = g_pKinectSensor->get_InfraredFrameSource(&pInfraredFrameSource);// 적외선 소스를 hr에 저장.
			hr = g_pKinectSensor->get_ColorFrameSource(&pColorFrameSource); // color 소스를 hr에 저장.
			hr = g_pKinectSensor->get_CoordinateMapper(&g_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))// kinect가 작동이 된지 확인
		{
			hr = pDepthFrameSource->OpenReader(&g_pDepthFrameReader); //깊이 프레임소스의 OpenReader을 hr에 저장.
			hr = pInfraredFrameSource->OpenReader(&g_pInfraredFrameReader);//적외선 프레임소스의 OpenReader을 hr에 저장.
			hr = pColorFrameSource->OpenReader(&g_pColorFrameReader);// color 프레임에서 OpenReader을 hr에 저장.
		}

		SafeRelease(pDepthFrameSource); //깊이프레임소스를 안전하게 메모리 해제.
	}

	if (!g_pKinectSensor || FAILED(hr)) //kinect 센서가 ready 상태가 아님.
	{
		printf("No ready Kinect found!");
	}
}

void exitKinectV2() //kinect2 카메라 해제.
{
	// done with depth frame reader.
	SafeRelease(g_pDepthFrameReader);//깊이프레임을 읽어오는 메모리 해제
	SafeRelease(g_pInfraredFrameReader);//적외선프레임을 읽어오는 메모리 해제
	SafeRelease(g_pColorFrameReader);// color 이미지 읽어오는 메모리 해제
	SafeRelease(g_pCoordinateMapper);

	if (g_pColorCoordinates)
	{
		delete[] g_pColorCoordinates;
		g_pColorCoordinates = NULL;
	}


	// close the Kinect Sensor.
	if (g_pKinectSensor)
	{
		g_pKinectSensor->Close(); //kinect 센서 전원 끔.
	}

	SafeRelease(g_pKinectSensor); //kinet 센서 메모리 해제
}
