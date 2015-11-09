
#include <iostream>

// Kinect Library
#include <Kinect.h>
#include <Kinect.Face.h>

//OpenCV Library
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/flann/flann.hpp"

#include <d3d10.h>
#include <D3DX10.h>
#include <DxErr.h>
#include "d3dUtil.h"


#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10.lib")

#include <omp.h>




#define PI 3.14159265

using namespace std;
using namespace cv;
using namespace cv::gpu;

struct Particle
{
	float x;
	float y;
	float Depth;
};

struct FilterGroup
{
	enum Enum
	{
		ePARTICLE = (1 << 0),
	};
};


class KinectHandle
{
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	static const int		particle_flow_height = 360;
	static const int		particle_flow_width = 640;
	static const int		factor = 3;
	static const int		num_particle_pso = 64;
	static const int		num_generation_pso = 30;



	
public: 



	float minConfig[26];
	float maxConfig[26];

	float* handParameter;
	D3DXQUATERNION temp;
	// Kinect
	IKinectSensor*     m_pKinectSensor;
	ICoordinateMapper* m_pCoordinateMapper;
	DepthSpacePoint*   m_pDepthCoordinates;

	Mat	referenceFrame;
	Mat LastGrayFrame;
	Mat Last_u;
	Mat	Last_v;

	std::vector<Particle> proxyParticle;
	bool gotFace;


	// to prevent drawing until we have data for both streams
	bool m_bDepthReceived;
	bool m_bColorReceived;
	bool m_bNearMode;
	bool m_bPaused;

	// Kinect reader
	IColorFrameReader*     m_pColorFrameReader;
	IBodyFrameReader*      m_pBodyFrameReader;
	IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];
	IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];
	IFaceFrameResult*      pFaceFrameResult;
	IMultiSourceFrameReader* m_pMultiSourceFrameReader;


	// For   mapping depth to color
	RGBQUAD* m_pColorRGBX;
	RGBQUAD* m_pOutputRGBX;
	RGBQUAD* m_pBackgroundRGBX;
	BYTE*	 m_pBodyIndex;


	KinectHandle();
	~KinectHandle();
	
	HRESULT Initialize();
	HRESULT CreateFirstConnected();
	HRESULT	InitOpenCV();
	void	InitPhysx();

	HRESULT	ProcessColor(int nBufferSize);
	HRESULT KinectProcess();

	


	void	RenderParticle();
	void	UpdateParticle(Mat& u, Mat& v);
	void    getFlowField(const Mat& u, const Mat& v, Mat& flowField);
	void    ProcessFrame(INT64 nTime,
		UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
		const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
		const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight, IBody** ppBodies, int nBodyCount, Mat&u, Mat&v, USHORT nMinDistance, USHORT nMaxDistance);

	void	CloseKinect();
	void	CloseOpenCV();
	void	ParticleCompute();
	BOOLEAN ProcessFaces(IMultiSourceFrame* pMultiFrame);

	HRESULT UpdateBodyData(IBody** ppBodies, IMultiSourceFrame* pMultiFrame);
	
	void	getFaceResult(float*, float*, float*);


private:
	LONG                                m_depthWidth;
	LONG                                m_depthHeight;

	LONG                                m_colorWidth;
	LONG                                m_colorHeight;

	LONG                                m_colorToDepthDivisor;

	float								face_x;
	float								face_y;
	float								face_z;

	void HandEncoding(float*, D3DXVECTOR4*);

	float compareHand(float*,Mat&, Point,Mat&);


	float* optimized(float inputConfig[26], float suggestConfig[7] ,
		Point HandPosition_depth, CameraSpacePoint HandPosition_camera, Mat depthMap, Mat dist);

};



// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

