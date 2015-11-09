#include "stdafx.h"
#include "kinect_handle.h"


static const float c_FaceTextLayoutOffsetX = -0.1f;

// face property text layout offset in Y axis
static const float c_FaceTextLayoutOffsetY = -0.125f;

static const float particleSize = 0.2f;
static const float particleRadius = 0.01f;

static const float height = 1.20;
static const float degree = 65.0f;
static const float interact_limit_y = 5.0f; // in 10 CM
static const float interact_limit_y2 = 0.1f; // in 10 CM
static const float interact_limit_Z = -5.0f;



// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;

KinectHandle::KinectHandle()
{
	m_depthWidth = static_cast<LONG>(cDepthWidth);
	m_depthHeight = static_cast<LONG>(cDepthHeight);

	m_colorWidth = static_cast<LONG>(cColorWidth);
	m_colorHeight = static_cast<LONG>(cColorHeight);

	m_colorToDepthDivisor = m_colorWidth / m_depthWidth;

	m_pCoordinateMapper = NULL;
	m_pKinectSensor     = NULL;
	m_pColorFrameReader = NULL;
	m_pBodyFrameReader  = NULL;
	m_pDepthCoordinates = NULL;

	m_pOutputRGBX     = new RGBQUAD[cColorWidth * cColorHeight];
	m_pColorRGBX      = new RGBQUAD[cColorWidth * cColorHeight];
	m_pBackgroundRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	gotFace = false;
	

	// create heap storage for the coorinate mapping from color to depth
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];

	for (int i = 0; i < BODY_COUNT; i++)
	{
		m_pFaceFrameSources[i] = nullptr;
		m_pFaceFrameReaders[i] = nullptr;
	}

	face_x = 0.0f;
	face_y = 2.5f;
	face_z = -8.0f;

	referenceFrame = Mat::zeros(1080, 1920, CV_8UC1);

	proxyParticle.clear();


	//InitialIze max finger Config


	//maxConfig[3] = PI/2 ;
	//minConfig[3] = -PI/2;
	//maxConfig[4] = PI/2;
	//minConfig[4] = -PI/2.0f;
	//maxConfig[5] = PI/2.0f;
	//minConfig[5] = -PI/2.0f;

	maxConfig[7] = D3DX_PI / 2.0f;
	minConfig[7] = 0;//-D3DX_PI/2.0f;
	maxConfig[8] = 0.5236f;
	minConfig[8] = -0.5236f;
	maxConfig[9] = D3DX_PI / 2.0f;
	minConfig[9] = 0;//-D3DX_PI / 2.0f;
	maxConfig[10] = D3DX_PI / 2.0f;
	minConfig[10] = 0;//-D3DX_PI / 2.0f;

	maxConfig[11] = D3DX_PI / 2.0f;
	minConfig[11] = 0;//-D3DX_PI / 2.0f;

	maxConfig[12] = 0.5236f;
	minConfig[12] =-0.5236f;

	maxConfig[13] = D3DX_PI / 2.0f;
	minConfig[13] = 0;//-D3DX_PI / 2.0f;

	maxConfig[14] = D3DX_PI / 2.0f;
	minConfig[14] = 0;//-D3DX_PI / 2.0f;

	maxConfig[15] = D3DX_PI / 2.0f;
	minConfig[15] = 0;//-D3DX_PI / 2.0f;
	
	maxConfig[16] = 0.174f;
	minConfig[16] = -0.174f;

	maxConfig[17] = D3DX_PI / 2.0f;
	minConfig[17] = 0;//-D3DX_PI / 2.0f;

	maxConfig[18] = D3DX_PI / 2.0f;
	minConfig[18] = 0;//-D3DX_PI / 2.0f;

	maxConfig[19] = D3DX_PI / 2.0f;
	minConfig[19] = 0;//-D3DX_PI / 2.0f;

	maxConfig[20] = D3DX_PI / 2.0f;
	minConfig[20] = 0;//-D3DX_PI / 2.0f;

	maxConfig[21] = D3DX_PI / 2.0f;
	minConfig[21] = 0;//-D3DX_PI / 2.0f;

	maxConfig[22] = D3DX_PI / 2.0f;
	minConfig[22] = 0;//-D3DX_PI / 2.0f;

	maxConfig[23] = 0.174f;
	minConfig[23] = -0.7853f;
	minConfig[24] = 0;// -D3DX_PI / 2.0f;
	maxConfig[24] = D3DX_PI / 2.0f;

	minConfig[25] = 0;//-D3DX_PI / 2.0f;
	maxConfig[25] = D3DX_PI / 2.0f;

	minConfig[26] = 0;//-D3DX_PI / 2.0f;
	maxConfig[26] = D3DX_PI / 2.0f;

	handParameter = new float[27];
	for (int i = 0; i < 27; i++)
	{
		handParameter[i] = 0;
	}


	D3DXQuaternionRotationYawPitchRoll(&temp, 0.0f, D3DX_PI / 2.0f, 0.0f);
	handParameter[3] = temp.x;
	handParameter[4] = temp.y;
	handParameter[5] = temp.z;
	handParameter[6] = temp.w;


};


HRESULT KinectHandle::InitOpenCV()
{
	DWORD colorWidth, colorHeight;
	colorHeight = cColorHeight;
	colorWidth = cColorWidth;

	//Size size(colorWidth, colorHeight);
	//namedWindow("flowfield");
	//////namedWindow("reference");

	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());
	return S_OK;
}

HRESULT KinectHandle::Initialize()
{
	HRESULT hr;
	hr = CreateFirstConnected();
	if (SUCCEEDED(hr))
	{
		hr = InitOpenCV();
	}
	return hr;
}
/// <summary>
/// Create the first connected Kinect found 
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT KinectHandle::CreateFirstConnected()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth |
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_BodyIndex |
				FrameSourceTypes::FrameSourceTypes_Body,
				&m_pMultiSourceFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	return hr;


}


HRESULT KinectHandle::KinectProcess()
{
	if (!m_pMultiSourceFrameReader)
	{
		return E_FAIL;
	}

	IMultiSourceFrame* pMultiSourceFrame       = NULL;
	IDepthFrame*       pDepthFrame             = NULL;
	IColorFrame*       pColorFrame             = NULL;
	IBodyIndexFrame*   pBodyIndexFrame         = NULL;
	IBodyFrame*		   pBodyFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	// Get Depth
	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	// Get Color
	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}

	//Get Body Index
	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}

		SafeRelease(pBodyIndexFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;
		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}
		SafeRelease(pBodyFrameReference);
	}


	// If all succeeded combine frame
	if (SUCCEEDED(hr))
	{
		INT64                     nDepthTime             = 0;
		IFrameDescription*        pDepthFrameDescription = NULL;
		int                       nDepthWidth            = 0;
		int                       nDepthHeight           = 0;
		USHORT	                  nMaxDistance           = 0;
		USHORT                    nMinDistance           = 0;
		UINT                      nDepthBufferSize       = 0;
		UINT16                    *pDepthBuffer          = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int                nColorWidth            = 0;
		int                nColorHeight           = 0;
		ColorImageFormat   imageFormat            = ColorImageFormat_None;
		UINT               nColorBufferSize       = 0;
		RGBQUAD            *pColorBuffer          = NULL;

		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int                nBodyIndexWidth            = 0;
		int                nBodyIndexHeight           = 0;
		UINT               nBodyIndexBufferSize       = 0;
		BYTE               *pBodyIndexBuffer          = NULL;

		INT64				nBodyTime = 0;
		IBody* ppBodies[BODY_COUNT] = { 0 };


		pDepthFrame->get_RelativeTime(&nDepthTime);
		pDepthFrame->get_DepthMaxReliableDistance(&nMaxDistance);
		pDepthFrame->get_DepthMinReliableDistance(&nMinDistance);
		hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		
		if (SUCCEEDED(hr))
		{
			pDepthFrameDescription->get_Width(&nDepthWidth);
			pDepthFrameDescription->get_Height(&nDepthHeight);
			
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}

		// get color frame data
		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			pColorFrameDescription->get_Width(&nColorWidth);
			pColorFrameDescription->get_Height(&nColorHeight);
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}


		//Copy color data to the heap
		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}


		if (SUCCEEDED(hr))
		{
			gotFace = FALSE;// ProcessFaces(pMultiSourceFrame);
			ProcessFrame(nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
				pColorBuffer, nColorWidth, nColorHeight,
				pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight, ppBodies, BODY_COUNT, Last_u, Last_v, nMinDistance, nMaxDistance);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pBodyIndexFrameDescription);
	}

	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pBodyFrame);
	SafeRelease(pMultiSourceFrame);
	

	return hr;

}

void KinectHandle::ProcessFrame(INT64 nTime,
	UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight, IBody** ppBodies, int nBodyCount, Mat&u, Mat&v, USHORT nMinDistance, USHORT nMaxDistance)
{
	DepthSpacePoint rh_d;
	DepthSpacePoint lh_d;
	BOOL gotHandLeft = false;
	BOOL gotHandRight = false;
	int boxSize = 100;
	int halfBoxSize = boxSize / 2.0f;
	HRESULT hr;
	CameraSpacePoint rh;
	CameraSpacePoint lh;
	
	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				i = nBodyCount;
				Joint joints[JointType_Count];
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					rh = joints[JointType::JointType_HandRight].Position;
					lh = joints[JointType::JointType_HandLeft].Position;

					m_pCoordinateMapper->MapCameraPointToDepthSpace(rh, &rh_d);
					m_pCoordinateMapper->MapCameraPointToDepthSpace(lh, &lh_d);
					
					if (rh_d.X > halfBoxSize+1 && rh_d.X < nDepthWidth-halfBoxSize && rh_d.Y > halfBoxSize+1 && rh_d.Y < nDepthHeight-halfBoxSize)
						gotHandLeft = true;
					if (lh_d.X > halfBoxSize+1 && lh_d.X < nDepthWidth-halfBoxSize && lh_d.Y > halfBoxSize+1 && lh_d.Y < nDepthHeight - halfBoxSize)
						gotHandRight = true;
					//gotFace = TRUE;

				}
			}
		}
	}

	Mat _depthMap(nDepthHeight, nDepthWidth, CV_16UC1, reinterpret_cast<void*>(pDepthBuffer));
	Mat depthMap = _depthMap.clone();
	double scale = 1.0 / (nMaxDistance - nMinDistance);
	Mat img0(nDepthHeight, nDepthWidth, CV_32FC1);
	depthMap.convertTo(img0, CV_32FC1, scale);

	
		CameraSpacePoint* points = new CameraSpacePoint[nDepthHeight*nDepthWidth];
		m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthHeight*nDepthWidth, pDepthBuffer, nDepthHeight*nDepthWidth, points);

		if (gotHandRight)
		{
			circle(img0, cv::Point(rh_d.X, rh_d.Y), 1, Scalar(255));
		}
	
		if (gotHandLeft)
			circle(img0, cv::Point(lh_d.X, lh_d.Y), 1, Scalar(255));

		for (int j = 0; j < nDepthHeight; j++)
		{
			ushort* p = depthMap.ptr<ushort>(j);
			float* q = img0.ptr<float>(j);
			for (int i = 0; i < nDepthWidth; i++)
			{
				float y = cos(degree * PI / 180.0f)*points[j*nDepthWidth + i].Y - sin(degree * PI / 180.0f)*points[j*nDepthWidth + i].Z + height;
				float z = -sin(degree * PI / 180.0f)*points[j*nDepthWidth + i].Y - cos(degree * PI / 180.0f)*points[j*nDepthWidth + i].Z;

				if (y < 0.05 || z < -0.5)
				{
					p[i] = 0;
					q[i] = 0;
				}
			}
		}

		float* suggestPosition = new float[7];

		suggestPosition[0] = rh.X - 0.02;
		suggestPosition[1] = cos(degree * PI / 180.0f)*rh.Y - sin(degree * PI / 180.0f)*rh.Z + height - 0.02;
		suggestPosition[2] = -sin(degree * PI / 180.0f)*rh.Y - cos(degree * PI / 180.0f)*rh.Z+0.02;
		suggestPosition[3] = temp.x;
		suggestPosition[4] = temp.y;
		suggestPosition[5] = temp.z;
		suggestPosition[6] = temp.w;

		//handParameter[0] = rh.X - 0.02;
		//handParameter[1] = cos(degree * PI / 180.0f)*rh.Y - sin(degree * PI / 180.0f)*rh.Z + height - 0.02;
		//handParameter[2] = -sin(degree * PI / 180.0f)*rh.Y - cos(degree * PI / 180.0f)*rh.Z + 0.02;
		//handParameter[3] = temp.x;
		//handParameter[4] = temp.y;
		//handParameter[5] = temp.z;
		//handParameter[6] = temp.w;



		std::free(points);
		Mat img1,segmented;
		img1.create(img0.rows, img0.cols, CV_32FC1);
		segmented.create(img0.rows, img0.cols, CV_8UC1);
		cv::bilateralFilter(img0, img1, 3, 50, 50);
		img1.convertTo(segmented, CV_8UC1, 255);

		Mat bw;
		cv::threshold(segmented, bw, 100, 255, CV_THRESH_BINARY_INV | CV_THRESH_OTSU );
		Mat dist;
		cv::distanceTransform(bw, dist, CV_DIST_L2, 3);

		float result = 99999999;
		if (gotHandRight)
		{
			DepthSpacePoint* depthPoints = new DepthSpacePoint[48];
			D3DXVECTOR4* output = new D3DXVECTOR4[48];
			CameraSpacePoint* spacePoint = new CameraSpacePoint[48];
			handParameter = optimized(handParameter, suggestPosition, cv::Point(rh_d.X, rh_d.Y), rh, depthMap, dist);
			//result = compareHand(handParameter, depthMap, cv::Point(rh_d.X, rh_d.Y), dist);
			
			HandEncoding(handParameter, output);
			for (int i = 0; i < 48; i++)
			{
				spacePoint[i].X = output[i].x;
				spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
				spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
				m_pCoordinateMapper->MapCameraPointToDepthSpace(spacePoint[i], &depthPoints[i]);
				circle(segmented, cv::Point(depthPoints[i].X, depthPoints[i].Y), 1, cv::Scalar(255), -1);
			}
		}
		D3DXQUATERNION test_quaternion = D3DXQUATERNION(handParameter[3], handParameter[4], handParameter[5], handParameter[6]);
		D3DXVECTOR3 test_axis;
		float angle;

		D3DXQuaternionNormalize(&test_quaternion, &test_quaternion);
		D3DXQuaternionToAxisAngle(&test_quaternion, &test_axis, &angle);
		
		string text = std::to_string(result);
		text.append("/");
		text.append(std::to_string(test_axis.x));
		text.append("/");
		text.append(std::to_string(test_axis.y));
		text.append("/");
		text.append(std::to_string(test_axis.z));
		text.append("/");
		text.append(std::to_string(angle*180/D3DX_PI));
		int fontFace = CV_FONT_HERSHEY_PLAIN;
		double fontScale = 1;
		int thickness = 1;
		cv::Point textOrg(10, 130);
		cv::putText(segmented, text, textOrg, fontFace, fontScale, Scalar::all(255), thickness, 8);
		
	//if (gotHandLeft)
	//{
	//	floodFill(segmented, cv::Point(lh_d.X,lh_d.Y), Scalar(255), 0, Scalar(1), Scalar(1)  );
	//}
	//if (gotHandRight)
	//{
	//	floodFill(segmented, cv::Point(rh_d.X, rh_d.Y), Scalar(128), 0, Scalar(1), Scalar(1));
	//}
	cv::imshow("Depth Only", segmented);
}


void KinectHandle::getFlowField(const Mat& u, const Mat& v, Mat& flowField)
{
	float maxDisplacement = -1.0f;

	for (int i = 0; i < u.rows; i += 10)
	{
		const float* ptr_u = u.ptr<float>(i);
		const float* ptr_v = v.ptr<float>(i);

		for (int j = 0; j < u.cols; j += 10)
		{
			float d = sqrt(ptr_u[j] * ptr_u[j] + ptr_v[j] * ptr_v[j]); //max(fabsf(ptr_u[j]), fabsf(ptr_v[j]));

			if (d > maxDisplacement)
				maxDisplacement = d;
		}
	}

	//flowField = Mat::zeros(u.size(), CV_8UC3);


	for (int i = 0; i < flowField.rows; i += 10)
	{
		const float* ptr_u = u.ptr<float>(i);
		const float* ptr_v = v.ptr<float>(i);

		//Vec4b* row = flowField.ptr<Vec4b>(i);

		for (int j = 0; j < flowField.cols; j += 10)
		{
			//row[j][0] = 0;
			//row[j][1] = static_cast<unsigned char> (mapValue(-ptr_v[j], -maxDisplacement, maxDisplacement, 0.0f, 255.0f));
			//row[j][2] = static_cast<unsigned char> (mapValue(ptr_u[j], -maxDisplacement, maxDisplacement, 0.0f, 255.0f));
			//row[j][3] = 255;


			Point p = Point(j, i);
			float l = sqrt(ptr_u[j] * ptr_u[j] + ptr_v[j] * ptr_v[j]);
			float l_max = maxDisplacement;

			float dx = ptr_u[j];
			float dy = ptr_v[j];
			if (l > 0 && flowField.at<Vec4b>(i, j) != Vec4b(255, 0, 0))
			{
				double spinSize = 5.0 * l / l_max;  // Factor to normalise the size of the spin depeding on the length of the arrow

				Point p2 = Point(p.x + (int)(dx), p.y + (int)(dy));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

				double angle;               // Draws the spin of the arrow
				angle = atan2((double)p.y - p2.y, (double)p.x - p2.x);

				p.x = (int)(p2.x + spinSize * cos(angle + 3.1416 / 4));
				p.y = (int)(p2.y + spinSize * sin(angle + 3.1416 / 4));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

				p.x = (int)(p2.x + spinSize * cos(angle - 3.1416 / 4));
				p.y = (int)(p2.y + spinSize * sin(angle - 3.1416 / 4));
				line(flowField, p, p2, CV_RGB(0, 255, 0), 1);

			}
		}
	}

}

KinectHandle::~KinectHandle()
{
	CloseKinect();
	CloseOpenCV();
};

void KinectHandle::CloseKinect()
{
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();

	}
	SafeRelease(m_pKinectSensor);

	// done with face sources and readers
	for (int i = 0; i < BODY_COUNT; i++)
	{
		SafeRelease(m_pFaceFrameSources[i]);
		SafeRelease(m_pFaceFrameReaders[i]);
	}

	// done with body frame reader
	SafeRelease(m_pBodyFrameReader);

	// done with color frame reader
	SafeRelease(m_pColorFrameReader);

	//SAFE_RELEASE(m_pFaceTracker);
	//SAFE_RELEASE(m_pFTResult);

	if (m_pOutputRGBX)
	{
		delete[] m_pOutputRGBX;
		m_pOutputRGBX = NULL;
	}

	if (m_pBackgroundRGBX)
	{
		delete[] m_pBackgroundRGBX;
		m_pBackgroundRGBX = NULL;
	}

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pDepthCoordinates)
	{
		delete[] m_pDepthCoordinates;
		m_pDepthCoordinates = NULL;
	}

}

void KinectHandle::CloseOpenCV()
{
	destroyAllWindows();
}

BOOLEAN KinectHandle::ProcessFaces(IMultiSourceFrame* pMultiFrame)
{
	HRESULT hr;
	IBody* ppBodies[BODY_COUNT] = { 0 };
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies, pMultiFrame));
	bool bOneFaceTraked = false;
	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
			if (bFaceTracked)
			{
				bOneFaceTraked = true;
			}

		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);
	}

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}
	return bOneFaceTraked;
}

HRESULT KinectHandle::UpdateBodyData(IBody** ppBodies, IMultiSourceFrame* pMultiFrame)
{
	HRESULT hr = E_FAIL;

	if (pMultiFrame != nullptr)
	{
		IBodyFrame* pBodyFrame = nullptr;
		IBodyFrameReference* pBodyFrameReference = nullptr;
		hr = pMultiFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
			}
		}

		SafeRelease(pBodyFrame);
	}

	return hr;
}

void KinectHandle::getFaceResult(float* x, float* y , float* z)
{
	*x = face_x ;
	*y = face_y -0.5f;
	*z = face_z +0.5f;
}

float KinectHandle::compareHand(float* HandParameter, Mat& depthMap, Point Handcenter, Mat& dist)
{
	DepthSpacePoint* depthPoints = new DepthSpacePoint[48];
	float handCenter_depth = (float)depthMap.at<ushort>(Handcenter.y, Handcenter.x);
	if (handCenter_depth == 0)
	{
		return 999999999;
	}

	float e_d = 0;
	float e_b = 0;
	float e_b_plus = 0;
	float e_b_minus = 0;
	float e_l = 0;
	float result = 0;
	float convert_pixel_to_m = handCenter_depth*tan(35.3*PI / 180.0f) / 256000.0f;
	
	int parameter = (int)((float)rand() / float(RAND_MAX) * 26.0f);
	int stepSize = 0.1;
	//Gradient Descent

		D3DXVECTOR4* output = new D3DXVECTOR4[48];
		HandEncoding(HandParameter, output);
		float initialHandParameter = HandParameter[parameter];
		// Original Hand Parameter
		CameraSpacePoint* spacePoint = new CameraSpacePoint[48]; // depthSpacePoint
		for (int i = 0; i < 48; i++)
		{
			spacePoint[i].X = output[i].x;
			spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
			spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
		}
		m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
		for (int i = 0; i < 48; i++)
		{
			depthPoints[i].X = 125;
			depthPoints[i].Y = 128;
			if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
			{
				e_b += pow(200 * convert_pixel_to_m, 2.0f);
			}
			else
			{
				float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
				if (depth != 0)
				{
					depth = depth / 1000; // convert to meter
					e_b += pow(max(0.0f, spacePoint[i].Z - depth), 2.0f);
				}
				else
				{
					float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
					e_b += pow(distant * convert_pixel_to_m, 2.0f);
				}
			}
		}
		// + Hand Parameter
		HandParameter[parameter] += stepSize;
		HandEncoding(HandParameter, output);
		for (int i = 0; i < 48; i++)
		{
			spacePoint[i].X = output[i].x;
			spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
			spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
		}
		m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
		for (int i = 0; i < 48; i++)
		{
			if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
			{
				e_b_plus += pow(200 * convert_pixel_to_m, 2.0f);
			}
			else
			{
				float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
				if (depth != 0)
				{
					depth = depth / 1000; // convert to meter
					e_b_plus += pow(max(0.0f, spacePoint[i].Z - depth), 2.0f);
				}
				else
				{
					float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
					e_b_plus += pow(distant * convert_pixel_to_m, 2.0f);
				}
			}
		}

		// -HandParmeter 
		HandParameter[parameter] = initialHandParameter - stepSize;
		HandEncoding(HandParameter, output);
		for (int i = 0; i < 48; i++)
		{
			spacePoint[i].X = output[i].x;
			spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
			spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
		}
		m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
		for (int i = 0; i < 48; i++)
		{
			if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
			{
				e_b_minus += pow(200 * convert_pixel_to_m, 2.0f);
			}
			else
			{
				float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
				if (depth != 0)
				{
					depth = depth / 1000; // convert to meter
					e_b_minus += pow(max(0.0f, spacePoint[i].Z - depth), 2.0f);
				}
				else
				{
					float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
					e_b_minus += pow(distant * convert_pixel_to_m, 2.0f);
				}
			}
		}

		if (e_b_plus < e_b_minus && e_b_plus < e_b)
		{
			HandParameter[parameter] = initialHandParameter + stepSize;
			for (int k = 0; k < 10; k++)
			{
				if (e_b_plus < e_b)
				{
					e_b = e_b_plus;
					HandParameter[parameter] += stepSize;

				}
				else
				{
					k = 10;
				}
				HandEncoding(HandParameter, output);
				for (int i = 0; i < 48; i++)
				{
					spacePoint[i].X = output[i].x;
					spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
					spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
				}
				m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
				for (int i = 0; i < 48; i++)
				{
					if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
					{
						e_b_plus += pow(200 * convert_pixel_to_m, 2.0f);
					}
					else
					{
						float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
						if (depth != 0)
						{
							depth = depth / 1000; // convert to meter
							e_b_plus += pow(max(0.0f, spacePoint[i].Z - depth), 2.0f);
						}
						else
						{
							float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
							e_b_plus += pow(distant * convert_pixel_to_m, 2.0f);
						}
					}
				}
			}
		}
		else if (e_b_minus < e_b_plus && e_b_minus < e_b)
		{
			HandParameter[parameter] = initialHandParameter - stepSize;
			for (int k = 0; k < 10; k++)
			{
				if (e_b_minus < e_b)
				{
					e_b = e_b_minus;
					HandParameter[parameter] -= stepSize;

				}
				else
				{
					k = 10;
				}
				HandEncoding(HandParameter, output);
				for (int i = 0; i < 48; i++)
				{
					spacePoint[i].X = output[i].x;
					spacePoint[i].Y = cos(degree * PI / 180.0f)*output[i].y - sin(degree * PI / 180.0f)*output[i].z - 0.4437489982;
					spacePoint[i].Z = -sin(degree * PI / 180.0f)*output[i].y - cos(degree * PI / 180.0f)*output[i].z + 0.9516233869;
				}
				m_pCoordinateMapper->MapCameraPointsToDepthSpace(48, spacePoint, 48, depthPoints);
				for (int i = 0; i < 48; i++)
				{
					if (depthPoints[i].Y < 0 || depthPoints[i].Y >= 424 || depthPoints[i].X < 0 || depthPoints[i].X >= 512)
					{
						e_b_minus += pow(200 * convert_pixel_to_m, 2.0f);
					}
					else
					{
						float depth = (float)depthMap.at<USHORT>(depthPoints[i].Y, depthPoints[i].X);
						if (depth != 0)
						{
							depth = depth / 1000; // convert to meter
							e_b_minus += pow(max(0.0f, spacePoint[i].Z - depth), 2.0f);
						}
						else
						{
							float distant = dist.at<float>(depthPoints[i].Y, depthPoints[i].X);
							e_b_minus += pow(distant * convert_pixel_to_m, 2.0f);
						}
					}
				}
			}
		}

		//Mat pointToSearch(48, 3, CV_32F);
		//for (int i = 0; i < 48; i++)
		//{
		//	pointToSearch.at<float>(i, 0) = spacePoint[i].X;
		//	pointToSearch.at<float>(i, 1) = spacePoint[i].Y;
		//	pointToSearch.at<float>(i, 2) = spacePoint[i].Z;
		//}
		//flann::Index kdtree(pointToSearch, flann::LinearIndexParams());


	for (int i = 16; i < 40; i++)
		for (int j = i + 1; j < 40; j++)
		{
			Point3f a(spacePoint[i].X, spacePoint[i].Y, spacePoint[i].Z);
			Point3f b(spacePoint[j].X, spacePoint[j].Y, spacePoint[j].Z);
			e_l += pow(max(particleRadius * 2.0f - (float)norm(a - b), 0.0f),2.0f);
		}


	for (int i = 0; i < 256; i++)
	{
		CameraSpacePoint camera_surface;
		DepthSpacePoint random_surface;
		UINT16 depth;
		do
		{
			float random_x = (((float)rand() / (float)(RAND_MAX)) * 100.0) - 50.0f;
			float random_y = (((float)rand() / (float)(RAND_MAX)) * 100.0f) - 50.0f;
			random_surface.Y = Handcenter.y + random_y;
			random_surface.X = Handcenter.x + random_x;
			depth = depthMap.at<ushort>(random_surface.Y, random_surface.X);

		} while (depth == 0);
		//circle(depthMap, cv::Point(random_surface.X, random_surface.Y), 1, cv::Scalar(128), -1);
		

		m_pCoordinateMapper->MapDepthPointToCameraSpace(random_surface, depth, &camera_surface);

		float min_distance = 999999; 
		////float query[3] = { camera_surface.X, camera_surface.Y, camera_surface.Z };
		//vector<float> query;
		//query.push_back(camera_surface.X);
		//query.push_back(camera_surface.Y);
		//query.push_back(camera_surface.Z);
		//vector<int> indice;
		//vector<float> distance;
		//kdtree.knnSearch(query, indice, distance, 1);
		
		for (int j = 0; j < 48; j++)
		{
			Point3f a(spacePoint[j].X, spacePoint[j].Y, spacePoint[j].Z);
			Point3f b(camera_surface.X, camera_surface.Y, camera_surface.Z);
			float distance = norm(a - b);
			if (distance < min_distance)
			{
				min_distance = distance;
			}
		}

		e_d +=  pow(min_distance - particleRadius, 2.0f);
	}


	result = (48.0f / 256.0f)*e_d+e_l+e_b;
	return result;
}

void KinectHandle::HandEncoding(float* HandParameter, D3DXVECTOR4* output)
{


	//D3DXMATRIX index_offset, middle_offset, ring_offset, pinky_offset, thumb_offset;
	//D3DXMatrixTranslation(&index_offset, -particleRadius*0.25f, particleRadius*1.0f, 0.0f);
	//D3DXMatrixTranslation(&middle_offset, 0.0f, particleRadius*1.2f, 0.0f);
	//D3DXMatrixTranslation(&ring_offset, particleRadius*0.25f, particleRadius*1.0f, 0.0f);
	//D3DXMatrixTranslation(&pinky_offset, particleRadius*0.5f, particleRadius*0.6, 0.0f);
	D3DXMATRIX index_offset, middle_offset, ring_offset, pinky_offset, thumb_offset;
	D3DXMatrixTranslation(&index_offset, 0.0f, particleRadius*1.0f, 0.0f);
	D3DXMatrixTranslation(&middle_offset, 0.0f, particleRadius*1.2f, 0.0f);
	D3DXMatrixTranslation(&ring_offset, 0.0f, particleRadius*1.0f, 0.0f);
	D3DXMatrixTranslation(&pinky_offset, 0.0f, particleRadius*0.6, 0.0f);

	D3DXMatrixTranslation(&thumb_offset, particleRadius*0.5, particleRadius*0.5, 0.0f);

	//World Coordinate -Palm
	D3DXMATRIX global_translate;
	D3DXMatrixTranslation(&global_translate, HandParameter[0], HandParameter[1], HandParameter[2]);

	D3DXMATRIX global_rotate, global;
	D3DXQUATERNION global_quaternion;
	global_quaternion.x = HandParameter[3];
	global_quaternion.y = HandParameter[4];
	global_quaternion.z = HandParameter[5];
	global_quaternion.w = HandParameter[6];
	D3DXQuaternionNormalize(&global_quaternion, &global_quaternion);
	D3DXMatrixRotationQuaternion(&global_rotate, &global_quaternion);

	global = global_rotate* global_translate ;

	// index_finger
	D3DXMATRIX index_rotate[3], index_transform[3], index_final[6], index_position;

	D3DXMatrixRotationYawPitchRoll(&index_rotate[0], 0.0f, HandParameter[7], HandParameter[8]);
	index_transform[0] = index_offset*index_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&index_rotate[1], 0.0f, HandParameter[9], 0.0f);
	index_transform[1] = index_offset * index_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&index_rotate[2], 0.0f, HandParameter[10], 0.0f);
	index_transform[2] = index_offset * index_rotate[2];

	D3DXMatrixTranslation(&index_position, -particleRadius * 3.5, particleRadius * 3, 0.0f);
	index_final[0] = index_transform[0] * (index_position* global);
	index_final[1] = index_offset * index_final[0];
	index_final[2] = index_offset * index_final[1];
	index_final[3] = index_transform[1] * index_final[2];
	index_final[4] = index_offset * index_final[3];
	index_final[5] = index_transform[2] * index_final[4];

	// middle_finger
	D3DXMATRIX middle_rotate[3], middle_transform[3], middle_final[6], middle_position;

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[0], 0.0f, HandParameter[11], HandParameter[12]);
	middle_transform[0] = middle_offset*middle_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[1], 0.0f, HandParameter[13], 0.0f);
	middle_transform[1] = middle_offset * middle_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&middle_rotate[2], 0.0f, HandParameter[14], 0.0f);
	middle_transform[2] = middle_offset * middle_rotate[2];

	D3DXMatrixTranslation(&middle_position, -particleRadius, particleRadius * 3.0, 0.0f);
	middle_final[0] = middle_transform[0] * (middle_position* global);
	middle_final[1] = middle_offset * middle_final[0];
	middle_final[2] = middle_offset * middle_final[1];
	middle_final[3] = middle_transform[1] * middle_final[2];
	middle_final[4] = middle_offset * middle_final[3];
	middle_final[5] = middle_transform[2] * middle_final[4];

	// ring_finger
	D3DXMATRIX ring_rotate[3], ring_transform[3], ring_final[6], ring_position;

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[0], 0.0f, HandParameter[15], HandParameter[16]);
	ring_transform[0] = ring_offset*ring_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[1], 0.0f, HandParameter[17], 0.0f);
	ring_transform[1] = ring_offset * ring_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&ring_rotate[2], 0.0f, HandParameter[18], 0.00);
	ring_transform[2] = ring_offset * ring_rotate[2];

	D3DXMatrixTranslation(&ring_position, particleRadius*1.5, particleRadius * 3.0, 0.0f);
	ring_final[0] = ring_transform[0] * (ring_position* global);
	ring_final[1] = ring_offset * ring_final[0];
	ring_final[2] = ring_offset * ring_final[1];
	ring_final[3] = ring_transform[1] * ring_final[2];
	ring_final[4] = ring_offset * ring_final[3];
	ring_final[5] = ring_transform[2] * ring_final[4];


	//Pinky finger
	D3DXMATRIX pinky_rotate[3], pinky_transform[3], pinky_final[6], pinky_position;

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[0], 0.0f, HandParameter[19], HandParameter[20]);
	pinky_transform[0] = pinky_offset*pinky_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[1], 0.0f, HandParameter[21], 0.0f);
	pinky_transform[1] = pinky_offset * pinky_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&pinky_rotate[2], 0.0f, HandParameter[22], 0.0f);
	pinky_transform[2] = pinky_offset * pinky_rotate[2];

	D3DXMatrixTranslation(&pinky_position, particleRadius*3.5, particleRadius* 3.0, 0.0f);
	pinky_final[0] = pinky_transform[0] * (pinky_position* global);
	pinky_final[1] = pinky_offset * pinky_final[0];
	pinky_final[2] = pinky_offset * pinky_final[1];
	pinky_final[3] = pinky_transform[1] * pinky_final[2];
	pinky_final[4] = pinky_offset * pinky_final[3];
	pinky_final[5] = pinky_transform[2] * pinky_final[4];


	//thumb
	D3DXMATRIX thumb_rotate[3], thumb_transform[3], thumb_final[8], thumb_position;

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[0], 0.0f, HandParameter[23], HandParameter[24]);
	thumb_transform[0] = thumb_offset*thumb_rotate[0];

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[1], 0.0f, HandParameter[25], 0.0f);
	thumb_transform[1] = thumb_offset * thumb_rotate[1];

	D3DXMatrixRotationYawPitchRoll(&thumb_rotate[2], 0.0f, HandParameter[26], 0.0f);
	thumb_transform[2] = thumb_offset * thumb_rotate[2];

	D3DXMatrixTranslation(&thumb_position, particleRadius * 2, -particleRadius * 2.0, -particleRadius);
	thumb_final[0] = thumb_transform[0] * (thumb_position* global);
	thumb_final[1] = thumb_offset * thumb_final[0];
	thumb_final[2] = thumb_offset * thumb_final[1];
	thumb_final[3] = thumb_offset * thumb_final[2];
	thumb_final[4] = thumb_offset * thumb_final[3];
	thumb_final[5] = thumb_transform[1] * thumb_final[4];
	thumb_final[6] = thumb_offset * thumb_final[5];
	thumb_final[7] = thumb_transform[2] * thumb_final[6];


	D3DXVECTOR4 palm[16];


	palm[6] = D3DXVECTOR4(particleRadius*2.50, -particleRadius * 3, 0.0f, 1.0f);
	palm[7] = D3DXVECTOR4(-particleRadius*2.50, -particleRadius * 3, 0.0f, 1.0f);
	palm[10] = D3DXVECTOR4(particleRadius*0.75, -particleRadius * 3, 0.0f, 1.0f);
	palm[11] = D3DXVECTOR4(-particleRadius*0.75, -particleRadius * 3, 0.0f, 1.0f);

	palm[14] = D3DXVECTOR4(particleRadius * 3, -particleRadius*1, 0.0f, 1.0f);
	palm[15] = D3DXVECTOR4(-particleRadius * 3, -particleRadius*1, 0.0f, 1.0f);
	palm[2] = D3DXVECTOR4(particleRadius, -particleRadius*1, 0.0f, 1.0f);
	palm[3] = D3DXVECTOR4(-particleRadius, -particleRadius, 0.0f, 1.0f);

	palm[12] = D3DXVECTOR4(particleRadius * 3, particleRadius, 0.0f, 1.0f);
	palm[13] = D3DXVECTOR4(-particleRadius * 3, particleRadius, 0.0f, 1.0f);
	palm[0] = D3DXVECTOR4(particleRadius, particleRadius,0.0f, 1.0f);
	palm[1] = D3DXVECTOR4(-particleRadius, particleRadius, 0.0f, 1.0f);

	palm[4] = D3DXVECTOR4(particleRadius*3.25, particleRadius *3, 0.0f, 1.0f);
	palm[5] = D3DXVECTOR4(-particleRadius*3.25, particleRadius * 3, 0.0f, 1.0f);
	palm[8] = D3DXVECTOR4(particleRadius*1.25, particleRadius * 3, 0.0f, 1.0f);
	palm[9] = D3DXVECTOR4(-particleRadius, particleRadius * 3, 0.0f, 1.0f);

	
	D3DXVECTOR4 origin = D3DXVECTOR4(0, 0, 0, 1.0);

	for (int i = 0; i < 16; i++)
		D3DXVec4Transform(&output[i], &palm[i], &global);

	for (int i = 0; i < 6; i++)
	{
		D3DXVec4Transform(&output[i + 16], &origin, &index_final[i]);
		D3DXVec4Transform(&output[i + 22], &origin, &middle_final[i]);
		D3DXVec4Transform(&output[i + 28], &origin, &ring_final[i]);
		D3DXVec4Transform(&output[i + 34], &origin, &pinky_final[i]);

	}

	for (int i = 0; i < 8; i++)
		D3DXVec4Transform(&output[i + 40], &origin, &thumb_final[i]);

}


float* KinectHandle::optimized(float inputConfig[27], float suggestConfig[7], Point HandPosition_depth, CameraSpacePoint
	HandPosition_camera, Mat depthMap, Mat dist) 
{
	float* outputConfig = new float[27];
	float x[num_particle_pso][27];
	float xPbest[num_particle_pso][27];
	float pbest[num_particle_pso];
	float gbest = 999999;
	float v[num_particle_pso][27];
	float minDistance;
	int minDistanceIndex;
	int i, j, k, m;

	//apply to limit search range
	minConfig[0] = inputConfig[0] - 0.05;
	maxConfig[0] = inputConfig[0] + 0.05;
	minConfig[1] = inputConfig[1] - 0.05;
	maxConfig[1] = inputConfig[1] + 0.05;
	minConfig[2] = inputConfig[2] - 0.05;
	maxConfig[2] = inputConfig[2] + 0.05;
	
	//Generate Particle From the last frame


	//Set first particle as last frame particle
	for (j = 0; j<27; j++)
	{
		x[0][j] = inputConfig[j];
		xPbest[0][j] = inputConfig[j];
		outputConfig[j] = inputConfig[j];
		v[0][j] = 0;
		pbest[0] = 9999;
	}
	
	// Random position for 4 particle without finger configuration
	for (i = 1; i<5; i++)
	{
		x[i][0] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[0];
		x[i][1] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[1];
		x[i][2] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[2];

		v[i][0] = 0;
		v[i][1] = 0;
		v[i][2] = 0;

		xPbest[i][0] = x[i][0];
		xPbest[i][1] = x[i][1];
		xPbest[i][2] = x[i][2];

		for (j = 3; j<7; j++)
		{
			x[i][j] = suggestConfig[j];
			xPbest[i][j] = suggestConfig[j];
			v[i][j] = 0;
		}
		for (j = 7; j < 27; j++)
		{
			x[i][j] = inputConfig[j];
			xPbest[i][j] = inputConfig[j];
			v[i][j] = 0;
		}
		pbest[i] = 99999999;
	}
	// Other Particle Random Every thing
	for (i = 5; i< num_particle_pso ; i++)
	{
		x[i][0] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[0];
		x[i][1] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[1];
		x[i][2] = ((float)rand() / (float(RAND_MAX))*0.1 - 0.05) + suggestConfig[2];
		v[i][0] = 0;
		v[i][1] = 0;
		v[i][2] = 0;
		xPbest[i][0] = x[i][0];
		xPbest[i][1] = x[i][1];
		xPbest[i][2] = x[i][2];
		for (j = 3; j<7; j++)
		{
			x[i][j] = (float)rand() / (float(RAND_MAX))*0.5 - 0.25f + suggestConfig[j];
			v[i][j] = 0;
			xPbest[i][j] = x[i][j];
		}		

		for (j = 7; j<23; j++)
		{

			x[i][j] = ((float)rand() / (float)(RAND_MAX)) * (maxConfig[j] - minConfig[j])+ inputConfig[j];
			v[i][j] = 0;
			xPbest[i][j] = x[i][j];
		}
		for (j = 23; j<27; j++)
		{

			x[i][j] = 0;//((float)rand() / (float)(RAND_MAX)) * (maxConfig[j] - minConfig[j]) + inputConfig[j];
			v[i][j] = 0;
			xPbest[i][j] = x[i][j];
		}
		pbest[i] = 999999;
	}

	//For Each Generation
	for (k = 0; k<num_generation_pso; k++)
	{
		omp_set_num_threads(4);
		#pragma omp parallel for private(j)
		for (i = 0; i<num_particle_pso; i++)
		{
			float energy = compareHand(x[i], depthMap, HandPosition_depth, dist);
			if (energy < pbest[i])
			{
				for (j = 0; j<27; j++)
				{
					xPbest[i][j] = x[i][j];
				}
				pbest[i] = energy;
			}

			if (energy < gbest)
			{
				for (j = 0; j<27; j++)
				{
					outputConfig[j] = x[i][j];
				}
				gbest = energy;
			}

		}

		//Update Function
		#pragma omp parallel for private(j)
		for (i = 0; i<num_particle_pso; i++)
		{
			for (j = 0; j<27; j++)
			{
				v[i][j] = 0.729843f*(v[i][j] + (float)rand() / float(RAND_MAX)*2.8f *(xPbest[i][j] - x[i][j]) + 1.3f* (float)rand() / (float)RAND_MAX *(outputConfig[j] - x[i][j]));
				x[i][j] = x[i][j] + v[i][j];

				if (x[i][j]>maxConfig[j] && j != 3 && j!=4 && j != 5 && j!= 6)
				{
					x[i][j] = maxConfig[j] - (x[i][j] - maxConfig[j]);
				}
				if (x[i][j]<minConfig[j] && j != 3 && j != 4 && j != 5 && j != 6)
				{
					x[i][j] = minConfig[j] + ( minConfig[j] - x[i][j]);
				}

			}

		}
	}
	return outputConfig;
}