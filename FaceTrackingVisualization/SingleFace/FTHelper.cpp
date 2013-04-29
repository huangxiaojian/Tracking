//------------------------------------------------------------------------------
// <copyright file="FTHelper.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "StdAfx.h"
#include "FTHelper.h"
#include "Visualize.h"

#include <math.h>

#ifdef _DEBUG
#include <iostream>
#endif // _DEBUG


#ifdef SAMPLE_OPTIONS
#include "Options.h"
#else
PVOID _opt = NULL;
#endif

FTHelper::FTHelper()
{
    m_pFaceTracker = 0;
    m_hWnd = NULL;
    m_pFTResult = NULL;
    m_colorImage = NULL;
    m_depthImage = NULL;
    m_ApplicationIsRunning = false;
    m_LastTrackSucceeded = false;
    m_CallBack = NULL;
    m_XCenterFace = 0;
    m_YCenterFace = 0;
    m_hFaceTrackingThread = NULL;
    m_DrawMask = TRUE;
    m_depthType = NUI_IMAGE_TYPE_DEPTH;
    m_depthRes = NUI_IMAGE_RESOLUTION_INVALID;
    m_bNearMode = FALSE;
    m_bFallbackToDefault = FALSE;
    m_colorType = NUI_IMAGE_TYPE_COLOR;
    m_colorRes = NUI_IMAGE_RESOLUTION_INVALID;
}

FTHelper::~FTHelper()
{
    Stop();
}

HRESULT FTHelper::Init(HWND hWnd, FTHelperCallBack callBack, PVOID callBackParam, 
                       NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, BOOL bFallbackToDefault, NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes, BOOL bSeatedSkeletonMode)
{
    if (!hWnd || !callBack)
    {
        return E_INVALIDARG;
    }
    m_hWnd = hWnd;
    m_CallBack = callBack;
    m_CallBackParam = callBackParam;
    m_ApplicationIsRunning = true;
    m_depthType = depthType;
    m_depthRes = depthRes;
    m_bNearMode = bNearMode;
    m_bFallbackToDefault = bFallbackToDefault;
    m_bSeatedSkeletonMode = bSeatedSkeletonMode;
    m_colorType = colorType;
    m_colorRes = colorRes;
    m_hFaceTrackingThread = CreateThread(NULL, 0, FaceTrackingStaticThread, (PVOID)this, 0, 0);

	m_gazeTrack = new GazeTracking();
	m_gazeTrack->initialize("res/haarcascade_frontalface_alt.xml");
    return S_OK;
}

HRESULT FTHelper::Stop()
{
    m_ApplicationIsRunning = false;
    if (m_hFaceTrackingThread)
    {
        WaitForSingleObject(m_hFaceTrackingThread, 1000);
    }
    m_hFaceTrackingThread = 0;
    return S_OK;
}

BOOL FTHelper::SubmitFraceTrackingResult(IFTResult* pResult)
{
    if (pResult != NULL && SUCCEEDED(pResult->GetStatus()))
    {
        if (m_CallBack)
        {
            (*m_CallBack)(m_CallBackParam);
        }

        if (m_DrawMask)
        {
            FLOAT* pSU = NULL;
            UINT numSU;
            BOOL suConverged;
            m_pFaceTracker->GetShapeUnits(NULL, &pSU, &numSU, &suConverged);
            POINT viewOffset = {0, 0};
            FT_CAMERA_CONFIG cameraConfig;
            if (m_KinectSensorPresent)
            {
                m_KinectSensor.GetVideoConfiguration(&cameraConfig);
            }
            else
            {
                cameraConfig.Width = 640;
                cameraConfig.Height = 480;
                cameraConfig.FocalLength = 500.0f;
            }
            IFTModel* ftModel;
            HRESULT hr = m_pFaceTracker->GetFaceModel(&ftModel);
            if (SUCCEEDED(hr))
            {
				IplImage *img = cvCreateImage(cvSize(m_colorImage->GetWidth(), m_colorImage->GetHeight()), IPL_DEPTH_8U, 4);
				memcpy(img->imageData, m_colorImage->GetBuffer(), m_colorImage->GetBufferSize());
				cv::Mat frame(img, true);
				if(!frame.empty())
				{
					m_gazeTrack->process(frame);
#ifdef _DEBUG
					if(m_gazeTrack->isFindFace())
						std::cout << "Gaze Tracked!" << std::endl;
#endif // _DEBUG

				}

				FLOAT *pAUs;
				UINT auCount;
				hr = m_pFTResult->GetAUCoefficients(&pAUs, &auCount);
				FLOAT scale, rotationXYZ[3], translationXYZ[3];
				m_pFTResult->Get3DPose(&scale, rotationXYZ, translationXYZ);
				ftModel->Get3DShape(pSU, ftModel->GetSUCount(), pAUs, ftModel->GetAUCount(), scale, rotationXYZ, translationXYZ, m_pPts3D, VERTEXCOUNT);
				ftModel->GetProjectedShape(&cameraConfig, 1.0, viewOffset, pSU, ftModel->GetSUCount(), pAUs, auCount, 
					scale, rotationXYZ, translationXYZ, m_pPts2D, VERTEXCOUNT);
				ftModel->GetTriangles(&m_pTriangles, &m_TriangleCount);

				int x, y;
				POINT pos;
				m_gazeTrack->getLeftPupilXY(x, y);
				pos.x = x;
				pos.y = y;
				DrawGazeInImage(pos, 5, 0xffff0000);
				m_gazeTrack->getRightPupilXY(x, y);
				pos.x = x;
				pos.y = y;
				DrawGazeInImage(pos, 5, 0xffff0000);

				VisualizeFaceModel(ftModel, &cameraConfig, pSU, 1.0, viewOffset, 0xffffff00);
                //hr = VisualizeFaceModel(m_colorImage, ftModel, &cameraConfig, pSU, 1.0, viewOffset, pResult, 0x00FFFF00);
                ftModel->Release();
            }
        }
    }
    return TRUE;
}

// We compute here the nominal "center of attention" that is used when zooming the presented image.
void FTHelper::SetCenterOfImage(IFTResult* pResult)
{
    float centerX = ((float)m_colorImage->GetWidth())/2.0f;
    float centerY = ((float)m_colorImage->GetHeight())/2.0f;
    if (pResult)
    {
        if (SUCCEEDED(pResult->GetStatus()))
        {
            RECT faceRect;
            pResult->GetFaceRect(&faceRect);
            centerX = (faceRect.left+faceRect.right)/2.0f;
            centerY = (faceRect.top+faceRect.bottom)/2.0f;
        }
        m_XCenterFace += 0.02f*(centerX-m_XCenterFace);
        m_YCenterFace += 0.02f*(centerY-m_YCenterFace);
    }
    else
    {
        m_XCenterFace = centerX;
        m_YCenterFace = centerY;
    }
}

// Get a video image and process it.
void FTHelper::CheckCameraInput()
{
    HRESULT hrFT = E_FAIL;

    if (m_KinectSensorPresent && m_KinectSensor.GetVideoBuffer())
    {
        HRESULT hrCopy = m_KinectSensor.GetVideoBuffer()->CopyTo(m_colorImage, NULL, 0, 0);
        if (SUCCEEDED(hrCopy) && m_KinectSensor.GetDepthBuffer())
        {
            hrCopy = m_KinectSensor.GetDepthBuffer()->CopyTo(m_depthImage, NULL, 0, 0);
        }
        // Do face tracking
        if (SUCCEEDED(hrCopy))
        {
            FT_SENSOR_DATA sensorData(m_colorImage, m_depthImage, m_KinectSensor.GetZoomFactor(), m_KinectSensor.GetViewOffSet());

            FT_VECTOR3D* hint = NULL;
            if (SUCCEEDED(m_KinectSensor.GetClosestHint(m_hint3D)))
            {
                hint = m_hint3D;
            }
            if (m_LastTrackSucceeded)
            {
                hrFT = m_pFaceTracker->ContinueTracking(&sensorData, hint, m_pFTResult);
            }
            else
            {
                hrFT = m_pFaceTracker->StartTracking(&sensorData, NULL, hint, m_pFTResult);
            }
        }
    }

    m_LastTrackSucceeded = SUCCEEDED(hrFT) && SUCCEEDED(m_pFTResult->GetStatus());
    if (m_LastTrackSucceeded)
    {
        SubmitFraceTrackingResult(m_pFTResult);
    }
    else
    {
        m_pFTResult->Reset();
    }
    SetCenterOfImage(m_pFTResult);
}

DWORD WINAPI FTHelper::FaceTrackingStaticThread(PVOID lpParam)
{
    FTHelper* context = static_cast<FTHelper*>(lpParam);
    if (context)
    {
        return context->FaceTrackingThread();
    }
    return 0;
}

DWORD WINAPI FTHelper::FaceTrackingThread()
{
    FT_CAMERA_CONFIG videoConfig;
    FT_CAMERA_CONFIG depthConfig;
    FT_CAMERA_CONFIG* pDepthConfig = NULL;

    // Try to get the Kinect camera to work
    HRESULT hr = m_KinectSensor.Init(m_depthType, m_depthRes, m_bNearMode, m_bFallbackToDefault, m_colorType, m_colorRes, m_bSeatedSkeletonMode);
    if (SUCCEEDED(hr))
    {
        m_KinectSensorPresent = TRUE;
        m_KinectSensor.GetVideoConfiguration(&videoConfig);
        m_KinectSensor.GetDepthConfiguration(&depthConfig);
        pDepthConfig = &depthConfig;
        m_hint3D[0] = m_hint3D[1] = FT_VECTOR3D(0, 0, 0);
    }
    else
    {
        m_KinectSensorPresent = FALSE;
        WCHAR errorText[MAX_PATH];
        ZeroMemory(errorText, sizeof(WCHAR) * MAX_PATH);
        wsprintf(errorText, L"Could not initialize the Kinect sensor. hr=0x%x\n", hr);
        MessageBoxW(m_hWnd, errorText, L"Face Tracker Initialization Error\n", MB_OK);
        return 1;
    }

    // Try to start the face tracker.
    m_pFaceTracker = FTCreateFaceTracker(_opt);
    if (!m_pFaceTracker)
    {
        MessageBoxW(m_hWnd, L"Could not create the face tracker.\n", L"Face Tracker Initialization Error\n", MB_OK);
        return 2;
    }

    hr = m_pFaceTracker->Initialize(&videoConfig, pDepthConfig, NULL, NULL); 
    if (FAILED(hr))
    {
        WCHAR path[512], buffer[1024];
        GetCurrentDirectoryW(ARRAYSIZE(path), path);
        wsprintf(buffer, L"Could not initialize face tracker (%s). hr=0x%x", path, hr);

        MessageBoxW(m_hWnd, /*L"Could not initialize the face tracker.\n"*/ buffer, L"Face Tracker Initialization Error\n", MB_OK);

        return 3;
    }

    hr = m_pFaceTracker->CreateFTResult(&m_pFTResult);
    if (FAILED(hr) || !m_pFTResult)
    {
        MessageBoxW(m_hWnd, L"Could not initialize the face tracker result.\n", L"Face Tracker Initialization Error\n", MB_OK);
        return 4;
    }

    // Initialize the RGB image.
    m_colorImage = FTCreateImage();
    if (!m_colorImage || FAILED(hr = m_colorImage->Allocate(videoConfig.Width, videoConfig.Height, FTIMAGEFORMAT_UINT8_B8G8R8X8)))
    {
        return 5;
    }

    if (pDepthConfig)
    {
        m_depthImage = FTCreateImage();
        if (!m_depthImage || FAILED(hr = m_depthImage->Allocate(depthConfig.Width, depthConfig.Height, FTIMAGEFORMAT_UINT16_D13P3)))
        {
            return 6;
        }
    }

    SetCenterOfImage(NULL);
    m_LastTrackSucceeded = false;

    while (m_ApplicationIsRunning)
    {
        CheckCameraInput();
        InvalidateRect(m_hWnd, NULL, FALSE);
        UpdateWindow(m_hWnd);
        Sleep(16);
    }

    m_pFaceTracker->Release();
    m_pFaceTracker = NULL;

    if(m_colorImage)
    {
        m_colorImage->Release();
        m_colorImage = NULL;
    }

    if(m_depthImage) 
    {
        m_depthImage->Release();
        m_depthImage = NULL;
    }

    if(m_pFTResult)
    {
        m_pFTResult->Release();
        m_pFTResult = NULL;
    }
    m_KinectSensor.Release();
    return 0;
}

HRESULT FTHelper::GetCameraConfig(FT_CAMERA_CONFIG* cameraConfig)
{
    return m_KinectSensorPresent ? m_KinectSensor.GetVideoConfiguration(cameraConfig) : E_FAIL;
}

HRESULT FTHelper::VisualizeFacetracker(UINT32 color)
{
	if (!m_colorImage->GetBuffer() || !m_pFTResult)
	{
		return E_POINTER;
	}

	// Insufficient data points to render face data
	FT_VECTOR2D* pPts2D;
	UINT pts2DCount;
	HRESULT hr = m_pFTResult->Get2DShapePoints(&pPts2D, &pts2DCount);
	if (FAILED(hr))
	{
		return hr;
	}

	if (pts2DCount < 86)
	{
		return E_INVALIDARG;
	}


	POINT* pFaceModel2DPoint = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * pts2DCount));
	if (!pFaceModel2DPoint)
	{
		return E_OUTOFMEMORY;
	}


	for (UINT ipt = 0; ipt < pts2DCount; ++ipt)
	{
		pFaceModel2DPoint[ipt].x = LONG(pPts2D[ipt].x + 0.5f);
		pFaceModel2DPoint[ipt].y = LONG(pPts2D[ipt].y + 0.5f);
	}

	for (UINT ipt = 0; ipt < 8; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt+1)%8];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 8; ipt < 16; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt-8+1)%8+8];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 16; ipt < 26; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt-16+1)%10+16];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 26; ipt < 36; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt-26+1)%10+26];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 36; ipt < 47; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[ipt+1];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 48; ipt < 60; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt-48+1)%12+48];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 60; ipt < 68; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[(ipt-60+1)%8+60];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}

	for (UINT ipt = 68; ipt < 86; ++ipt)
	{
		POINT ptStart = pFaceModel2DPoint[ipt];
		POINT ptEnd = pFaceModel2DPoint[ipt+1];
		m_colorImage->DrawLine(ptStart, ptEnd, color, 1);
	}
	_freea(pFaceModel2DPoint);

	return hr;
}

HRESULT FTHelper::VisualizeFaceModel(IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, FLOAT zoomFactor, POINT viewOffset, UINT32 color)

{
	if (!m_colorImage || !pModel || !pCameraConfig || !pSUCoef || !m_pFTResult)
	{
		return E_POINTER;
	}


	HRESULT hr = S_OK;
	UINT vertexCount = pModel->GetVertexCount();
	FT_VECTOR2D* pPts2D = reinterpret_cast<FT_VECTOR2D*>(_malloca(sizeof(FT_VECTOR2D) * vertexCount));
#ifdef _DEBUG	
	FT_VECTOR3D* pPts3D = reinterpret_cast<FT_VECTOR3D*>(_malloca(sizeof(FT_VECTOR3D) * vertexCount));
#endif
	if (pPts2D)
	{
		FLOAT *pAUs;
		UINT auCount;
		hr = m_pFTResult->GetAUCoefficients(&pAUs, &auCount);
		if (SUCCEEDED(hr))
		{
			FLOAT scale, rotationXYZ[3], translationXYZ[3];
			hr = m_pFTResult->Get3DPose(&scale, rotationXYZ, translationXYZ);
#ifdef _DEBUG
			pModel->Get3DShape(pSUCoef, pModel->GetSUCount(), pAUs, pModel->GetAUCount(), scale, rotationXYZ, translationXYZ, pPts3D, vertexCount);
			std::cout << pPts3D[5].x << ' ' << pPts3D[5].y << ' ' << pPts3D[5].z << std::endl;
#endif

			if (SUCCEEDED(hr))
			{
				hr = pModel->GetProjectedShape(pCameraConfig, zoomFactor, viewOffset, pSUCoef, pModel->GetSUCount(), pAUs, auCount, 
					scale, rotationXYZ, translationXYZ, pPts2D, vertexCount);
				if (SUCCEEDED(hr))
				{
					POINT* p3DMdl   = reinterpret_cast<POINT*>(_malloca(sizeof(POINT) * vertexCount));
					if (p3DMdl)
					{
						for (UINT i = 0; i < vertexCount; ++i)
						{
							p3DMdl[i].x = LONG(pPts2D[i].x + 0.5f);
							p3DMdl[i].y = LONG(pPts2D[i].y + 0.5f);
						}

						FT_TRIANGLE* pTriangles;
						UINT triangleCount;
						hr = pModel->GetTriangles(&pTriangles, &triangleCount);
						if (SUCCEEDED(hr))
						{
							struct EdgeHashTable
							{
								UINT32* pEdges;
								UINT edgesAlloc;

								void Insert(int a, int b) 
								{
									UINT32 v = (min(a, b) << 16) | max(a, b);
									UINT32 index = (v + (v << 8)) * 49157, i;
									for (i = 0; i < edgesAlloc - 1 && pEdges[(index + i) & (edgesAlloc - 1)] && v != pEdges[(index + i) & (edgesAlloc - 1)]; ++i)
									{
									}
									pEdges[(index + i) & (edgesAlloc - 1)] = v;
								}
							} eht;

							eht.edgesAlloc = 1 << UINT(log(2.f * (1 + vertexCount + triangleCount)) / log(2.f));
							eht.pEdges = reinterpret_cast<UINT32*>(_malloca(sizeof(UINT32) * eht.edgesAlloc));
							if (eht.pEdges)
							{
								ZeroMemory(eht.pEdges, sizeof(UINT32) * eht.edgesAlloc);
								for (UINT i = 0; i < triangleCount; ++i)
								{ 
									eht.Insert(pTriangles[i].i, pTriangles[i].j);
									eht.Insert(pTriangles[i].j, pTriangles[i].k);
									eht.Insert(pTriangles[i].k, pTriangles[i].i);
								}
								for (UINT i = 0; i < eht.edgesAlloc; ++i)
								{
									if(eht.pEdges[i] != 0)
									{
										m_colorImage->DrawLine(p3DMdl[eht.pEdges[i] >> 16], p3DMdl[eht.pEdges[i] & 0xFFFF], color, 1);
									}
								}
								_freea(eht.pEdges);
							}

							// Render the face rect in magenta
							//RECT rectFace;
							hr = m_pFTResult->GetFaceRect(&m_faceRect);
							if (SUCCEEDED(hr))
							{
								POINT leftTop = {m_faceRect.left, m_faceRect.top};
								POINT rightTop = {m_faceRect.right - 1, m_faceRect.top};
								POINT leftBottom = {m_faceRect.left, m_faceRect.bottom - 1};
								POINT rightBottom = {m_faceRect.right - 1, m_faceRect.bottom - 1};
								UINT32 nColor = 0xff00ff;
								SUCCEEDED(hr = m_colorImage->DrawLine(leftTop, rightTop, nColor, 1)) &&
									SUCCEEDED(hr = m_colorImage->DrawLine(rightTop, rightBottom, nColor, 1)) &&
									SUCCEEDED(hr = m_colorImage->DrawLine(rightBottom, leftBottom, nColor, 1)) &&
									SUCCEEDED(hr = m_colorImage->DrawLine(leftBottom, leftTop, nColor, 1));
							}
						}

						_freea(p3DMdl); 
					}
					else
					{
						hr = E_OUTOFMEMORY;
					}
				}
			}
		}
		_freea(pPts2D);
	}
	else
	{
		hr = E_OUTOFMEMORY;
	}
	return hr;
}

void FTHelper::DrawGazeInImage(POINT pos, int radius, UINT32 color)
{
	POINT up, left, down, right;
	down.x = up.x = pos.x;
	left.y = right.y = pos.y;

	for(int i = 1; i < radius;i ++)
	{
		up.y = pos.y+i;
		left.x = pos.x-i;
		right.x = pos.x + i;
		down.y = pos.y-i;
		m_colorImage->DrawLine(up, left, color, 1);
		m_colorImage->DrawLine(left, down, color, 1);
		m_colorImage->DrawLine(down, right, color, 1);
		m_colorImage->DrawLine(right, up, color, 1);
	}
}

void FTHelper::SaveModel(IFTModel* model, const float* pSUs, UINT32 suCount, const float* pAUs, UINT32 auCount, float scale, const float* rotationXYZ, const float* translationXYZ, int count)
{
	UINT32 vertexCount = model->GetVertexCount();//model->GetVertexCount(); 
	FT_VECTOR3D* pVertices = new FT_VECTOR3D[vertexCount]; 
	model->Get3DShape(pSUs, suCount, pAUs, auCount, scale, rotationXYZ, translationXYZ, pVertices, vertexCount); 
	UINT32 triangleCount = 0; 
	FT_TRIANGLE* pTriangles = NULL; 
	model->GetTriangles(&pTriangles, &triangleCount); 
	FILE* fobj = NULL; 
	char filename[10];
	sprintf_s(filename, "%d", count);
	strcat(filename, ".obj");
	fopen_s(&fobj, filename, "w"); 
	fprintf(fobj, "# %u vertices, # %u faces\n", vertexCount, triangleCount); 
	for (UINT32 vi = 0; vi < vertexCount; ++vi) { 
		fprintf(fobj, "v %f %f %f\n", pVertices[vi].x, pVertices[vi].y, pVertices[vi].z);
	} 
	for (UINT32 ti = 0; ti < triangleCount; ++ti) { 
		fprintf(fobj, "f %d %d %d\n", pTriangles[ti].i+1, pTriangles[ti].j+1, pTriangles[ti].k+1); 
	} 
	fclose(fobj); 
	delete[] pVertices; 
	return; 
}