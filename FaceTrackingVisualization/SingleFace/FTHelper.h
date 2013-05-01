//------------------------------------------------------------------------------
// <copyright file="FTHelper.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <FaceTrackLib.h>
#include "KinectSensor.h"

#include "gazeTracking.h"

#include "utilVector.h"

#define VERTEXCOUNT 121
#define TRIANGLECOUNT 206

#define LASTPOSITIONNUM 3

#ifndef max
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif

//#define OUTPUTTOFILE

#ifdef OUTPUTTOFILE
#define FPNUM 8
enum FileIndex{LEFTX, LEFTY, NOSEX, NOSEY, FILTERLEFTX, FILTERLEFTY, INLEFTX, INLEFTY};
extern FILE* fp[FPNUM];
//#define NEEDFILTER
#define INFUNCTION
#endif

typedef void (*FTHelperCallBack)(PVOID lpParam);

namespace util{

	FT_VECTOR3D PLUS(const FT_VECTOR3D& a, const FT_VECTOR3D& b)
	{
		return FT_VECTOR3D(a.x+b.x, a.y+b.y, a.z+b.z);
	}
	FT_VECTOR3D MINUS(const FT_VECTOR3D& a, const FT_VECTOR3D& b)
	{
		return FT_VECTOR3D(a.x-b.x, a.y-b.y, a.z-b.z);
	}
	FT_VECTOR3D TIMES(const FT_VECTOR3D& a, float t)
	{
		return FT_VECTOR3D(a.x*t, a.y*t, a.z*t);
	}

	FT_VECTOR3D triangleMap(const POINT& p, const POINT& a, const POINT& b, const POINT& c, const FT_VECTOR3D& d, const FT_VECTOR3D& e, const FT_VECTOR3D& f, float& x, float& y)
	{
		x = ((p.x-a.x)*(b.y-a.y)-(p.y-a.y)*(b.x-a.x))*1.0/((c.x-a.x)*(b.y-a.y)-(c.y-a.y)*(b.x-a.x));
		y = ((p.x-a.x)*(c.y-a.y)-(p.y-a.y)*(c.x-a.x))*1.0/((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
		return FT_VECTOR3D(x*(e.x-d.x)+y*(f.x-d.x)+d.x, x*(e.y-d.y)+y*(f.y-d.y)+d.y, x*(e.z-d.z)+y*(f.z-d.z)+d.z);
	}

	FT_VECTOR3D triangleMap(const POINT& p, const POINT P[3], const FT_VECTOR3D V[3], float& x, float& y)
	{
		return triangleMap(p, P[0], P[1], P[2], V[0], V[1], V[2], x ,y);
	}

	bool PointinTriangle(const POINT &A, const POINT &B, const POINT &C, const POINT &P, float tol = 0.0)
	{
		POINT v0 = {C.x-A.x, C.y-A.y};
		POINT v1 = {B.x-A.x, B.y-A.y};
		POINT v2 = {P.x-A.x, P.y-A.y};

		LONG dot00 = v0.x*v0.x+v0.y*v0.y ;
		LONG dot01 = v0.x*v1.x+v0.y*v1.y ;
		LONG dot02 = v0.x*v2.x+v0.y*v2.y ;
		LONG dot11 = v1.x*v1.x+v1.y*v1.y ;
		LONG dot12 = v1.x*v2.x+v1.y*v2.y ;

		double inverDeno = 1.0 / (dot00 * dot11 - dot01 * dot01) ;

		double u = (dot11 * dot02 - dot01 * dot12) * inverDeno ;
		if (u+tol < 0 || u-tol > 1) // if u out of range, return directly
		{
			return false ;
		}

		double v = (dot00 * dot12 - dot01 * dot02) * inverDeno ;
		if (v+tol < 0 || v-tol > 1) // if v out of range, return directly
		{
			return false ;
		}

		return u + v + tol <= 1 ;
	}

	bool PointinTriangle(const POINT &A, const POINT P[3], float tol = 0.0)
	{
		return PointinTriangle(P[0], P[1], P[2], A, tol);
	}

	POINT IntToPOINT(int x, int y)
	{
		POINT t;
		t.x = x;
		t.y = y;
		return t;
	}

	POINT LONGToPOINT(LONG x, LONG y)
	{
		POINT t;
		t.x = x;
		t.y = y;
		return t;
	}

	POINT FloatToPOINT(float x, float y)
	{
		POINT t;
		t.x = LONG(x + 0.5f);
		t.y = LONG(y + 0.5f);
		return t;
	}

	Vector3 FT_VECTOR3DtoVector3(const FT_VECTOR3D& p)
	{
		return Vector3(p.x, p.y, p.z);
	}

	FT_VECTOR3D Vector3ToFT_VECTOR3D(const Vector3& p)
	{
		return FT_VECTOR3D(p[0], p[1], p[2]);
	}

	Vector3 Normal(const Vector3& a, const Vector3& b, const Vector3& c)
	{
		Vector3 p1 = b-a;
		Vector3 p2 = c-a;
		return (p1%p2).normalize();
	}

	FT_VECTOR3D Normal(const FT_VECTOR3D &A, const FT_VECTOR3D &B, const FT_VECTOR3D& C)
	{
		return Vector3ToFT_VECTOR3D(Normal(FT_VECTOR3DtoVector3(A), FT_VECTOR3DtoVector3(B), FT_VECTOR3DtoVector3(C)));
	}

	FT_VECTOR3D Normal(FT_VECTOR3D P[3])
	{
		return Normal(P[0], P[1], P[2]);
	}
};

struct GazeState{
	inline void set(float ox, float oy, int t0, int t1, int t2)
	{
		x = ox;
		y = oy;
		triangle[0] = t0;
		triangle[1] = t1;
		triangle[2] = t2;
	}
	float x;
	float y;
	int triangle[3];
};

class FTHelper
{
public:
    FTHelper();
    ~FTHelper();

    HRESULT Init(HWND hWnd, FTHelperCallBack callBack, PVOID callBackParam, 
        NUI_IMAGE_TYPE depthType, NUI_IMAGE_RESOLUTION depthRes, BOOL bNearMode, BOOL bFallbackToDefault, NUI_IMAGE_TYPE colorType, NUI_IMAGE_RESOLUTION colorRes, BOOL bSeatedSkeletonMode);
    HRESULT Stop();
    IFTResult* GetResult()      { return(m_pFTResult);}
    BOOL IsKinectPresent()      { return(m_KinectSensorPresent);}
    IFTImage* GetColorImage()   { return(m_colorImage);}
    float GetXCenterFace()      { return(m_XCenterFace);}
    float GetYCenterFace()      { return(m_YCenterFace);}
    void SetDrawMask(BOOL drawMask) { m_DrawMask = drawMask;}
    BOOL GetDrawMask()          { return(m_DrawMask);}
    IFTFaceTracker* GetTracker() { return(m_pFaceTracker);}
    HRESULT GetCameraConfig(FT_CAMERA_CONFIG* cameraConfig);

	//gazetracking
	RECT& GetFaceRect()			{return m_faceRect;}
	FT_VECTOR3D& GetLeftPupil()	{return m_leftPupil;}
	FT_VECTOR3D& GetRightPupil(){return m_rightPupil;}

	//opengl
	FT_VECTOR3D* GetVertices()	{return m_pPts3D;}
	FT_TRIANGLE* GetTriangles() {return m_pTriangles;}
	int GetVertexNum()			{return VERTEXCOUNT;}
	int GetTriangleNum()		{return TRIANGLECOUNT;}

	//opengl tracking
	bool isLastTrackSucceed()	{return m_LastTrackSucceeded;}
	float GetPupilR()			{return m_pupilR;}

private:
    KinectSensor                m_KinectSensor;
    BOOL                        m_KinectSensorPresent;
    IFTFaceTracker*             m_pFaceTracker;
    HWND                        m_hWnd;
    IFTResult*                  m_pFTResult;
    IFTImage*                   m_colorImage;
    IFTImage*                   m_depthImage;
    FT_VECTOR3D                 m_hint3D[2];
    bool                        m_LastTrackSucceeded;
    bool                        m_ApplicationIsRunning;
    FTHelperCallBack            m_CallBack;
    LPVOID                      m_CallBackParam;
    float                       m_XCenterFace;
    float                       m_YCenterFace;
    HANDLE                      m_hFaceTrackingThread;
    BOOL                        m_DrawMask;
    NUI_IMAGE_TYPE              m_depthType;
    NUI_IMAGE_RESOLUTION        m_depthRes;
    BOOL                        m_bNearMode;
    BOOL                        m_bFallbackToDefault;
    BOOL                        m_bSeatedSkeletonMode;
    NUI_IMAGE_TYPE              m_colorType;
    NUI_IMAGE_RESOLUTION        m_colorRes;

	//gazetracking
	GazeTracking*				m_gazeTrack;
	RECT						m_faceRect;
	FT_VECTOR3D					m_leftPupil;
	FT_VECTOR3D					m_rightPupil;

	//opengl
	FT_VECTOR3D					m_pPts3D[VERTEXCOUNT];
	FT_VECTOR2D					m_pPts2D[VERTEXCOUNT];
	FT_TRIANGLE*				m_pTriangles;
	UINT						m_TriangleCount;

	//opengl gaze tracking
	float						m_pupilR;
	GazeState					m_gazeLastState[2];
#ifdef NEEDFILTER
	POINT						m_lastPosition[LASTPOSITIONNUM][2];
#endif
    BOOL SubmitFraceTrackingResult(IFTResult* pResult);
    void SetCenterOfImage(IFTResult* pResult);
    void CheckCameraInput();
    DWORD WINAPI FaceTrackingThread();
    static DWORD WINAPI FaceTrackingStaticThread(PVOID lpParam);

	HRESULT VisualizeFacetracker(UINT32 color);
	HRESULT VisualizeFaceModel(IFTModel* pModel, FT_CAMERA_CONFIG const* pCameraConfig, FLOAT const* pSUCoef, FLOAT zoomFactor, POINT viewOffset, UINT32 color);

	void DrawGazeInImage(POINT pos, int radius, UINT32 color);

	void SaveModel(IFTModel* model, const float* pSUs, UINT32 suCount, const float* pAUs, UINT32 auCount, float scale, const float* rotationXYZ, const float* translationXYZ, int count);

	void Map2Dto3D();
	float PointDis(int n, int m);
	void GetPupilFromLastState(FT_VECTOR3D& pupil, GazeState& gazeState);
#ifdef NEEDFILTER
	void LowFilter(POINT pupil[2]);
#endif
};
