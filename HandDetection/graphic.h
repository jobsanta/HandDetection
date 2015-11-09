#include <stdio.h>
#include <d3d10.h>
#include <D3DX10.h>
#include <DxErr.h>
#include "d3dUtil.h"
#include <iostream>

#include <Optional\DXUTShapes.h>

//#include "Box.h"
#include "modelclass.h"
#include "lightclass.h"
#include "lightshaderclass.h"

#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "d3d10.lib")
#pragma comment(lib, "d3dx10.lib")

#include "nvapi.h"
#pragma comment(lib, "nvapi.lib")
#include "terrainclass.h"
#include "textureclass.h"

#include <tchar.h>
#include "nvStereo.h"

using namespace std;


class Graphic 
{
public:

	Graphic();
	~Graphic();
	bool Initialize(HWND);
	void Render(float,float,float);


private:

	// NVAPI Stereo Handle
	StereoHandle                 g_StereoHandle;
	float                        g_EyeSeparation;
	float                        g_Separation;
	float                        g_Convergence;
	float						 g_CameraDistance;
	nv::StereoParametersD3D10    g_StereoParamD3D10;


	// Directx and Effect
	ID3D10Device*             g_D3D10Device;
	DXGI_SWAP_CHAIN_DESC      g_DXGISwapChainDesc;
	IDXGISwapChain*           g_DXGISwapChain;
	ID3D10RenderTargetView*   g_D3D10BackBufferRTV;
	ID3D10DepthStencilView*   g_D3D10DepthBufferDSV;
	D3D10_VIEWPORT            g_D3D10MainViewport;
	ID3D10ShaderResourceView* g_D3D10DepthBufferSRV;

	ID3D10Effect*               mFX_Box;
	ID3D10EffectTechnique*      mTech_Box;
	ID3D10EffectMatrixVariable* mfxWVPVar_Box;
	ID3D10EffectScalarVariable* mfxCameraDistance_Box;

	ID3D10Effect*               mFX_Sphere;
	ID3D10EffectTechnique*      mTech_Sphere;
	ID3D10EffectMatrixVariable* mfxWVPVar_Sphere;
	ID3D10EffectScalarVariable* mfxCameraDistance_Sphere;

	ID3D10InputLayout*          mVertexLayout;

	D3DXMATRIX mView;
	D3DXMATRIX mProj;
	D3DXMATRIX mWVP;


	TerrainClass terrain;
	ModelClass* m_Model;
	LightShaderClass* m_LightShader;
	LightClass* m_Light;

	ID3DX10Mesh* sphere;

	void       buildFX();
	void       buildVertexLayouts();

	bool       CreateDevice(HWND);
	void       FreeDevice();


	
	
};
