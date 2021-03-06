#include "stdafx.h"

#include "graphic.h"

const float BoxSize = 0.50f;
static const float particleRadius = 0.1f;

Graphic::Graphic()
{
	m_Model = 0;
	m_LightShader = 0;
	m_Light = 0;
}

Graphic::~Graphic()
{
	FreeDevice();
}


bool Graphic::Initialize(HWND g_hWnd)
{
	if (!CreateDevice(g_hWnd))
	{
		return false;
	}
}

void Graphic::buildFX()
{
	DWORD shaderFlags = D3D10_SHADER_ENABLE_STRICTNESS;
#if defined( DEBUG ) || defined( _DEBUG )
	shaderFlags |= D3D10_SHADER_DEBUG;
	shaderFlags |= D3D10_SHADER_SKIP_OPTIMIZATION;
#endif

	ID3D10Blob* compilationErrors = 0;
	HRESULT hr = 0;
	hr = D3DX10CreateEffectFromFile(L"color.fx", 0, 0,
		"fx_4_0", shaderFlags, 0, g_D3D10Device, 0, 0, &mFX_Box, &compilationErrors, 0);
	if (FAILED(hr))
	{
		if (compilationErrors)
		{
			MessageBoxA(0, (char*)compilationErrors->GetBufferPointer(), 0, 0);
			ReleaseCOM(compilationErrors);
		}
		DXTrace(__FILE__, (DWORD)__LINE__, hr, L"D3DX10CreateEffectFromFile", true);
	}

	mTech_Box = mFX_Box->GetTechniqueByName("ColorTech");
	mfxWVPVar_Box = mFX_Box->GetVariableByName("gWVP")->AsMatrix();
	mfxCameraDistance_Box = mFX_Box->GetVariableByName("cameraDepth")->AsScalar();


	hr = D3DX10CreateEffectFromFile(L"sphere.fx", 0, 0,
		"fx_4_0", shaderFlags, 0, g_D3D10Device, 0, 0, &mFX_Sphere, &compilationErrors, 0);
	if (FAILED(hr))
	{
		if (compilationErrors)
		{
			MessageBoxA(0, (char*)compilationErrors->GetBufferPointer(), 0, 0);
			ReleaseCOM(compilationErrors);
		}
		DXTrace(__FILE__, (DWORD)__LINE__, hr, L"D3DX10CreateEffectFromFile", true);
	}

	mTech_Sphere = mFX_Sphere->GetTechniqueByName("ColorTech");
	mfxWVPVar_Sphere = mFX_Sphere->GetVariableByName("gWVP")->AsMatrix();
	mfxCameraDistance_Sphere = mFX_Sphere->GetVariableByName("cameraDepth")->AsScalar();
}

void Graphic::buildVertexLayouts()
{

	D3D10_INPUT_ELEMENT_DESC vertexDesc[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D10_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D10_INPUT_PER_VERTEX_DATA, 0 }
	};

	// Create the input layout
	D3D10_PASS_DESC PassDesc;
	mTech_Sphere->GetPassByIndex(0)->GetDesc(&PassDesc);
	HR(g_D3D10Device->CreateInputLayout(vertexDesc, 2, PassDesc.pIAInputSignature,
		PassDesc.IAInputSignatureSize, &mVertexLayout));


	mTech_Box->GetPassByIndex(0)->GetDesc(&PassDesc);
	HR(g_D3D10Device->CreateInputLayout(vertexDesc, 2, PassDesc.pIAInputSignature,
		PassDesc.IAInputSignatureSize, &mVertexLayout));
}

bool Graphic::CreateDevice(HWND g_hWnd)
{
	D3DXMatrixIdentity(&mView);
	D3DXMatrixIdentity(&mProj);
	D3DXMatrixIdentity(&mWVP);

	DEVMODE devMode;

	if (!EnumDisplaySettings(NULL, ENUM_CURRENT_SETTINGS, &devMode))
		return false;

	HRESULT hr = S_OK;

	// Prepare NVAPI for use in this application
	NvAPI_Status status;
	status = NvAPI_Initialize();
	if (status != NVAPI_OK)
	{
		NvAPI_ShortString errorMessage;
		NvAPI_GetErrorMessage(status, errorMessage);
		MessageBoxA(NULL, errorMessage, "Unable to initialize NVAPI", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
	}
	else
	{
		// Check the Stereo availability
		NvU8 isStereoEnabled;
		status = NvAPI_Stereo_IsEnabled(&isStereoEnabled);

		// Stereo status report an error
		if (status != NVAPI_OK)
		{
			// GeForce Stereoscopic 3D driver is not installed on the system
			MessageBoxA(NULL, "Stereo is not available\nMake sure the stereo driver is installed correctly", "Stereo not available", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);

		}
		// Stereo is available but not enabled, let's enable it
		else if (NVAPI_OK == status && !isStereoEnabled)
		{
			MessageBoxA(NULL, "Stereo is available but not enabled\nLet's enable it", "Stereo not enabled", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
			status = NvAPI_Stereo_Enable();
		}

		NvAPI_Stereo_CreateConfigurationProfileRegistryKey(NVAPI_STEREO_DEFAULT_REGISTRY_PROFILE);
	}


	IDXGIAdapter* capableAdapter = NULL;
	{
		IDXGIFactory *pFactory;
		hr = CreateDXGIFactory(__uuidof(IDXGIFactory), (void**)(&pFactory));

		for (UINT adapter = 0; !capableAdapter; ++adapter)
		{
			// get a candidate DXGI adapter
			IDXGIAdapter* pAdapter = NULL;
			hr = pFactory->EnumAdapters(adapter, &pAdapter);
			if (FAILED(hr))
			{
				break;
			}
			// query to see if there exists a corresponding compute device

			// if so, mark it as the one against which to create our d3d10 device
			capableAdapter = pAdapter;
			capableAdapter->AddRef();

			pAdapter->Release();
		}
		pFactory->Release();
	}

	// Create device and swapchain
	ZeroMemory(&g_DXGISwapChainDesc, sizeof(g_DXGISwapChainDesc));
	g_DXGISwapChainDesc.BufferCount = 1;
	g_DXGISwapChainDesc.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
	g_DXGISwapChainDesc.BufferDesc.RefreshRate.Denominator = 1;
	g_DXGISwapChainDesc.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	g_DXGISwapChainDesc.OutputWindow = g_hWnd;
	g_DXGISwapChainDesc.SampleDesc.Count = 1;
	g_DXGISwapChainDesc.SampleDesc.Quality = 0;


	RECT rect;
	GetWindowRect(g_hWnd, &rect);

	g_DXGISwapChainDesc.Windowed = true;
	g_DXGISwapChainDesc.BufferDesc.RefreshRate.Numerator = devMode.dmDisplayFrequency;
	g_DXGISwapChainDesc.BufferDesc.Width = rect.right - rect.left;
	g_DXGISwapChainDesc.BufferDesc.Height = rect.bottom - rect.top;
	// Fullscreen mode mode
	//g_DXGISwapChainDesc.Windowed = FALSE;
	////g_DXGISwapChainDesc.BufferDesc.RefreshRate.Numerator = devMode.dmDisplayFrequency;
	//g_DXGISwapChainDesc.BufferDesc.Width = devMode.dmPelsWidth;
	//g_DXGISwapChainDesc.BufferDesc.Height = devMode.dmPelsHeight;

	hr = D3D10CreateDeviceAndSwapChain(
		capableAdapter,
		D3D10_DRIVER_TYPE_HARDWARE,
		NULL,
		0, //D3D10_CREATE_DEVICE_DEBUG,
		D3D10_SDK_VERSION,
		&g_DXGISwapChainDesc,
		&g_DXGISwapChain,
		&g_D3D10Device);

	capableAdapter->Release();


	// Create the RenderTarget View
	ID3D10Texture2D* lBuffer;
	g_DXGISwapChain->GetBuffer(0, __uuidof(ID3D10Texture2D), (LPVOID*)&lBuffer);
	if (FAILED(g_D3D10Device->CreateRenderTargetView(lBuffer,
		NULL,
		&g_D3D10BackBufferRTV)))
	{
		MessageBox(NULL, _T("Failed to create RenderTargetView for D3D10."), _T("Test App Error"), MB_SETFOREGROUND | MB_OK | MB_SYSTEMMODAL | MB_TOPMOST);
		return false;
	}
	lBuffer->Release();

	// Create a depthStencil buffer and view
	D3D10_TEXTURE2D_DESC texDesc;
	memset(&texDesc, 0, sizeof(D3D10_TEXTURE2D_DESC));
	texDesc.Width = g_DXGISwapChainDesc.BufferDesc.Width;
	texDesc.Height = g_DXGISwapChainDesc.BufferDesc.Height;
	texDesc.MipLevels = 1;
	texDesc.ArraySize = 1;
	texDesc.SampleDesc.Count = 1;
	texDesc.Usage = D3D10_USAGE_DEFAULT;
	texDesc.BindFlags = D3D10_BIND_DEPTH_STENCIL | D3D10_BIND_SHADER_RESOURCE;
	texDesc.Format = DXGI_FORMAT_R32_TYPELESS;

	ID3D10Texture2D* lSurface = 0;
	g_D3D10Device->CreateTexture2D(&texDesc, 0, &lSurface);

	D3D10_DEPTH_STENCIL_VIEW_DESC dsvDesc;
	memset(&dsvDesc, 0, sizeof(D3D10_DEPTH_STENCIL_VIEW_DESC));
	dsvDesc.Format = DXGI_FORMAT_D32_FLOAT;
	dsvDesc.ViewDimension = D3D10_DSV_DIMENSION_TEXTURE2D;
	dsvDesc.Texture2D.MipSlice = 0;
	g_D3D10Device->CreateDepthStencilView(lSurface, &dsvDesc, &g_D3D10DepthBufferDSV);

	D3D10_SHADER_RESOURCE_VIEW_DESC srvDesc;
	memset(&srvDesc, 0, sizeof(D3D10_SHADER_RESOURCE_VIEW_DESC));
	srvDesc.Format = DXGI_FORMAT_R32_FLOAT;
	srvDesc.ViewDimension = D3D10_SRV_DIMENSION_TEXTURE2D;
	srvDesc.Texture2D.MipLevels = 1;
	srvDesc.Texture2D.MostDetailedMip = 0;
	g_D3D10Device->CreateShaderResourceView(lSurface, &srvDesc, &g_D3D10DepthBufferSRV);

	lSurface->Release();

	// Setup the viewport
	g_D3D10MainViewport.Width = g_DXGISwapChainDesc.BufferDesc.Width;
	g_D3D10MainViewport.Height = g_DXGISwapChainDesc.BufferDesc.Height;
	g_D3D10MainViewport.MinDepth = 0.0f;
	g_D3D10MainViewport.MaxDepth = 1.0f;
	g_D3D10MainViewport.TopLeftX = 0;
	g_D3D10MainViewport.TopLeftY = 0;


	ID3D10RasterizerState* rs;
	D3D10_RASTERIZER_DESC rasterizerState;
	rasterizerState.FillMode = D3D10_FILL_SOLID;
	rasterizerState.CullMode = D3D10_CULL_NONE;
	rasterizerState.FrontCounterClockwise = false;
	rasterizerState.DepthBias = false;
	rasterizerState.DepthBiasClamp = 0;
	rasterizerState.SlopeScaledDepthBias = 0;
	rasterizerState.DepthClipEnable = false;
	rasterizerState.ScissorEnable = false;
	rasterizerState.MultisampleEnable = false;
	rasterizerState.AntialiasedLineEnable = false;
	g_D3D10Device->CreateRasterizerState(&rasterizerState, &rs);
	g_D3D10Device->RSSetState(rs);


	D3DXVECTOR3 pos(0.0f, -0.3f, -0.8f);
	D3DXVECTOR3 target(0.0f, 0.35f, 0.0f);
	D3DXVECTOR3 up(0.0f, 1.0f, 0.0f);
	D3DXMatrixLookAtLH(&mView, &pos, &target, &up);

	float Near = 2.f;
	float Far = 1000.0f;
	float fAspectRatio = (FLOAT)g_D3D10MainViewport.Width / (FLOAT)g_D3D10MainViewport.Height;
	D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI /4 , fAspectRatio, Near, Far);

	//float fAspectRatio = (FLOAT)g_D3D10MainViewport.Width / (FLOAT)g_D3D10MainViewport.Height;
	//D3DXMatrixPerspectiveFovLH(&mProj, D3DX_PI / 6, fAspectRatio, Near, Far);


	m_Model = new ModelClass();
	if (!m_Model)
	{
		return false;
	}
	bool result;
	result = m_Model->Initialize(g_D3D10Device, "data/cube.txt", L"data/WoodCrate01.dds");
	if (!result)
	{
		return false;
	}

	// Create the light shader object.
	m_LightShader = new LightShaderClass;
	if (!m_LightShader)
	{
		return false;
	}

	// Initialize the light shader object.
	result = m_LightShader->Initialize(g_D3D10Device, g_hWnd);
	if (!result)
	{
		MessageBox(g_hWnd, L"Could not initialize the light shader object.", L"Error", MB_OK);
		return false;
	}

	// Create the light object.
	m_Light = new LightClass;
	if (!m_Light)
	{
		return false;
	}

	// Initialize the light object.
	m_Light->SetDiffuseColor(1.0f, 1.0f, 1.0f, 1.0f);
	m_Light->SetDirection(0.0f, 0.0f, 1.0f);
	

	// Create 3D models that will use
	terrain.Initialize(g_D3D10Device);

	DXUTCreateSphere(g_D3D10Device, particleRadius, 10, 10, &sphere); 
	const D3D10_INPUT_ELEMENT_DESC* vDesc;
	UINT cnt = 0;
	sphere->GetVertexDescription(&vDesc, &cnt);
	sphere->CommitToDevice();


	// Simple FX and layout
	buildFX();
	buildVertexLayouts();

	return true;
}

void Graphic::FreeDevice()
{
	if (g_DXGISwapChain)
	{
		BOOL isFullScreen;
		g_DXGISwapChain->GetFullscreenState(&isFullScreen, 0);
		if (isFullScreen)
		{
			g_DXGISwapChain->SetFullscreenState(FALSE, 0);
		}
	}

	if (g_D3D10DepthBufferDSV)
	{
		g_D3D10DepthBufferDSV->Release();
		g_D3D10DepthBufferDSV = 0;
	}
	if (g_D3D10DepthBufferSRV)
	{
		g_D3D10DepthBufferSRV->Release();
		g_D3D10DepthBufferSRV = 0;
	}
	if (g_D3D10BackBufferRTV)
	{
		g_D3D10BackBufferRTV->Release();
		g_D3D10BackBufferRTV = 0;
	}
	if (g_DXGISwapChain)
	{
		g_DXGISwapChain->Release();
		g_DXGISwapChain = 0;
	}


	if (g_D3D10Device)
	{
		g_D3D10Device->Release();
		g_D3D10Device = 0;
	}
	// Release the light object.
	if (m_Light)
	{
		delete m_Light;
		m_Light = 0;
	}

	// Release the light shader object.
	if (m_LightShader)
	{
		m_LightShader->Shutdown();
		delete m_LightShader;
		m_LightShader = 0;
	}

	// Release the model object.
	if (m_Model)
	{
		m_Model->Shutdown();
		delete m_Model;
		m_Model = 0;
	}
	g_StereoParamD3D10.destroyGraphics();


	ReleaseCOM(mFX_Sphere);
	ReleaseCOM(mFX_Box);
	ReleaseCOM(mVertexLayout);
}

void Graphic::Render(float x, float y, float z)
{
	D3DXVECTOR3 pos(x,y,z );
	D3DXVECTOR3 target(0.0f, 0.0, 0.0f);
	D3DXVECTOR3 up(0.0f, 1.0f, 0.0f);
	D3DXMatrixLookAtLH(&mView, &pos, &target, &up);

	
	static unsigned int frameNb = 0;

	g_D3D10Device->RSSetViewports(1, &g_D3D10MainViewport);

	float color[] = { 0.0f, 0.0f, 0.0f, 1.f };
	g_D3D10Device->ClearRenderTargetView(g_D3D10BackBufferRTV, color);
	g_D3D10Device->ClearDepthStencilView(g_D3D10DepthBufferDSV, D3D10_CLEAR_DEPTH, 1, 0);

	g_D3D10Device->OMSetRenderTargets(1, &g_D3D10BackBufferRTV, g_D3D10DepthBufferDSV);

	float sep, conv;
	if (NVAPI_OK != NvAPI_Stereo_GetSeparation(g_StereoHandle, &sep))
	{
		//MessageBoxA(NULL, "Couldn't get the separation", "NvAPI_Stereo_GetSeparation failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);
	}
	if (NVAPI_OK != NvAPI_Stereo_GetConvergence(g_StereoHandle, &conv))
	{
		//MessageBoxA(NULL, "Couldn't get the convergence", "NvAPI_Stereo_GetConvergence failed", MB_OK | MB_SETFOREGROUND | MB_TOPMOST);

	}
	if (sep * 0.01 != g_Separation || conv != g_Convergence)
	{
		g_Separation = sep * 0.01;
		g_Convergence = conv;
		//g_pStereoSeparation->SetFloat(g_Separation);
		//g_pStereoConvergence->SetFloat(g_Convergence);

		g_StereoParamD3D10.updateStereoParamsMap(g_D3D10Device, g_EyeSeparation, g_Separation, g_Convergence);
	}

	D3D10_TECHNIQUE_DESC techDesc;
	D3DXMATRIX mat;
	D3DXMatrixTranslation(&mat, -50.0, 0.0f, -50.0);
	mWVP = mat* mView*mProj;
	mfxWVPVar_Box->SetMatrix((float*)&mWVP);
	mfxCameraDistance_Box->SetFloat(z);
	mfxCameraDistance_Sphere->SetFloat(z);
	g_CameraDistance = z;
	terrain.Render(g_D3D10Device);
	mTech_Box->GetDesc(&techDesc);
	for (UINT p = 0; p < techDesc.Passes; ++p)
	{
		mTech_Box->GetPassByIndex(p)->Apply(0);

		g_D3D10Device->DrawIndexed(terrain.GetIndexCount(), 0, 0);
	}

	g_D3D10Device->OMSetDepthStencilState(0, 0);
	float blendFactors[] = { 0.0f, 0.0f, 0.0f, 0.0f };
	g_D3D10Device->OMSetBlendState(0, blendFactors, 0xffffffff);
	g_D3D10Device->IASetInputLayout(mVertexLayout);
	g_D3D10Device->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);


	D3DXVECTOR4* out = new D3DXVECTOR4[48];
	float* handParameter = new float[26];
	for (int i = 0; i < 26; i++)
	{
		handParameter[i] = 0;
	}
	//handParameter[0] = 2;
	//handParameter[1] = 1;

	//handParameter[3] = D3DX_PI / 2;

	handParameter[22] = D3DX_PI/2;
	handParameter[23] = 0;
	handParameter[24] = D3DX_PI / 4;


	//HandEncoding(handParameter, out);
	for (int i = 0; i < 48; i++)
	{
		D3DXMatrixTranslation(&mat, out[i].x, out[i].y, out[i].z);
		mWVP = mat* mView*mProj;

		mfxWVPVar_Sphere->SetMatrix((float*)&mWVP);

		mTech_Sphere->GetDesc(&techDesc);
		for (UINT p = 0; p < techDesc.Passes; ++p)
		{
			mTech_Sphere->GetPassByIndex(p)->Apply(0);

			sphere->DrawSubset(0);
		}
	}

	// Stop drawing in the depth buffer so we can fetch from it
	g_D3D10Device->OMSetRenderTargets(1, &g_D3D10BackBufferRTV, 0);

	g_DXGISwapChain->Present(0, 0);

}


