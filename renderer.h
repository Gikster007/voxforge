#pragma once

#include "light.h"
#include "skydome/skydome.h"
#include "BVH/bvh.h"

namespace Tmpl8
{

class Renderer : public TheApp
{
public:
	// game flow methods
	void Init();
	float3 Trace( Ray& ray );
	void Tick( float deltaTime );
	void UI();
	void Shutdown();
	// input handling
	void MouseUp( int ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int ) 
	{ 
		if (ImGui::GetIO().WantCaptureMouse)
            return;

		Ray ray = camera.GetPrimaryRay(float(mousePos.x), float(mousePos.y));
	    bvh.intersect_bvh(voxel_objects, ray, 0);

	}
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int ) { /* implement if you want to handle keys */ }
	void KeyDown( int ) { /* implement if you want to handle keys */ }
	// data members
	int2 mousePos;
	float4* accumulator;
	Scene scene;
	Camera camera;

private:
    int frames = 0;

    float3 sun_pos = float3(0.0f, 5.0f, 0.0f);
	float time = 0.0f;
    std::vector<Light> lights;
    
	Skydome skydome;

	VoxelVolume* voxel_objects = nullptr;
    BVH bvh;

	bool grid_view = false;
};

} // namespace Tmpl8

