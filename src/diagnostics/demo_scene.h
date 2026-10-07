#pragma once
#include <PglEncoder.h>
#include <cmath>

namespace GpuDiagnostic {
// The diagnostic submits the same wire contract as a host. It owns no second
// scene, renderer, output driver or raster path; installed builds stay at the
// normal 150 MHz profile, and a fresh real host session takes over immediately.
inline size_t EncodeFrame(uint8_t* buffer,size_t capacity,uint32_t frame,uint32_t elapsedUs) {
    static constexpr PglVec3 vertices[]={{-.5f,-.5f,.5f},{.5f,-.5f,.5f},{.5f,.5f,.5f},{-.5f,.5f,.5f},
        {-.5f,-.5f,-.5f},{.5f,-.5f,-.5f},{.5f,.5f,-.5f},{-.5f,.5f,-.5f}};
    static constexpr PglIndex3 faces[]={{0,1,2},{0,2,3},{5,4,7},{5,7,6},{1,5,6},{1,6,2},
        {4,0,3},{4,3,7},{3,2,6},{3,6,7},{4,5,1},{4,1,0}};
    constexpr PglQuat identity{1,0,0,0};constexpr PglVec3 zero{0,0,0},one{1,1,1};
    PglEncoder encoder(buffer,capacity);encoder.BeginFrame(frame,elapsedUs);
    if(frame==1) {
        encoder.CreateMesh(0,vertices,8,faces,12);
        const PglParamLight light{0,0,-1,40,30,10,255,180,50};
        encoder.CreateMaterial(0,PGL_MAT_LIGHT,PGL_BLEND_BASE,&light,sizeof(light));
        encoder.SetCamera(0,0,{0,0,-5},identity,one,identity,identity,false);
    }
    const float angle=float((frame*5u)%360)*0.0174532925199433f;
    const PglQuat rotation{std::cos(angle*.5f),0,std::sin(angle*.5f),0};
    encoder.DrawObject(0,0,zero,rotation,{2.5f,2.5f,2.5f},identity,identity,zero,zero,true);
    encoder.EndFrame();
    return encoder.HasOverflow()||encoder.HasInvalidCommand()?0:encoder.GetLength();
}
} // namespace GpuDiagnostic
