#include <PglEncoder.h>
#include <PglCRC16.h>
#include "command_parser.h"
#include "render/frame_renderer.h"
#include "diagnostics/demo_scene.h"
#include "render/screenspace_effects.h"
#include <chrono>
#include <cstdio>
#include <cstring>
#include <limits>

namespace {
using R = PglRuntime::Result;
SceneState scene;
Rasterizer rasterizer;
PglTileScheduler scheduler;
uint16_t pixels[GpuConfig::FRAMEBUF_PIXELS];
PhaseScratch::DepthWorkspace depth;
uint8_t wire[PglRuntime::MaxBatchBytes];
int failures = 0, checks = 0;
constexpr PglQuat identity{1,0,0,0};
constexpr PglVec3 zero{0,0,0}, one{1,1,1};
const PglVec3 vertices[] = {{-8,-8,2},{8,-8,2},{-8,8,2}};
const PglIndex3 indices[] = {{0,1,2}};
const PglParamSimple red{255,0,0};
uint64_t Now() {
    return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
void Check(bool condition, const char* what) {
    ++checks;
    if (!condition) { ++failures; std::printf("FAIL: %s\n", what); }
}
void Fresh() {
    scene.renderWidth = scene.renderHeight = 32;
    scene.Reset(); CommandParser::ClearErrors();
}
R Parse(PglEncoder& encoder, bool resources = false) {
    CommandParser::BatchInfo info;
    return CommandParser::Parse(encoder.GetBuffer(), encoder.GetLength(), &scene, info, resources);
}
R Render() {
    FrameRenderer engine(rasterizer,scheduler); FrameTimings timing;
    return engine.Render(scene,pixels,depth,32,32,timing,Now,nullptr);
}
void Draw(PglEncoder& encoder, PglMesh mesh=0, PglMaterial material=0) {
    encoder.DrawObject(mesh,material,zero,identity,one,identity,identity,zero,zero,true);
}
size_t InjectUnknown(PglEncoder& encoder, bool resources) {
    const size_t length = encoder.GetLength(), insert = length - (resources ? 2 : 9);
    std::memmove(wire+insert+3,wire+insert,length-insert);
    wire[insert]=0xfe;wire[insert+1]=wire[insert+2]=0;
    PglRuntime::Store32(wire+6,uint32_t(length+3));
    PglRuntime::Store16(wire+10,PglRuntime::Load16(wire+10)+1);
    PglRuntime::Store16(wire+length+1,PglCRC16::Compute(wire,length+1));
    return length+3;
}
void TestAtomicAdmissionAndPixels() {
    Fresh(); PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);
    encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
    encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
    Draw(encoder);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok,"actual host mesh/material/camera batch admitted");
    Check(Render()==R::Ok && pixels[10*32+10]==0xf800 && pixels[27*32+27]==0,
          "encoded orthographic triangle reaches independently selected interior/background pixels");
    const auto used=scene.sceneHeap.stats().usedBytes;
    auto* oldVertices=scene.meshes[0].vertices;
    const auto elapsed=scene.elapsedTimeUs;
    encoder.BeginFrame(2,16666);encoder.CreateMesh(1,vertices,3,indices,1);encoder.EndFrame();
    const size_t corruptLength=InjectUnknown(encoder,false);CommandParser::BatchInfo info;
    Check(CommandParser::Parse(wire,corruptLength,&scene,info)==R::Unsupported,
          "late unsupported command rejects entire batch");
    Check(scene.frameNumber==1 && scene.elapsedTimeUs==elapsed && !scene.meshes[1].active &&
          scene.meshes[0].vertices==oldVertices && scene.sceneHeap.stats().usedBytes==used,
          "late rejection preserves previous scene and returns every reserved byte");
    Check(Render()==R::Ok && pixels[10*32+10]==0xf800,"rejected transaction leaves previous scene renderable");
    encoder.BeginFrame(2,16666);encoder.DestroyMesh(0);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok,"destroy admitted after earlier frame readers drain");
    scene.RetireResourceReads();
    encoder.BeginFrame(3,16666);encoder.CreateMesh(PglMakeHandle(1,0),vertices,3,indices,1);
    Draw(encoder,PglMakeHandle(1,0));encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[10*32+10]==0xf800,
          "recreated next-generation mesh is usable");
    encoder.BeginFrame(4,16666);Draw(encoder,0);encoder.EndFrame();
    Check(Parse(encoder)==R::InvalidHandle && scene.frameNumber==3,
          "stale generation cannot draw or change accepted frame");
    encoder.BeginFrame(4,16666);
    encoder.CreateMesh(PglMakeHandle(0xff,1),vertices,3,indices,1);encoder.EndFrame();
    Check(Parse(encoder)==R::InvalidHandle && scene.frameNumber==3 && !scene.meshes[1].active,
          "raw reserved generation255 cannot bypass host-side exhaustion policy");
}
void TestStreamsAndRollback() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    const uint16_t texels[4]={0xf800,0x07e0,0x001f,0xffff};
    uint8_t payload[sizeof(PglCmdCreateTextureHeader)+sizeof(texels)];
    const PglCmdCreateTextureHeader header{0,2,2,PGL_TEX_RGB565};
    std::memcpy(payload,&header,sizeof(header));std::memcpy(payload+sizeof(header),texels,sizeof(texels));
    encoder.BeginResourceBatch();
    encoder.BeginResourceStream(PGL_RES_CLASS_TEXTURE,0,0,sizeof(payload),PglRuntime::PayloadChecksum(payload,sizeof(payload)));
    encoder.StreamResourceData(PGL_RES_CLASS_TEXTURE,0,0,payload,10);encoder.EndResourceBatch();
    Check(Parse(encoder,true)==R::Ok && !scene.textures[0].active,"incomplete streamed texture is not visible");
    const auto used=scene.sceneHeap.stats().usedBytes;
    encoder.BeginResourceBatch();encoder.StreamResourceData(PGL_RES_CLASS_TEXTURE,0,10,payload+10,sizeof(payload)-10);
    encoder.EndResourceBatch();const size_t badLength=InjectUnknown(encoder,true);CommandParser::BatchInfo info;
    Check(CommandParser::Parse(wire,badLength,&scene,info,true)==R::Unsupported && scene.upload.received==10 &&
          scene.sceneHeap.stats().usedBytes==used,"failed append batch preserves cursor and storage ownership");
    encoder.BeginResourceBatch();encoder.StreamResourceData(PGL_RES_CLASS_TEXTURE,0,10,payload+10,sizeof(payload)-10);
    encoder.CommitResourceStream(PGL_RES_CLASS_TEXTURE,0);encoder.EndResourceBatch();
    Check(Parse(encoder,true)==R::Ok && !scene.upload.active && scene.textures[0].active &&
          std::memcmp(scene.textures[0].pixels,texels,sizeof(texels))==0,"retry append and commit publish exact texture bytes once");
    encoder.BeginFrame(1,16666);encoder.DrawSprite(0,4,4,0);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[4*32+4]==0xf800 &&
          pixels[4*32+5]==0x07e0 && pixels[5*32+4]==0x001f && pixels[5*32+5]==0xffff,
          "streamed texture is consumed by the real primary-target sprite renderer");
}
void TestPrimaryClearsAndEmptyFrames() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.LayerClear(0,0x001f);encoder.DrawRect2D(0,3,4,2,2,0xf800);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[0]==0x001f && pixels[4*32+3]==0xf800,
          "primary 2D operations render without any camera");
    encoder.BeginFrame(2,16666);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok,"empty successor frame admitted");
    bool black=true;for(size_t i=0;i<32*32;++i)black &= pixels[i]==0;
    Check(black,"empty frame cannot reuse stale primary pixels");
}
void TestCandidateDestructionAndMultipleAdoptions() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
    encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
    Draw(encoder);encoder.DestroyMesh(0);encoder.DestroyMaterial(0);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[10*32+10]==0xf800,
          "create draw destroy retains the candidate until the real frame consumes it");
    scene.RetireResourceReads();
    Check(!scene.meshes[0].active && !scene.materials[0].active && scene.sceneHeap.stats().usedBytes==0,
          "last source release retires newly created and destroyed candidates exactly once");
    Fresh();
    const uint16_t first[4]={0xf800,0x07e0,0x001f,0xffff}, second[4]={0x07e0,0x001f,0xffff,0xf800};
    uint8_t p0[sizeof(PglCmdCreateTextureHeader)+sizeof(first)],p1[sizeof(p0)];
    const PglCmdCreateTextureHeader h0{0,2,2,PGL_TEX_RGB565},h1{1,2,2,PGL_TEX_RGB565};
    std::memcpy(p0,&h0,sizeof(h0));std::memcpy(p0+sizeof(h0),first,sizeof(first));
    std::memcpy(p1,&h1,sizeof(h1));std::memcpy(p1+sizeof(h1),second,sizeof(second));
    encoder.BeginResourceBatch();
    encoder.BeginResourceStream(PGL_RES_CLASS_TEXTURE,0,0,sizeof(p0),PglRuntime::PayloadChecksum(p0,sizeof(p0)));
    encoder.StreamResourceData(PGL_RES_CLASS_TEXTURE,0,0,p0,sizeof(p0));encoder.EndResourceBatch();
    Check(Parse(encoder,true)==R::Ok,"first complete upload remains staged before commit");
    encoder.BeginResourceBatch();encoder.CommitResourceStream(PGL_RES_CLASS_TEXTURE,0);
    encoder.BeginResourceStream(PGL_RES_CLASS_TEXTURE,1,0,sizeof(p1),PglRuntime::PayloadChecksum(p1,sizeof(p1)));
    encoder.StreamResourceData(PGL_RES_CLASS_TEXTURE,1,0,p1,sizeof(p1));encoder.CommitResourceStream(PGL_RES_CLASS_TEXTURE,1);
    encoder.EndResourceBatch();Check(Parse(encoder,true)==R::Ok,"two upload adoptions commit in one resource batch");
    void* pressure[2048];size_t pressureCount=0;
    while(pressureCount<2048) {
        void* allocation=scene.SceneHeapAlloc(32);if(!allocation)break;
        pressure[pressureCount++]=allocation;std::memset(allocation,0xa5,32);
    }
    encoder.BeginFrame(1,16666);encoder.DrawSprite(0,4,4,0);encoder.DrawSprite(0,8,4,1);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[4*32+4]==0xf800 &&
          pixels[4*32+8]==0x07e0 && pixels[5*32+5]==0xffff,
          "both adopted textures survive subsequent allocations and real sampling");
    for(size_t i=0;i<pressureCount;++i)scene.SceneHeapFree(pressure[i]);
    scene.Reset();
}
struct Backing {
    uint8_t bytes[8192] = {};
    bool failReads = false;
    static bool Read(void* context,uint32_t offset,void* destination,uint32_t count) {
        auto& self=*static_cast<Backing*>(context);
        if(self.failReads)return false;
        std::memcpy(destination,self.bytes+offset,count);return true;
    }
    static bool Write(void* context,uint32_t offset,const void* source,uint32_t count) {
        auto& self=*static_cast<Backing*>(context);
        std::memcpy(self.bytes+offset,source,count);return true;
    }
};
void TestRealColdConsumers() {
    Fresh();Backing backing;gpumem::AssetService assets;alignas(16) uint8_t staging[128];
    gpumem::AssetBacking store{&backing,sizeof(backing.bytes),Backing::Read,Backing::Write};
    Check(assets.bind(store,staging,48)==gpumem::AssetStatus::Ok,"bounded external byte store binds");
    scene.externalAssets=&assets;
    auto upload=[&](uint8_t resourceClass,const void* payload,size_t count) {
        PglEncoder encoder(wire,sizeof(wire));encoder.BeginResourceBatch();
        encoder.BeginResourceStream(resourceClass,0,1,count,PglRuntime::PayloadChecksum(static_cast<const uint8_t*>(payload),count));
        encoder.StreamResourceData(resourceClass,0,0,payload,count);encoder.CommitResourceStream(resourceClass,0);
        encoder.EndResourceBatch();return Parse(encoder,true);
    };
    uint8_t meshPayload[sizeof(PglCmdCreateMeshHeader)+sizeof(vertices)+sizeof(indices)];
    const PglCmdCreateMeshHeader mh{0,3,1,0};
    std::memcpy(meshPayload,&mh,sizeof(mh));std::memcpy(meshPayload+sizeof(mh),vertices,sizeof(vertices));
    std::memcpy(meshPayload+sizeof(mh)+sizeof(vertices),indices,sizeof(indices));
    const uint16_t texels[4]={0xf800,0x07e0,0x001f,0xffff};
    uint8_t texturePayload[sizeof(PglCmdCreateTextureHeader)+sizeof(texels)];
    const PglCmdCreateTextureHeader th{0,2,2,PGL_TEX_RGB565};
    std::memcpy(texturePayload,&th,sizeof(th));std::memcpy(texturePayload+sizeof(th),texels,sizeof(texels));
    auto submit=[&] {
        PglEncoder encoder(wire,sizeof(wire));encoder.BeginFrame(1,16666);
        encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
        encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
        Draw(encoder);encoder.DrawSprite(0,4,4,0);encoder.EndFrame();return Parse(encoder);
    };
    Check(upload(PGL_RES_CLASS_MESH,meshPayload,sizeof(meshPayload))==R::Ok &&
          scene.meshes[0].externalToken==0 && !scene.meshes[0].vertices,
          "cold mesh uses valid token zero without pretending to be an SRAM pointer");
    Check(upload(PGL_RES_CLASS_TEXTURE,texturePayload,sizeof(texturePayload))==R::Ok &&
          !scene.textures[0].pixels,"cold texture remains nonresident before frame prefetch");
    Check(submit()==R::Ok,"frame with cold mesh and sprite references admitted");
    for(auto& pixel:pixels)pixel=0xa55a;
    Check(Render()==R::Capacity && assets.stats().leasedSpanCount==0 &&
          !scene.meshes[0].vertices && pixels[10*32+10]==0xa55a,
          "insufficient complete-span staging fails before target writes and releases earlier pins");
    scene.Reset();Check(assets.stats().assetCount==0,"scene reset retires every external resource");
    Check(assets.bind(store,staging,sizeof(staging))==gpumem::AssetStatus::Ok,"larger staging arena binds");
    Check(upload(PGL_RES_CLASS_MESH,meshPayload,sizeof(meshPayload))==R::Ok &&
          upload(PGL_RES_CLASS_TEXTURE,texturePayload,sizeof(texturePayload))==R::Ok && submit()==R::Ok,
          "cold scene reconstructed after reset");
    Check(Render()==R::Ok && pixels[10*32+10]==0xf800 && pixels[4*32+5]==0x07e0 &&
          pixels[5*32+4]==0x001f && pixels[5*32+5]==0xffff,
          "actual rasterizer and 2D sampler consume exact leased cold mesh and texture bytes");
    Check(assets.destroy(gpumem::assetHandleFromToken(scene.meshes[0].externalToken))==gpumem::AssetStatus::LeaseActive,
          "display conversion lifetime keeps cold resources pinned against destruction");
    scene.RetireResourceReads();
    uint32_t freed=0;
    Check(assets.evictCached(sizeof(staging),&freed)==gpumem::AssetStatus::Ok,
          "released render spans can be evicted completely");
    backing.failReads=true;for(auto& pixel:pixels)pixel=0xa55a;
    Check(Render()==R::Io && pixels[10*32+10]==0xa55a && assets.stats().leasedSpanCount==0,
          "backing failure is explicit without partial target publication or stale-pointer fallback");
    scene.Reset();scene.externalAssets=nullptr;
}
void TestLayerOnlyCameraAndDiagnostic() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
    encoder.LayerCreate(1,32,32,PGL_PIXFMT_RGB565,PGL_LAYER_BLEND_ALPHA,255);
    encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
    encoder.SetCameraTarget(0,1,0,0,32,32,0);Draw(encoder);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[10*32+10]==0xf800,
          "layer-only camera is executed without any back-buffer camera");
    Fresh();CommandParser::BatchInfo info;
    size_t length=GpuDiagnostic::EncodeFrame(wire,sizeof(wire),1,33333);
    Check(CommandParser::Parse(wire,length,&scene,info)==R::Ok && Render()==R::Ok,
          "diagnostic scene uses the actual parser and two-worker frame engine");
    const uint16_t center=pixels[16*32+16];
    Check(center!=0 && pixels[0]==0,"diagnostic lit cube has a visible center and black background");
    length=GpuDiagnostic::EncodeFrame(wire,sizeof(wire),2,33333);
    scene.RetireResourceReads();scene.RetireFrameData();
    Check(CommandParser::Parse(wire,length,&scene,info)==R::Ok && Render()==R::Ok &&
          pixels[16*32+16]!=0,"diagnostic later frame reuses resident resources");
}
uint8_t concurrentWire[64];
size_t concurrentLength = 0;
unsigned concurrentAttempts = 0;
R concurrentResult = R::Ok;
void ParseDuringPreparation() {
    ++concurrentAttempts;
    CommandParser::BatchInfo info;
    concurrentResult = CommandParser::Parse(concurrentWire,concurrentLength,&scene,info);
}
void TestCompactDrawBoundaryAndPhaseExclusion() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    const PglVec3 morphed[]={{-2,-2,2},{2,-2,2},{-2,2,2}};
    constexpr PglQuat quarterTurn{0.7071067812f,0,0,0.7071067812f};
    auto draws=[&](uint32_t frame) {
        encoder.BeginFrame(frame,16666);
        for(unsigned i=0;i<GpuConfig::MAX_DRAW_CALLS-1;++i)
            encoder.DrawObject(0,0,{100,100,0},identity,one,identity,identity,zero,zero,true);
        encoder.DrawObjectMorphed(0,0,{4,3,0},quarterTurn,{2,1,1},quarterTurn,quarterTurn,
                                 {1,0,0},{0,1,0},true,morphed,3);
    };
    encoder.BeginResourceBatch();encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
    encoder.EndResourceBatch();
    Check(Parse(encoder,true)==R::Ok,"boundary scene resources admitted separately");
    draws(1);encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);encoder.EndFrame();
    // Independent transform: final world vertices (1,11),(1,3),(5,11),
    // hence panel vertices (17,27),(17,19),(21,27).
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[25*32+18]==0xf800 &&
          pixels[20*32+19]==0 && pixels[10*32+10]==0,
          "last of64 compact records consumes morph vertices and all transform fields");
    const auto used=scene.sceneHeap.stats().usedBytes;
    draws(2);encoder.EndFrame();const size_t rejected=InjectUnknown(encoder,false);
    CommandParser::BatchInfo info;
    Check(CommandParser::Parse(wire,rejected,&scene,info)==R::Unsupported &&
          scene.frameNumber==1 && scene.sceneHeap.stats().usedBytes==used &&
          Render()==R::Ok && pixels[25*32+18]==0xf800,
          "late rejection returns candidate override while preserving accepted morph readers");
    draws(2);Draw(encoder);encoder.EndFrame();
    Check(Parse(encoder)!=R::Ok && scene.frameNumber==1 &&
          scene.sceneHeap.stats().usedBytes==used && Render()==R::Ok &&
          pixels[25*32+18]==0xf800,"65th draw rejects atomically without truncating previous frame");
    PglEncoder concurrent(concurrentWire,sizeof(concurrentWire));
    concurrent.BeginFrame(99,16666);concurrent.EndFrame();concurrentLength=concurrent.GetLength();
    concurrentAttempts=0;concurrentResult=R::Ok;
    rasterizer.Initialize(&scene,depth,32,32);
    rasterizer.SetServiceCallback(ParseDuringPreparation);
    rasterizer.PrepareFrame(&scene);
    rasterizer.SetServiceCallback(nullptr);
    Check(concurrentAttempts==1 && concurrentResult==R::Busy && scene.frameNumber==1 &&
          Render()==R::Ok && pixels[25*32+18]==0xf800,
          "service callback cannot overwrite aliased preparation/parser scratch or live geometry");
    Check(CommandParser::Parse(concurrentWire,concurrentLength,&scene,info)==R::Ok &&
          Render()==R::Ok && pixels[25*32+18]==0,
          "retired preparation admits the next frame and clears its predecessor");
}
void TestPrimaryClipAndFiniteBuiltinAdmission() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.SetClipRect(0,6,7,3,3);
    encoder.SetViewport(0,5,6,512,512);encoder.DrawRect2D(0,0,0,4,4,0x07e0);
    encoder.EndFrame();
    Check(Parse(encoder)==R::Ok && Render()==R::Ok && pixels[7*32+6]==0x07e0 &&
          pixels[9*32+8]==0x07e0 && pixels[7*32+5]==0 && pixels[10*32+8]==0,
          "primary-target viewport scaling and clip intersect in physical target pixels");
    const auto used=scene.sceneHeap.stats().usedBytes;
    encoder.BeginFrame(2,16666);encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
    encoder.SetHorizontalBlur(0,0,std::numeric_limits<float>::quiet_NaN(),3);encoder.EndFrame();
    Check(Parse(encoder)==R::InvalidValue && scene.frameNumber==1 && !scene.meshes[0].active &&
          scene.sceneHeap.stats().usedBytes==used && Render()==R::Ok &&
          pixels[7*32+6]==0x07e0,"nonfinite builtin shader rejects all resource and frame mutation");
}
void TestTwoCameraFrameMetrics() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.CreateMesh(0,vertices,3,indices,1);
    encoder.CreateMaterial(0,PGL_MAT_SIMPLE,PGL_BLEND_BASE,&red,sizeof(red));
    for(uint8_t camera=0;camera<2;++camera) {
        encoder.SetCamera(camera,0,zero,identity,one,identity,identity,true);
        encoder.SetCameraTarget(camera,0,uint16_t(camera*16),0,16,32,PGL_CAMERA_TARGET_SCISSOR);
    }
    Draw(encoder);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok,"two-camera metric scene admitted");
    FrameRenderer engine(rasterizer,scheduler);FrameTimings timings;
    Check(engine.Render(scene,pixels,depth,32,32,timings,Now,nullptr)==R::Ok &&
          pixels[10*32+10]==0xf800 && pixels[10*32+18]==0xf800 && timings.triangles==2,
          "two visible camera passes report two source triangles, not cumulative-prefix sums");
}
void TestShaderJobFailureAndResume() {
    Fresh();PglEncoder encoder(wire,sizeof(wire));
    encoder.BeginFrame(1,16666);encoder.SetCamera(0,0,zero,identity,one,identity,identity,true);
    encoder.SetHorizontalBlur(0,0,1.0f,1);encoder.EndFrame();
    Check(Parse(encoder)==R::Ok,"finite shader scene admitted for maintenance-boundary exercise");
    for(unsigned y=0;y<32;++y)for(unsigned x=0;x<32;++x)
        pixels[y*32+x]=(x&1)?0xf000:0x001e;
    Check(scheduler.EnterMaintenance()==PglSchedResult::Ok,"live shader worker parks before maintenance");
    auto apply=[&] {
        scene.shaderFrameOps=0;
        return ScreenspaceShaders::ApplyShaderSlots(&scene,scene.cameras[0].shaders,
            PGL_MAX_SHADERS_PER_CAMERA,pixels,32,32,32,0,0,32,32,
            depth.BeginDepth(),GpuConfig::FRAMEBUF_PIXELS,0.0f,nullptr);
    };
    Check(apply()==R::FrameFailed && pixels[5*32+3]==0xf000,
          "rejected shader job cannot report successful effect completion");
    scheduler.ExitMaintenance();
    // Exact box mean: red30/3=10, blue60/3=20; no rounding tie.
    Check(apply()==R::Ok && pixels[5*32+3]==0x5014,
          "resumed two-band shader consumes the exact neighbour snapshot");
}
}
int main() {
    if(!scene.InitSceneHeap())return 2;
    scheduler.Initialize();if(!scheduler.StartWorker())return 2;
    TestAtomicAdmissionAndPixels();TestStreamsAndRollback();TestPrimaryClearsAndEmptyFrames();
    TestRealColdConsumers();
    TestCandidateDestructionAndMultipleAdoptions();
    TestLayerOnlyCameraAndDiagnostic();
    TestCompactDrawBoundaryAndPhaseExclusion();
    TestPrimaryClipAndFiniteBuiltinAdmission();
    TestTwoCameraFrameMetrics();
    TestShaderJobFailureAndResume();
    scheduler.Shutdown();scene.Reset();
    std::printf("command/render pipeline: %d checks, %d failures\n",checks,failures);
    return failures?1:0;
}
