#include "frame_renderer.h"
#include "rasterizer_2d.h"
#include "screenspace_effects.h"
#include <cstring>

using R = PglRuntime::Result;

R FrameRenderer::DrawLayers(SceneState& scene, uint16_t* color, uint16_t width, uint16_t height,
                            bool clears, void (*service)()) {
    for (uint16_t i = 0; i < scene.drawCmd2DCount; ++i) {
        const auto& command = scene.drawCmds2D[i];
        if ((command.type == DRAW_CMD_2D_CLEAR) != clears) continue;
        Rasterizer2D::Target target;
        if (!command.layerId) { target.pixels = color; target.width = width; target.height = height; }
        else {
            if (command.layerId >= GpuConfig::MAX_LAYERS || !scene.layers[command.layerId].active) return R::InvalidHandle;
            const auto& layer = scene.layers[command.layerId];
            target.pixels = layer.pixels; target.width = layer.width; target.height = layer.height;
        }
        target.stride = target.width;
        target.clipX = command.clipX; target.clipY = command.clipY; target.clipW = command.clipW; target.clipH = command.clipH;
        target.viewOffsetX = command.viewOffsetX; target.viewOffsetY = command.viewOffsetY;
        target.viewScaleXQ8 = command.viewScaleXQ8; target.viewScaleYQ8 = command.viewScaleYQ8;
        switch (command.type) {
            case DRAW_CMD_2D_CLEAR: Rasterizer2D::Clear(target, command.clear.color, service); break;
            case DRAW_CMD_2D_RECT: {
                const auto& p = command.rect; Rasterizer2D::DrawRect(target,p.x,p.y,p.w,p.h,p.color,p.filled,service); break;
            }
            case DRAW_CMD_2D_LINE: {
                const auto& p = command.line; Rasterizer2D::DrawLine(target,p.x0,p.y0,p.x1,p.y1,p.color); break;
            }
            case DRAW_CMD_2D_CIRCLE: {
                const auto& p=command.circle; Rasterizer2D::DrawCircle(target,p.cx,p.cy,p.radius,p.color,p.filled,service); break;
            }
            case DRAW_CMD_2D_ROUNDED_RECT: {
                const auto& p=command.roundedRect; Rasterizer2D::DrawRoundedRect(target,p.x,p.y,p.w,p.h,p.radius,p.color,p.filled,service); break;
            }
            case DRAW_CMD_2D_ARC: {
                const auto& p=command.arc; Rasterizer2D::DrawArc(target,p.cx,p.cy,p.radius,p.startAngleDeg,p.endAngleDeg,p.color); break;
            }
            case DRAW_CMD_2D_TRIANGLE: {
                const auto& p=command.triangle; Rasterizer2D::DrawTriangle(target,p.x0,p.y0,p.x1,p.y1,p.x2,p.y2,p.color,service); break;
            }
            case DRAW_CMD_2D_GRADIENT_RECT: {
                const auto& p=command.gradient; Rasterizer2D::DrawGradientRect(target,p.x,p.y,p.w,p.h,p.color0,p.color1,p.direction,service); break;
            }
            case DRAW_CMD_2D_SPRITE:
            case DRAW_CMD_2D_SPRITE_BATCH: {
                uint16_t handle=command.type==DRAW_CMD_2D_SPRITE?command.sprite.textureId:command.spriteBatch.textureId;
                uint8_t flags=command.type==DRAW_CMD_2D_SPRITE?command.sprite.flags:command.spriteBatch.flags;
                const uint8_t index=PglHandleIndex(handle);
                if(index>=GpuConfig::MAX_TEXTURES || !scene.textures[index].active || scene.textureGeneration[index]!=PglHandleGeneration(handle))return R::InvalidHandle;
                const auto& texture=scene.textures[index];
                Rasterizer2D::SpriteSource source{texture.pixels,texture.pixelDataSize,texture.width,texture.height,texture.width,uint8_t(texture.format)};
                if(command.type==DRAW_CMD_2D_SPRITE)Rasterizer2D::DrawSprite(target,command.sprite.x,command.sprite.y,source,flags&PGL_SPRITE_FLIP_H,flags&PGL_SPRITE_FLIP_V,service);
                else {
                    if(uint32_t(command.spriteBatch.posOffset)+command.spriteBatch.count>scene.spritePosPool2D.used)return R::BadPacket;
                    Rasterizer2D::DrawSpriteBatch(target,source,scene.spritePosPool2D.data+command.spriteBatch.posOffset,command.spriteBatch.count,flags&PGL_SPRITE_FLIP_H,flags&PGL_SPRITE_FLIP_V,service);
                }
                break;
            }
            case DRAW_CMD_2D_TEXT: {
                const auto& p=command.text;uint8_t index=PglHandleIndex(p.fontTextureId);
                if(index>=GpuConfig::MAX_TEXTURES || !scene.textures[index].active || scene.textureGeneration[index]!=PglHandleGeneration(p.fontTextureId))return R::InvalidHandle;
                const auto& texture=scene.textures[index];
                if(texture.format!=PGL_TEX_RGB565)return R::InvalidValue;
                Rasterizer2D::GlyphAtlas atlas{reinterpret_cast<const uint16_t*>(texture.pixels),texture.width,texture.height,p.glyphW,p.glyphH,p.columns,p.firstChar};
                if(Rasterizer2D::DrawText(target,p.x,p.y,atlas,p.bytes,p.textLength,p.color,service)<0)return R::InvalidValue;
                break;
            }
            default:return R::Unsupported;
        }
        if(service)service();
    }
    return R::Ok;
}

void FrameRenderer::Composite(SceneState& scene,uint16_t* color,uint16_t width,uint16_t height,void(*service)()) {
    for(uint8_t index=1;index<GpuConfig::MAX_LAYERS;++index) {
        const auto& layer=scene.layers[index];
        if(!layer.active||!layer.visible||!layer.opacity||!layer.pixels)continue;
        int32_t x0=layer.offsetX<0?0:layer.offsetX,y0=layer.offsetY<0?0:layer.offsetY;
        int32_t x1=int32_t(layer.offsetX)+layer.width,y1=int32_t(layer.offsetY)+layer.height;
        if(x1>width)x1=width;
        if(y1>height)y1=height;
        for(int32_t y=y0;y<y1;++y) {
            const size_t srcRow=size_t(y-layer.offsetY)*layer.width,dstRow=size_t(y)*width;
            for(int32_t x=x0;x<x1;++x)color[dstRow+x]=Rasterizer2D::CompositeLayerPixel(layer.pixels[srcRow+x-layer.offsetX],color[dstRow+x],layer.blendMode,layer.opacity);
            if(service && ((y-y0)&15)==0)service();
        }
    }
}

R FrameRenderer::Render(SceneState& scene,uint16_t* color,PhaseScratch::DepthWorkspace& workspace,uint16_t width,uint16_t height,
                        FrameTimings& timings,uint64_t(*nowUs)(),void(*service)()) {
    if(!color||!nowUs||!TileConfig::ExtentValid(width,height)||uint32_t(width)*height>GpuConfig::FRAMEBUF_PIXELS)return R::InvalidValue;
    timings={};scene.shaderFrameOps=0;
    R pinned=scene.PinFrameAssets();if(pinned!=R::Ok)return pinned;
    for(uint16_t y=0;y<height;++y){std::memset(color+size_t(y)*width,0,size_t(width)*2);if(service && (y&15)==0)service();}
    R result=DrawLayers(scene,color,width,height,true,service);if(result!=R::Ok)return result;
    rasterizer_.Initialize(&scene,workspace,width,height);rasterizer_.SetServiceCallback(service);
    rasterizer_.SetElapsedTime(float(scene.elapsedTimeUs)*0.000001f);
    uint64_t start=nowUs();rasterizer_.PrepareFrame(&scene);timings.prepareUs+=uint32_t(nowUs()-start);
    if(rasterizer_.GetFrameError()!=R::Ok)return rasterizer_.GetFrameError();
    int8_t camera=rasterizer_.GetPreparedCameraIndex();
    if(camera<0) {
        uint8_t first=0;start=nowUs();
        if(rasterizer_.PrepareNextCameraPass(&scene,&first))camera=int8_t(first);
        timings.prepareUs+=uint32_t(nowUs()-start);
        if(rasterizer_.GetFrameError()!=R::Ok)return rasterizer_.GetFrameError();
    }
    while(camera>=0) {
        timings.triangles=rasterizer_.GetTriangleCount();
        uint16_t* depth=workspace.DepthPixels();
        const auto target=scene.ResolveCameraTarget(uint8_t(camera),color,width,height);
        if(!target.valid||!target.fb)return R::InvalidHandle;
        start=nowUs();
        auto dispatched=scheduler_.DispatchTilePass(&rasterizer_,target.fb,depth,target.width,target.height,service);
        timings.rasterUs+=uint32_t(nowUs()-start);
        if(dispatched!=PglSchedResult::Ok)return R::FrameFailed;
        if(rasterizer_.GetFrameError()!=R::Ok)return rasterizer_.GetFrameError();
        start=nowUs();
        result=ScreenspaceShaders::ApplyShaderSlots(&scene,scene.cameras[camera].shaders,PGL_MAX_SHADERS_PER_CAMERA,target.fb,target.width,target.height,target.width,
            target.scX0,target.scY0,target.scX1,target.scY1,depth,GpuConfig::FRAMEBUF_PIXELS,float(scene.elapsedTimeUs)*0.000001f,service);
        timings.effectsUs+=uint32_t(nowUs()-start);if(result!=R::Ok)return result;
        uint8_t next=0;start=nowUs();bool more=rasterizer_.PrepareNextCameraPass(&scene,&next);timings.prepareUs+=uint32_t(nowUs()-start);
        if(rasterizer_.GetFrameError()!=R::Ok)return rasterizer_.GetFrameError();
        camera=more?int8_t(next):-1;
    }
    start=nowUs();result=DrawLayers(scene,color,width,height,false,service);if(result!=R::Ok)return result;
    for(uint8_t index=1;index<GpuConfig::MAX_LAYERS;++index) {
        auto& layer=scene.layers[index];if(!layer.active||!layer.pixels)continue;
        result=ScreenspaceShaders::ApplyShaderSlots(&scene,layer.shaders,PGL_MAX_SHADERS_PER_CAMERA,layer.pixels,layer.width,layer.height,layer.width,0,0,layer.width,layer.height,
            workspace.DepthPixels(),GpuConfig::FRAMEBUF_PIXELS,float(scene.elapsedTimeUs)*0.000001f,service);
        if(result!=R::Ok)return result;
    }
    Composite(scene,color,width,height,service);timings.layersUs=uint32_t(nowUs()-start);
    return R::Ok;
}
