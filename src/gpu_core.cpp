#include "gpu_core.h"
#include "gpu_config.h"
#include "command_parser.h"
#include "gpu_clock.h"
#include "hardware_resources.h"
#include "display/display_manager.h"
#include "transport/pgl_spi_target.h"
#include "devices/device_service.h"
#include "render/frame_renderer.h"
#include "memory/mem_assets.h"
#if PROTOGPU_ENABLE_PSRAM
#include "memory/mem_qmi_psram.h"
#endif
#if PROTOGPU_HOSTLESS_DEMO
#include "diagnostics/demo_scene.h"
#endif
#include <PglEncoder.h>
#include <PglRuntimeProtocol.h>
#include "pgl_build_identity.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include <cstring>
#include "hardware/sync.h"
#include <cstdio>

extern "C" const volatile uint32_t pgl_build_id;
namespace {
using R=PglRuntime::Result;
HardwareResources resources(GpuConfig::GPIO_AVAILABLE_MASK);
DisplayManager displays(resources);
PglTileScheduler scheduler;
Rasterizer rasterizer;
FrameRenderer renderer(rasterizer,scheduler);
SceneState scene;
gpudev::DeviceService devices;
gpumem::AssetService assets;
PglSpiTarget* transport=nullptr;
alignas(4) uint16_t colors[2][GpuConfig::FRAMEBUF_PIXELS] = {};
PhaseScratch::DepthWorkspace depth;
alignas(16) uint8_t ingress[2][PglRuntime::MaxBatchBytes] = {};
struct Slot { enum State:uint8_t{Free,Reserved,Queued,Retained} state=Free;PglRuntime::Control reservation;size_t length=0; } slots[2];
int armed=-1,queued=-1,retained=-1;
uint8_t renderColor=0;
uint32_t session=0,lastSeenSequence=0,acceptedSequence=0,acceptedFrame=0,renderedFrame=0,transferredFrame=0,displayedFrame=0,failedFrame=0;
R lastError=R::Ok,lastPacketResult=R::Ok;
PglRuntime::State state=PglRuntime::State::Starting;
PglRuntime::Record published;
PglRuntime::FrameFence history[GpuConfig::FRAME_HISTORY];
uint8_t historyWrite=0;
bool executing=false,workerStarted=false,componentsReady=false,serviceBusy=false,resetRequested=false;
uint32_t sourceFrame=0;
FrameTimings timings;
uint64_t maxServiceGap=0,lastServiceUs=0;
uint32_t controlSequence=0;
PglRuntime::DeviceReply lastDeviceReply;
R lastDeviceResult=R::InvalidHandle;
uint32_t responseSession=0;
#if PROTOGPU_HOSTLESS_DEMO
bool diagnosticActive=true;
uint32_t diagnosticFrame=0;
uint64_t diagnosticLastUs=0;
#endif

uint64_t Now(){return time_us_64();}
uint32_t FreeSceneBytes(){return uint32_t(scene.sceneHeap.stats().freeBytes);}
uint8_t FreeSlots(){uint8_t n=0;for(const auto&slot:slots)if(slot.state==Slot::Free)++n;return n;}
PglRuntime::FrameFence* Fence(uint32_t frame){for(auto&f:history)if(f.frame==frame&&f.completion!=PglRuntime::Completion::Unknown)return &f;return nullptr;}
void UpdateFence(uint32_t frame,PglRuntime::Completion completion,R result){if(auto*f=Fence(frame)){f->completion=completion;f->result=result;if(completion==PglRuntime::Completion::Rendered)f->renderedUs=Now();if(completion==PglRuntime::Completion::Displayed||completion==PglRuntime::Completion::Transferred)f->presentedUs=Now();}}
void AddFence(uint32_t frame,uint32_t sequence){auto&f=history[historyWrite++%GpuConfig::FRAME_HISTORY];f={};f.frame=frame;f.transferSequence=sequence;f.completion=PglRuntime::Completion::Accepted;f.result=R::Pending;f.acceptedUs=Now();}

void Publish(PglRuntime::Info info=PglRuntime::Info::Status,R result=R::Ok,uint32_t argument=0,bool bulkTerminal=false) {
    published={};published.info=info;published.result=result;published.session=responseSession;published.sequence=controlSequence;
    if(info==PglRuntime::Info::Status){
        const auto clock=GpuClock::GetSnapshot();PglRuntime::Status value;
        value.responsePhase=bulkTerminal?PglRuntime::ResponsePhase::BulkTerminal:PglRuntime::ResponsePhase::Control;
        value.acceptedSequence=acceptedSequence;value.acceptedFrame=acceptedFrame;value.renderedFrame=renderedFrame;value.transferredFrame=transferredFrame;value.displayedFrame=displayedFrame;value.failedFrame=failedFrame;
        const bool admitting=componentsReady&&!resetRequested&&clock.transition==GpuClock::Transition::Idle;
        value.state=state;value.frameCredits=admitting&&queued<0?1:0;value.ingressCredits=admitting?FreeSlots():0;
        value.clockProfile=clock.actual;value.clockMHz=clock.actualHz/1000000;value.hostSckKHz=1000;
        value.lastError=uint32_t(lastError);value.freeSramBytes=FreeSceneBytes();value.parserErrors=CommandParser::GetParserErrorCount();PglRuntime::EncodeStatus(value,published);
    }else if(info==PglRuntime::Info::Capabilities){
        auto caps=displays.GetCaps();PglRuntime::Capabilities value;
        value.features=PglRuntime::Raster3D|PglRuntime::OrderedAlpha|PglRuntime::Layers2D|PglRuntime::ShaderVM|PglRuntime::ClockProfiles|PglRuntime::AttachedDevices|PglRuntime::QuadData|PglRuntime::StreamUpload|PglRuntime::PixelMapping|PglRuntime::Metrics;
        if(caps.displayCompletion)value.features|=PglRuntime::DisplayCompletion;
        if(assets.hasBacking())value.features|=PglRuntime::ExternalAssets;
        value.width=scene.renderWidth;value.height=scene.renderHeight;value.maxVertices=GpuConfig::MAX_VERTICES;value.maxSourceTriangles=GpuConfig::MAX_SOURCE_TRIANGLES;value.maxProjectedTriangles=GpuConfig::MAX_TRIANGLES;
        value.sceneHeapBytes=GpuConfig::SCENE_HEAP_MAX_BYTES;value.externalAssetBytes=assets.capacityBytes();value.maxMeshes=GpuConfig::MAX_MESHES;value.maxMaterials=GpuConfig::MAX_MATERIALS;value.maxTextures=GpuConfig::MAX_TEXTURES;value.maxLayers=GpuConfig::MAX_LAYERS;
        value.maxCameras=PGL_MAX_CAMERAS;value.maxDraws=GpuConfig::MAX_DRAW_CALLS;value.maxShaderPrograms=GpuConfig::MAX_SHADER_PROGRAMS;value.maxShaderInstructions=PSB_MAX_INSTRUCTIONS;value.maxDraws2D=PGL_MAX_2D_DRAW_CMDS;
        value.displayType=uint8_t(displays.Config().type);value.bootStorage=GpuConfig::RAM_HOST?PglRuntime::BootStorage::RamHost:PglRuntime::BootStorage::FlashLocal;value.dataWidths=5;value.clockProfiles=0xff;value.reserveBytes=GpuConfig::SRAM_RESERVE_BYTES;value.postFxBudgetKi=GpuConfig::POSTFX_WORK_BUDGET/1024;PglRuntime::EncodeCapabilities(value,published);
    }else if(info==PglRuntime::Info::Boot){
        PglRuntime::BootInfo value;value.buildId=pgl_build_id;value.bootStorage=GpuConfig::RAM_HOST?PglRuntime::BootStorage::RamHost:PglRuntime::BootStorage::FlashLocal;value.state=state;value.clockProfile=GpuClock::GetSnapshot().actual;value.freeSramBytes=FreeSceneBytes();std::memcpy(value.boardId,GpuConfig::BOARD_ID,sizeof(GpuConfig::BOARD_ID));PglRuntime::EncodeBootInfo(value,published);
    }else if(info==PglRuntime::Info::Fence){
        auto*f=Fence(argument);PglRuntime::FrameFence missing;missing.frame=argument;missing.result=R::InvalidHandle;PglRuntime::EncodeFence(f?*f:missing,published);
    }else if(info==PglRuntime::Info::Memory){
        const auto stats=assets.stats();const auto heap=scene.sceneHeap.stats();
        PglRuntime::MemoryUsage value;
        value.sceneFreeBytes=heap.freeBytes;value.sceneUsedBytes=heap.usedBytes;
        value.scenePeakUsedBytes=heap.peakUsedBytes;value.sceneLargestFreeBytes=heap.largestFreeBlock;
        value.sceneCapacityBytes=heap.segmentBytes;
        value.externalCapacityBytes=stats.storeCapacityBytes;value.externalFreeBytes=stats.storeFreeBytes;
        value.stagingFreeBytes=stats.stagingFreeBytes;value.stagingLargestFreeBytes=stats.stagingLargestFreeBytes;
        value.stagingCapacityBytes=stats.stagingBytes;PglRuntime::EncodeMemoryUsage(value,published);
    }else if(info==PglRuntime::Info::Metrics){
        PglRuntime::FrameMetrics value;
        value.timestampUs=Now();value.prepareUs=timings.prepareUs;value.rasterUs=timings.rasterUs;
        value.effectsUs=timings.effectsUs;value.layersUs=timings.layersUs;
        value.triangles=timings.triangles;value.maxServiceGapUs=maxServiceGap;
        PglRuntime::EncodeFrameMetrics(value,published);
    }else if(info==PglRuntime::Info::Clock){
        const auto clock=GpuClock::GetSnapshot();PglRuntime::ClockState value;
        value.requestedProfile=clock.requested;value.actualProfile=clock.actual;value.actualHz=clock.actualHz;
        value.overrideReason=uint8_t(clock.override);value.transition=uint8_t(clock.transition);value.lastResult=clock.lastResult;
        value.thermalEnabled=clock.thermalEnabled;value.criticalFaultPending=clock.criticalFaultPending;
        value.temperatureC=clock.lastTemperatureC;PglRuntime::EncodeClockState(value,published);
    }else if(info==PglRuntime::Info::ClockConfiguration){
        PglRuntime::ClockConfiguration value;
        published.result=GpuClock::GetClockConfiguration(uint8_t(argument),value);
        if(published.result==R::Ok)PglRuntime::EncodeClockConfiguration(value,published);
    }else if(info==PglRuntime::Info::Device){
        if(argument&&argument!=lastDeviceReply.requestId)published.result=R::InvalidHandle;
        else{published.result=lastDeviceResult;PglRuntime::EncodeDeviceReply(lastDeviceReply,published);}
    }else published.result=R::Unsupported;
    transport->PublishRecord(published);
    gpio_put(GpuConfig::HOST_IRQ,0);
}

void ReleaseSources(){scene.RetireResourceReads();scene.RetireFrameData();sourceFrame=0;}
void Service();
void RestoreFlashTiming(void*) {
    auto platform=GpuClock::TargetPlatform();
    if(platform.commitFlashDivisor && !platform.commitFlashDivisor(platform.ctx,GpuClock::FlashClkDivFor(platform.sysClockHz(platform.ctx))))state=PglRuntime::State::Fault;
}
R StartComponents(){
#if PROTOGPU_ENABLE_PSRAM
    gpumem::QmiPsramConfig memoryConfig;memoryConfig.csGpio=GpuConfig::PSRAM_CS_PIN;memoryConfig.resources=&resources;
    memoryConfig.postXipRestoreHook=RestoreFlashTiming;
    const uint32_t irqState=save_and_disable_interrupts();
    const auto initialized=gpumem::qmiPsramInit(memoryConfig);
    RestoreFlashTiming(nullptr);restore_interrupts(irqState);
    if(initialized==gpumem::QmiPsramStatus::Ready||initialized==gpumem::QmiPsramStatus::AlreadyInitialized){
        void* staging=scene.SceneHeapAlloc(GpuConfig::MAX_ACTIVE_ASSET_BYTES);
        if(!staging){gpumem::qmiPsramDeinit();lastError=R::NoMemory;}
        else if(assets.bind(gpumem::qmiPsramBacking(),staging,GpuConfig::MAX_ACTIVE_ASSET_BYTES)!=gpumem::AssetStatus::Ok){scene.SceneHeapFree(staging);gpumem::qmiPsramDeinit();lastError=R::Io;}
    }
#endif
    scene.externalAssets=&assets;
    DisplayConfig config;config.type=static_cast<DisplayType>(GpuConfig::DEFAULT_DISPLAY);config.width=config.height=0;
    R configured=displays.Configure(config);if(configured!=R::Ok)return configured;
    scene.renderWidth=displays.Config().width;scene.renderHeight=displays.Config().height;
    scene.layers[0].width=scene.layers[0].clipW=scene.renderWidth;
    scene.layers[0].height=scene.layers[0].clipH=scene.renderHeight;
    R device=gpudev::InitTarget(devices,resources);if(device!=R::Ok){displays.Shutdown();return device;}
    scheduler.Initialize();multicore_launch_core1(GpuCore::Core1Main);if(!scheduler.StartWorker())return R::Io;
    workerStarted=componentsReady=true;state=PglRuntime::State::Ready;return R::Ok;
}

bool Retime(void*,uint32_t hz){bool good=displays.Resume(hz)==R::Ok;good=devices.RetimeForClock(hz)&&good;good=(transport->Resume(hz)==R::Ok)&&good;uart_set_baudrate(uart0,115200);return good;}
bool PrepareMemory(void*,uint32_t hz){
#if PROTOGPU_ENABLE_PSRAM
    return !gpumem::qmiPsramIsReady()||gpumem::qmiPsramClockPrepare(hz);
#else
    (void)hz;return true;
#endif
}
bool FinalizeMemory(void*,uint32_t hz){
#if PROTOGPU_ENABLE_PSRAM
    return !gpumem::qmiPsramIsReady()||gpumem::qmiPsramClockFinalize(hz);
#else
    (void)hz;return true;
#endif
}

void HandleControl(const PglRuntime::Control& control){
    controlSequence=control.sequence;responseSession=control.session;R result=R::Ok;
    if(control.command==PglRuntime::Command::Query){
        if(control.session&&control.session!=session){Publish(PglRuntime::Info::Status,R::BadSession);return;}
        Publish(static_cast<PglRuntime::Info>(control.args[0]),R::Ok,control.args[1]);return;
    }
    if(control.command==PglRuntime::Command::ReleaseBootPins){
        if(control.session||executing){result=R::BadState;}
        else if(!componentsReady){result=StartComponents();if(result!=R::Ok)state=PglRuntime::State::Fault;}
        Publish(PglRuntime::Info::Status,result);return;
    }
    if(control.command==PglRuntime::Command::OpenSession){
        if(!componentsReady||executing||displays.ReadingSource()||queued>=0||armed>=0||!control.session){Publish(PglRuntime::Info::Status,R::Busy);return;}
#if PROTOGPU_HOSTLESS_DEMO
        diagnosticActive=false;
#endif
        if(session!=control.session){scene.Reset();session=control.session;lastSeenSequence=acceptedSequence=acceptedFrame=renderedFrame=transferredFrame=displayedFrame=failedFrame=0;retained=-1;for(auto&s:slots)s={};for(auto&f:history)f={};historyWrite=0;CommandParser::ClearErrors();}
        Publish();return;
    }
    if(!session||control.session!=session){Publish(PglRuntime::Info::Status,R::BadSession);return;}
    switch(control.command){
        case PglRuntime::Command::Reserve:{
            uint8_t lanes=PglRuntime::ReservationLanes(control.args[1]);auto kind=PglRuntime::ReservationKind(control.args[1]);
            if(!control.sequence||!control.args[0]||control.args[0]>PglRuntime::MaxBatchBytes||(lanes!=1&&lanes!=4)||(kind!=PglRuntime::BulkKind::Frame&&kind!=PglRuntime::BulkKind::AssetChunk)||(control.args[1]&0xffff0000u)){result=R::InvalidValue;break;}
            if(resetRequested || GpuClock::GetSnapshot().transition!=GpuClock::Transition::Idle){result=R::Busy;break;}
            if(kind==PglRuntime::BulkKind::Frame && (!control.args[2] ||
               (control.sequence>lastSeenSequence && control.args[2]<=acceptedFrame))){result=R::BadSequence;break;}
            if(armed>=0||queued>=0){result=R::Busy;break;}
            if(control.sequence<lastSeenSequence){result=R::BadSequence;break;}
            if(control.sequence==lastSeenSequence&&retained>=0){const auto&prior=slots[retained].reservation;if(std::memcmp(prior.args,control.args,sizeof(control.args))){result=R::Conflict;break;}}
            int chosen=-1;for(int i=0;i<2;++i)if(slots[i].state==Slot::Free){chosen=i;break;}if(chosen<0){result=R::Busy;break;}
            result=transport->Arm(ingress[chosen],sizeof(ingress[chosen]),control.args[0],lanes);
            if(result==R::Ok){armed=chosen;slots[chosen].state=Slot::Reserved;slots[chosen].reservation=control;slots[chosen].length=control.args[0];}break;
        }
        case PglRuntime::Command::Cancel:
            result=transport->Disarm();if(result==R::Ok&&armed>=0){slots[armed]={};armed=-1;}break;
        case PglRuntime::Command::SetClock:
            if(control.args[0]>=PglRuntime::ClockProfileCount)result=R::InvalidValue;else result=GpuClock::RequestProfile(uint8_t(control.args[0]));break;
        case PglRuntime::Command::SetBrightness:result=control.args[0]>255?R::InvalidValue:displays.SetBrightness(control.args[0]);break;
        case PglRuntime::Command::ConfigureDisplay:{
            if(executing||queued>=0||armed>=0||displays.ReadingSource()){result=R::Busy;break;}
            DisplayConfig config;config.type=static_cast<DisplayType>(control.args[0]);config.width=control.args[1];config.height=control.args[1]>>16;config.brightness=control.args[2];config.flipH=control.args[2]&(1u<<8);config.flipV=control.args[2]&(1u<<9);config.layoutId=control.args[3];
            if(control.args[0]>255||control.args[3]>=8||(control.args[2]&~1023u)){result=R::InvalidValue;break;}
            result=displays.Configure(config);
            if(result==R::Ok){
                scene.renderWidth=displays.Config().width;scene.renderHeight=displays.Config().height;
                scene.layers[0].width=scene.layers[0].clipW=scene.renderWidth;
                scene.layers[0].height=scene.layers[0].clipH=scene.renderHeight;
            }break;
        }
        case PglRuntime::Command::Device:{
            gpudev::DeviceRequest request;result=gpudev::DecodeDeviceArgs(control.args,request);
            if(result!=R::Ok)break;
            uint32_t id=0;result=devices.Submit(request,id);
            if(id){
                lastDeviceReply={};lastDeviceReply.requestId=id;lastDeviceReply.deviceId=request.deviceId;
                lastDeviceReply.operation=request.operation;lastDeviceResult=result==R::Ok?R::Pending:result;
                devices.Poll();devices.PopReply(lastDeviceReply,lastDeviceResult);
                Publish(PglRuntime::Info::Device,R::Ok,id);return;
            }break;
        }
        case PglRuntime::Command::Reset:
            if(executing||queued>=0||armed>=0||displays.ReadingSource())result=R::Busy;
            else resetRequested=true;
            break;
        default:result=R::Unsupported;break;
    }
    if(result!=R::Ok&&result!=R::Busy)lastError=result;
    Publish(PglRuntime::Info::Status,result);
}

void Service(){
    if(serviceBusy)return;
    serviceBusy=true;
    const uint64_t now=Now();if(lastServiceUs&&now-lastServiceUs>maxServiceGap)maxServiceGap=now-lastServiceUs;lastServiceUs=now;
    transport->Poll(now);displays.Poll();
    DisplayEvent event;while(displays.PopEvent(event)){
        if(event.result!=R::Ok){lastError=event.result;failedFrame=event.frame;UpdateFence(event.frame,PglRuntime::Completion::Failed,event.result);}
        else if(event.completion==PglRuntime::Completion::Transferred){transferredFrame=event.frame;UpdateFence(event.frame,event.completion,R::Ok);}
        else if(event.completion==PglRuntime::Completion::Displayed){displayedFrame=event.frame;transferredFrame=event.frame;UpdateFence(event.frame,event.completion,R::Ok);}
        else if(event.completion==PglRuntime::Completion::Rendered){renderedFrame=event.frame;UpdateFence(event.frame,event.completion,R::Ok);}
        if(event.sourceReleased&&event.frame==sourceFrame)ReleaseSources();
    }
    PglSpiTarget::RxTransaction transaction;
    if(transport->PopTransaction(transaction)==R::Ok){
        if(transaction.kind==PglSpiTargetDetail::TransactionKind::Control){PglRuntime::Control control;R decoded=PglRuntime::DecodeControl(transaction.bytes,transaction.length,control);if(decoded==R::Ok)HandleControl(control);else{lastError=decoded;Publish(PglRuntime::Info::Status,decoded);}}
        else if(transaction.kind==PglSpiTargetDetail::TransactionKind::Bulk){
            if(armed<0){lastError=R::BadState;Publish(PglRuntime::Info::Status,lastError);}
            else{queued=armed;armed=-1;slots[queued].state=Slot::Queued;}
        }else if(transaction.kind==PglSpiTargetDetail::TransactionKind::Read){
            gpio_put(GpuConfig::HOST_IRQ,1);
        }else if(transaction.overflow||transaction.partial){
            lastError=R::BadPacket;
            if(armed>=0){
                controlSequence=slots[armed].reservation.sequence;responseSession=slots[armed].reservation.session;
                transport->Disarm();slots[armed]={};armed=-1;
                Publish(PglRuntime::Info::Status,lastError,0,true);
            }else Publish(PglRuntime::Info::Status,lastError);
        }
        transport->ReleaseTransaction();
    }
    devices.Poll();transport->SetReady(!resetRequested&&state!=PglRuntime::State::Fault);
    serviceBusy=false;
}

void RunQueued(){
    if(queued<0||executing||displays.ReadingSource())return;
    int index=queued;queued=-1;auto&slot=slots[index];const auto control=slot.reservation;
    responseSession=control.session;
    R result=R::Ok;
    if(PglRuntime::PayloadChecksum(ingress[index],slot.length)!=control.args[3])result=R::BadPacket;
    if(result==R::Ok&&control.sequence==lastSeenSequence&&retained>=0){
        result=slot.length==slots[retained].length&&!std::memcmp(ingress[index],ingress[retained],slot.length)?lastPacketResult:R::Conflict;
        slot={};controlSequence=control.sequence;Publish(PglRuntime::Info::Status,result,0,true);return;
    }
    CommandParser::BatchInfo batch;
    const bool resourceOnly=PglRuntime::ReservationKind(control.args[1])==PglRuntime::BulkKind::AssetChunk;
    if(result==R::Ok&&(slot.length<sizeof(PglFrameHeader)+2||
       PglRuntime::Load32(ingress[index]+2)!=control.args[2]))result=R::BadPacket;
    if(result==R::Ok)result=CommandParser::Parse(ingress[index],slot.length,&scene,batch,resourceOnly);
    lastSeenSequence=control.sequence;lastPacketResult=result;
    if(retained>=0)slots[retained]={};
    retained=index;slot.state=Slot::Retained;
    controlSequence=control.sequence;
    if(result!=R::Ok){lastError=result;failedFrame=control.args[2];Publish(PglRuntime::Info::Status,result,0,true);return;}
    acceptedSequence=control.sequence;
    if(resourceOnly){Publish(PglRuntime::Info::Status,R::Ok,0,true);return;}
    acceptedFrame=batch.frameNumber;AddFence(batch.frameNumber,control.sequence);state=PglRuntime::State::Rendering;
    Publish(PglRuntime::Info::Status,R::Ok,0,true);
    executing=true;result=renderer.Render(scene,colors[renderColor],depth,scene.renderWidth,scene.renderHeight,timings,Now,Service);executing=false;state=PglRuntime::State::Ready;
    if(result!=R::Ok){failedFrame=batch.frameNumber;lastError=result;UpdateFence(batch.frameNumber,PglRuntime::Completion::Failed,result);ReleaseSources();return;}
    renderedFrame=batch.frameNumber;UpdateFence(batch.frameNumber,PglRuntime::Completion::Rendered,R::Pending);
    DisplayMapping mapping;const auto&config=displays.Config();const auto&layout=scene.pixelLayouts[config.layoutId];if(layout.active){mapping.count=layout.pixelCount;mapping.reversed=layout.flags&PGL_LAYOUT_REVERSED;mapping.rectangular=layout.flags&PGL_LAYOUT_RECTANGULAR;mapping.rectangle=layout.rectData;mapping.coordinates=layout.coords;}
    DisplaySurface surface{colors[renderColor],GpuConfig::FRAMEBUF_PIXELS,scene.renderWidth,scene.renderHeight,scene.renderWidth,batch.frameNumber};
    sourceFrame=batch.frameNumber;result=displays.Present(surface,mapping);
    if(result!=R::Ok&&result!=R::Pending){failedFrame=batch.frameNumber;lastError=result;UpdateFence(batch.frameNumber,PglRuntime::Completion::Failed,result);ReleaseSources();}
    else renderColor^=1;
}
#if PROTOGPU_HOSTLESS_DEMO
void Diagnostic() {
    if(!diagnosticActive||!componentsReady||session||executing||armed>=0||queued>=0||
       displays.ReadingSource()||resetRequested||state!=PglRuntime::State::Ready)return;
    const uint64_t now=Now();if(now-diagnosticLastUs<33333)return;
    int index=-1;for(int i=0;i<2;++i)if(slots[i].state==Slot::Free){index=i;break;}
    if(index<0)return;
    const uint32_t frame=diagnosticFrame+1;if(!frame){state=PglRuntime::State::Fault;return;}
    const size_t length=GpuDiagnostic::EncodeFrame(ingress[index],sizeof(ingress[index]),frame,33333);
    if(!length){state=PglRuntime::State::Fault;return;}
    diagnosticFrame=frame;diagnosticLastUs=now;
    auto& slot=slots[index];slot.state=Slot::Queued;slot.length=length;
    slot.reservation={};slot.reservation.command=PglRuntime::Command::Reserve;slot.reservation.sequence=frame;
    slot.reservation.args[0]=length;slot.reservation.args[1]=PglRuntime::ReservationMode(1,PglRuntime::BulkKind::Frame);
    slot.reservation.args[2]=frame;slot.reservation.args[3]=PglRuntime::PayloadChecksum(ingress[index],length);
    queued=index;
}
#endif


void Maintenance(){
    if(executing||queued>=0||armed>=0||displays.ReadingSource()||!devices.IsIdle()||!workerStarted)return;
    auto clock=GpuClock::GetSnapshot();if(clock.transition!=GpuClock::Transition::AwaitingSafePoint&&!resetRequested)return;
    state=PglRuntime::State::Quiescing;transport->SetReady(false);
    if(transport->Quiesce()!=R::Ok){state=PglRuntime::State::Ready;return;}
    if(displays.Quiesce()!=R::Ok){
        transport->Resume(GpuClock::GetSnapshot().actualHz);
        state=PglRuntime::State::Ready;return;
    }
    if(scheduler.EnterMaintenance()!=PglSchedResult::Ok){state=PglRuntime::State::Fault;return;}
    if(resetRequested){
        scene.Reset();session=0;lastSeenSequence=acceptedSequence=0;
        acceptedFrame=renderedFrame=transferredFrame=displayedFrame=failedFrame=0;
        lastError=lastPacketResult=R::Ok;lastDeviceReply={};lastDeviceResult=R::InvalidHandle;
        gpudev::ShutdownTarget(devices,resources);
        if(gpudev::InitTarget(devices,resources)!=R::Ok)state=PglRuntime::State::Fault;
        for(auto&s:slots)s={};
        for(auto&f:history)f={};
        historyWrite=0;
        retained=queued=armed=-1;resetRequested=false;CommandParser::ClearErrors();
    }
    else {
        GpuClock::SafeGates gates;gates.workersParked=gates.hostIdle=gates.displayDrained=gates.devicesDrained=gates.memoryDrained=true;
        GpuClock::ApplyHooks hooks;hooks.qmiPrepare=PrepareMemory;hooks.qmiFinalize=FinalizeMemory;hooks.retimeClients=Retime;
        R applied=GpuClock::TryApply(gates,hooks);if(applied!=R::Ok)lastError=applied;
    }
    scheduler.ExitMaintenance();
    if(transport->Quiesced()){
        displays.Resume(GpuClock::GetSnapshot().actualHz);
        transport->Resume(GpuClock::GetSnapshot().actualHz);
    }
    if(state!=PglRuntime::State::Fault)
        state=GpuClock::GetSnapshot().transition==GpuClock::Transition::Fault?PglRuntime::State::Fault:PglRuntime::State::Ready;
}
} // namespace

bool GpuCore::Initialize(){
    const uint32_t irqState=save_and_disable_interrupts();
    RestoreFlashTiming(nullptr);
    restore_interrupts(irqState);
    if(state==PglRuntime::State::Fault||!scene.InitSceneHeap())return false;
    scene.Reset();
    if(resources.ClaimGpios(uint64_t(1)<<GpuConfig::DEBUG_TX,HardwareResources::Owner::Debug)!=R::Ok)return false;
    if(resources.ClaimBus(uint8_t(4+GpuConfig::DEBUG_UART),HardwareResources::Owner::Debug)!=R::Ok)return false;
    if(resources.ClaimGpios(uint64_t(1)<<GpuConfig::HOST_IRQ,HardwareResources::Owner::Host)!=R::Ok)return false;
    gpio_init(GpuConfig::HOST_IRQ);gpio_set_dir(GpuConfig::HOST_IRQ,GPIO_OUT);gpio_put(GpuConfig::HOST_IRQ,1);
    transport=&PglSpiTargetInstance();if(transport->Init(resources)!=R::Ok)return false;
    state=GpuConfig::RAM_HOST||GpuConfig::PSRAM_ENABLED?PglRuntime::State::AwaitingBootRelease:PglRuntime::State::Ready;
    if(state==PglRuntime::State::Ready&&StartComponents()!=R::Ok)return false;
    Publish();transport->SetReady(true);return true;
}
[[noreturn]]void GpuCore::Core1Main(){scheduler.Core1Main();for(;;)__wfe();}
[[noreturn]]void GpuCore::Core0Main(){
    watchdog_enable(8000,true);
    for(;;){
        Service();
#if PROTOGPU_HOSTLESS_DEMO
        Diagnostic();
#endif
        RunQueued();Maintenance();
        if(state==PglRuntime::State::Fault){
            transport->SetReady(false);displays.Shutdown();watchdog_reboot(0,0,100);
            for(;;)tight_loop_contents();
        }
        // Feeding only after a complete service/render/maintenance turn
        // makes a stuck worker rendezvous reset, even if Service keeps running.
        watchdog_update();
        if(queued<0)best_effort_wfe_or_timeout(make_timeout_time_us(64));
    }
}
