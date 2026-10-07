#include "display_manager.h"
#include "pixel_mapper.h"
#if defined(PICO_ON_DEVICE)
#include "pico/time.h"
#include "hardware/clocks.h"
#endif

using R = PglRuntime::Result;

R DisplayManager::Validate(DisplayConfig& config) {
    if (config.layoutId >= 8) return R::InvalidValue;
    if (!config.width || !config.height) {
        if (config.width || config.height) return R::InvalidValue;
        switch(config.type) {
            case DisplayType::SpiSsd1331: config.width=96;config.height=64;break;
            case DisplayType::LedV5: config.width=16;config.height=16;break;
            case DisplayType::None:case DisplayType::Hub75:case DisplayType::CustomRgb888:config.width=128;config.height=64;break;
            default:return R::Unsupported;
        }
    }
    if(uint32_t(config.width)*config.height>GpuConfig::FRAMEBUF_PIXELS)return R::Capacity;
    const uint32_t tiles=uint32_t((config.width+15)/16)*((config.height+15)/16);
    if(tiles>64)return R::Capacity;
    if(config.type==DisplayType::SpiSsd1331&&(config.width!=96||config.height!=64))return R::InvalidValue;
    if(config.type==DisplayType::Hub75&&(config.height!=64||(config.width!=64&&config.width!=128)))return R::InvalidValue;
    switch(config.type){case DisplayType::None:case DisplayType::Hub75:case DisplayType::SpiSsd1331:case DisplayType::LedV5:case DisplayType::CustomRgb888:return R::Ok;default:return R::Unsupported;}
}

DisplayDriver* DisplayManager::Find(DisplayType type) {
#if defined(PICO_ON_DEVICE)
    switch(type) {
        case DisplayType::Hub75:return &Hub75Backend();
        case DisplayType::SpiSsd1331:return &SpiDisplayBackend();
        case DisplayType::LedV5:return &LedArrayBackend();
        case DisplayType::CustomRgb888:return &CustomArrayBackend();
        default:return nullptr;
    }
#else
    (void)type;
    return nullptr;
#endif
}

R DisplayManager::Configure(DisplayConfig config) {
    R checked=Validate(config);if(checked!=R::Ok)return checked;
    if(readingSource_ || noOutputEvent_)return R::Busy;
    DisplayDriver* replacement=Find(config.type);
    if(config.type!=DisplayType::None&&!replacement)return R::Unsupported;
    uint64_t required=0;
    if(config.type==DisplayType::Hub75)for(uint8_t pin=6;pin<=19;++pin)required|=uint64_t(1)<<pin;
    if(config.type==DisplayType::SpiSsd1331)required=(uint64_t(1)<<6)|(uint64_t(1)<<7)|(uint64_t(1)<<9)|(uint64_t(1)<<10)|(uint64_t(1)<<11);
    if(config.type==DisplayType::LedV5)required=uint64_t(1)<<6;
    if(config.type==DisplayType::CustomRgb888)for(uint8_t pin=6;pin<=16;++pin)required|=uint64_t(1)<<pin;
    for(uint8_t pin=0;pin<48;++pin)if((required&resources_.ClaimedGpios()&(uint64_t(1)<<pin))&&!resources_.OwnsGpios(uint64_t(1)<<pin,HardwareResources::Owner::Display))return R::Conflict;
    if(driver_) { R stopped=driver_->Quiesce();if(stopped!=R::Ok)return stopped; }
    const DisplayConfig oldConfig=config_;DisplayDriver* oldDriver=driver_;bool hadOld=initialized_;
    if(driver_)driver_->Shutdown();
    driver_=nullptr;initialized_=false;
    if(replacement) {
        R started=replacement->Init(resources_,config,workspace_,sizeof(workspace_));
        if(started!=R::Ok) {
            replacement->Shutdown();
            if(hadOld&&oldDriver) {
                if(oldDriver->Init(resources_,oldConfig,workspace_,sizeof(workspace_))==R::Ok){driver_=oldDriver;config_=oldConfig;initialized_=true;}
            }
            return started;
        }
    }
    config_=config;driver_=replacement;initialized_=true;return R::Ok;
}

R DisplayManager::Present(const DisplaySurface& surface,const DisplayMapping& mapping) {
    if(!initialized_)return R::NotReady;
    if(readingSource_||noOutputEvent_)return R::Busy;
    if(!ValidDisplaySurface(surface)||surface.width!=config_.width||surface.height!=config_.height)return R::InvalidValue;
    if(!driver_) {
        renderEvent_={surface.frame,PglRuntime::Completion::Rendered,R::Ok,0,true};
        noOutputEvent_=true;return R::Ok;
    }
    R result=driver_->Present(surface,mapping);
    if(result==R::Ok||result==R::Pending)readingSource_=true;
    return result;
}
void DisplayManager::Poll(){if(driver_)driver_->PollRefresh();}
bool DisplayManager::PopEvent(DisplayEvent& event) {
    if(noOutputEvent_){event=renderEvent_;noOutputEvent_=false;return true;}
    if(!driver_||!driver_->PopEvent(event))return false;
    if(event.sourceReleased)readingSource_=false;
    return true;
}
R DisplayManager::Quiesce(){if(noOutputEvent_)return R::Busy;return driver_?driver_->Quiesce():R::Ok;}
R DisplayManager::Resume(uint32_t hz){return driver_?driver_->Resume(hz):R::Ok;}
R DisplayManager::SetBrightness(uint8_t brightness){if(!initialized_)return R::NotReady;R result=driver_?driver_->SetBrightness(brightness):R::Ok;if(result==R::Ok)config_.brightness=brightness;return result;}
void DisplayManager::Shutdown(){if(driver_)driver_->Shutdown();driver_=nullptr;initialized_=false;readingSource_=false;noOutputEvent_=false;}
DisplayCapabilities DisplayManager::GetCaps()const {
    if(driver_)return driver_->GetCaps();
    DisplayCapabilities caps;caps.width=config_.width;caps.height=config_.height;return caps;
}
