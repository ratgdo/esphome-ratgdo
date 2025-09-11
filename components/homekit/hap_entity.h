#pragma once
#include "const.h"
#include "esphome/core/entity_base.h"
#include <esp_log.h>
#include <map>

namespace esphome
{
  namespace homekit
  {
    class HAPEntity
    {
    private:
      static constexpr const char* TAG = "HAPEntity";
    protected:
      std::map<AInfo, const char*> accessory_info = { {NAME, NULL}, {MODEL, "HAP-GENERIC"}, {SN, NULL}, {MANUFACTURER, "rednblkx"}, {FW_REV, "0.1"} };
    public:
      HAPEntity(){ }
      HAPEntity(std::map<AInfo, const char*> accessory_info) { setInfo(accessory_info); }
      void setInfo(std::map<AInfo, const char*> info) {
        for (const auto& pair : info) {
          if (this->accessory_info.find(pair.first) != this->accessory_info.end()) {
            this->accessory_info[pair.first] = pair.second;
          }
        }
      }
      void virtual setup() { ESP_LOGI(TAG, "Uninmplemented!"); }
    };
  }
}