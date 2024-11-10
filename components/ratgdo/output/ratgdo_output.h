#pragma once

#include "../ratgdo.h"
#include "esphome/components/rtttl/rtttl.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ratgdo {

    enum OutputType {
        RATGDO_BEEPER
    };

    class RATGDOOutput : public RATGDOClient, public Component {
    public:
        void setup() override;
        void play();
        void finished_playback();
        void dump_config() override;
        void set_output_type(OutputType output_type);
        void set_song(std::string rtttlSong) { this->rtttlSong_ = rtttlSong; }
        void set_rtttl(rtttl::Rtttl* output) { this->beeper_ = output; }

    protected:
        OutputType output_type_;
        rtttl::Rtttl* beeper_;
        std::string rtttlSong_;
        bool repeat_;
    };

} // namespace ratgdo
} // namespace esphome
