#pragma once

#include "../ratgdo.h"
#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"

namespace esphome {
namespace ratgdo {

  enum OutputType {
    RATGDO_TEST
  };

  class RATGDOFloatOutput : public output::FloatOutput, public RATGDOClient, public Component {
   public:
      void setup() override;
      void write_state(float state) override;
      void dump_config() override;
        void set_output_type(OutputType output_type);

  protected:
    OutputType output_type_;
  };

} //namespace ratgdo
} //namespace esphome