#pragma once

#include "../xysk.h"
#include "esphome/core/component.h"
#include "esphome/components/number/number.h"

namespace esphome {
namespace xysk {

class Xysk;

class XyskNumber : public number::Number, public Component {
 public:
  void set_parent(Xysk *parent) { this->parent_ = parent; };
  void set_holding_register(uint16_t holding_register) { this->holding_register_ = holding_register; };
  void dump_config() override;

 protected:
  void control(float value) override;

  Xysk *parent_;
  uint16_t holding_register_;
};

}  // namespace xysk
}  // namespace esphome
