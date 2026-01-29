#include "xysk_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace xysk {

static const char *const TAG = "xysk.number";

void XyskNumber::dump_config() { LOG_NUMBER("", "XYSK Number", this); }
void XyskNumber::control(float value) {
  float resolution = 100.0f;

  if (this->holding_register_ == 0x0001) {
    resolution = 1.0f / this->parent_->current_resolution_factor();
  }

  this->parent_->write_register(this->holding_register_, (uint16_t) (value * resolution));
}

}  // namespace xysk
}  // namespace esphome
