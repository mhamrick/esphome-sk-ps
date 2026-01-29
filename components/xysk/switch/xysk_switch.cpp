#include "xysk_switch.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace xysk {

static const char *const TAG = "xysk.switch";

void XyskSwitch::dump_config() { LOG_SWITCH("", "XYSK Switch", this); }
void XyskSwitch::write_state(bool state) { this->parent_->write_register(this->holding_register_, (uint16_t) state); }

}  // namespace xysk
}  // namespace esphome
