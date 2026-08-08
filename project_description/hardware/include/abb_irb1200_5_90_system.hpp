#ifndef abb_irb1200_5_90_system_hpp_
#define abb_irb1200_5_90_system_hpp_

#include <hardware_interface/hardware_component_interface.hpp>
#include <rclcpp/context.hpp>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"

namespace abb_irb1200_5_90{
    class IRB1200HardwareInterface: public hardware_interface::SystemInterface{
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(IRB1200HardwareInterface);

    };
}
#endif