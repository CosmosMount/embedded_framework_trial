#include "motorservice.hpp"

#include "djimotorhandler.hpp"
#include "dmmotorhandler.hpp"
#include "lkmotorhandler.hpp"
#include "xv2motorhandler.hpp"
#include "robot_config.hpp"

namespace motors
{

motorhandler* motor_service::handler_for(protocol value) noexcept
{
    switch (value)
    {
        case protocol::dji:
            if constexpr (robot::motors::has_dji)
            {
                return &djimotorhandler::instance();
            }
            return nullptr;
        case protocol::dm:
            if constexpr (robot::motors::has_dm)
            {
                return &dmmotorhandler::instance();
            }
            return nullptr;
        case protocol::lk:
            if constexpr (robot::motors::has_lk)
            {
                return &lkmotorhandler::instance();
            }
            return nullptr;
        case protocol::xv2:
            if constexpr (robot::motors::has_xv2)
            {
                return &xv2motorhandler::instance();
            }
            return nullptr;
        case protocol::count:
        default:
            return nullptr;
    }
}

} // namespace motors
