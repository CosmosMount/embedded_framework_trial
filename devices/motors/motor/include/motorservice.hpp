#pragma once

#include "motor.hpp"
#include "motorhandler.hpp"

#include <array>
#include <cstddef>

namespace motors
{

// Single control-thread owner; service and registered devices have program lifetime.
// Singleton protocol handlers cannot be shared between services or mixed with direct registration.
// start() may sleep for the sum of DM enable_timeout_ticks. stop/send_control never retry.
// ok means accepted by transport (grouped DJI/LK set_output only prepares data), not wire delivery.
// On failure inspect cleanup_status(); unsuccessful disable/zero submission is not a confirmed stop.
class motor_service
{
public:
    motor_service() = default;
    motor_service(const motor_service&) = delete;
    motor_service& operator=(const motor_service&) = delete;
    bool register_motor(motor& device) noexcept;

    types::status start() noexcept;
    types::status stop() noexcept;
    types::status send_control() noexcept;
    types::status cleanup_status() const noexcept { return cleanup_status_; }
    void alive_check() noexcept;

private:
    static constexpr std::size_t protocol_count = static_cast<std::size_t>(protocol::count);
    static constexpr std::size_t max_motors = bsp::can::bus_count * 8U;

    // Resolves the static-lifetime handler for a protocol.
    static motorhandler* handler_for(protocol value) noexcept;

    // Registered handlers and motors are non-owning and must outlive this service.
    std::array<motorhandler*, protocol_count> handlers_{};
    std::array<motor*, max_motors> motors_{};
    std::size_t motor_count_ = 0U;
    bool started_ = false;
    types::status cleanup_status_ = types::status::ok;
};

} // namespace motors
