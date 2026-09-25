#include "motorservice.hpp"

namespace motors
{

namespace
{

constexpr std::size_t protocol_index(protocol value) noexcept
{
    return static_cast<std::size_t>(value);
}

} // namespace

bool motor_service::register_motor(motor& device) noexcept
{
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        if (motors_[i] == &device)
        {
            return true;
        }
    }

    if (started_) return false;

    const std::size_t index = protocol_index(device.get_protocol());
    if (index >= handlers_.size() || motor_count_ >= motors_.size())
    {
        return false;
    }

    motorhandler* handler = handler_for(device.get_protocol());
    if (handler == nullptr || handler->supported_protocol() != device.get_protocol() ||
        (handler->owner_ && handler->owner_ != this) || !handler->register_motor(device))
        return false;
    handler->owner_ = this;
    handlers_[index] = handler;

    motors_[motor_count_++] = &device;
    return true;
}

types::status motor_service::start() noexcept
{
    if (started_)
    {
        return types::status::ok;
    }

    cleanup_status_ = types::status::ok;
    for (std::size_t index = 0; index < handlers_.size(); ++index)
    {
        auto* handler = handlers_[index];
        if (!handler) continue;
        const auto result = handler->start();
        if (result != types::status::ok)
        {
            cleanup_status_ = handler->cleanup_status();
            // Failing handler cleans its partial start; undo only earlier successful handlers.
            while (index)
            {
                auto* prior = handlers_[--index];
                if (prior) motorhandler::retain_first_error(cleanup_status_, prior->stop());
            }
            for (std::size_t i = 0; i < motor_count_; ++i) motors_[i]->relax();
            return result;
        }
    }
    started_ = true;
    return types::status::ok;
}

types::status motor_service::stop() noexcept
{
    started_ = false;
    auto result = types::status::ok;
    for (std::size_t index = handlers_.size(); index; --index)
        if (auto* handler = handlers_[index - 1])
            motorhandler::retain_first_error(result, handler->stop());
    cleanup_status_ = result;
    return result;
}

types::status motor_service::send_control() noexcept
{
    if (!started_) return types::status::not_initialized;
    auto result = types::status::ok;
    for (auto* handler : handlers_)
        if (handler) motorhandler::retain_first_error(result, handler->send_control());
    return result;
}

void motor_service::alive_check() noexcept
{
    for (std::size_t i = 0; i < motor_count_; ++i)
    {
        motors_[i]->alive_check();
    }
}

} // namespace motors
