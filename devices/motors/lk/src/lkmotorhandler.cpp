#include "lkmotorhandler.hpp"

namespace motors
{

lkmotorhandler& lkmotorhandler::instance()
{
    static lkmotorhandler handler;
    return handler;
}

types::status lkmotorhandler::register_motor(motor& device) noexcept
{
    if (device.get_protocol() != protocol::lk)
    {
        return types::status::invalid_arg;
    }
    auto& lk = static_cast<lkmotor&>(device);
    if (!lk.protocol_valid() || lk.can_id() < 0x141 || lk.can_id() > 0x144)
    {
        return types::status::invalid_arg;
    }

    const auto bus_status = ensure_bus(lk.can_bus());
    if (bus_status != types::status::ok) return bus_status;

    const uint8_t bus = static_cast<uint8_t>(lk.can_bus());
    const uint8_t slot = static_cast<uint8_t>(lk.can_id() - 0x141);
    if (motor_list_[bus][slot] && motor_list_[bus][slot] != &lk) return types::status::busy;
    motor_list_[bus][slot] = &lk;

    return types::status::ok;
}

types::status lkmotorhandler::stop() noexcept
{
    for (auto& row : motor_list_)
        for (auto* device : row)
            if (device) device->relax();
    return send_control();
}

types::status lkmotorhandler::send_control() noexcept
{
    auto result = types::status::ok;
    for (uint8_t bus = 0; bus < bsp::can::bus_count; ++bus)
    {
        uint8_t broadcast_data[8]{};

        bool has_motor = false;
        for (uint8_t i = 0; i < lk_max_motors; ++i)
        {
            lkmotor* m = motor_list_[bus][i];
            if (m == nullptr)
            {
                continue;
            }

            m->set_output();
            if (!m->broadcast())
            {
                const uint16_t value = static_cast<uint16_t>(m->cmd.current);
                uint8_t data[8]{};
                data[0] = 0xa1; // relax requests zero torque current, not driver shutdown.
                data[4] = static_cast<uint8_t>(value);
                data[5] = static_cast<uint8_t>(value >> 8);
                retain_first_error(result, bsp::can::transmit(static_cast<bsp::can::bus>(bus), m->can_id(), data, 8));
                continue;
            }
            has_motor = true;

            const uint8_t index = static_cast<uint8_t>(i * 2U);
            const uint16_t current = static_cast<uint16_t>(m->cmd.current);
            broadcast_data[index] = static_cast<uint8_t>(current);
            broadcast_data[index + 1U] = static_cast<uint8_t>(current >> 8U);
        }

        if (has_motor)
        {
            retain_first_error(result, bsp::can::transmit(static_cast<bsp::can::bus>(bus), lk_control_id, broadcast_data, 8));
        }
    }
    return result;
}

void lkmotorhandler::dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame)
{
    if (frame.id < 0x141 || frame.id >= 0x141 + lk_max_motors)
    {
        return;
    }

    const uint8_t slot = static_cast<uint8_t>(frame.id - 0x141);
    lkmotor* m = motor_list_[static_cast<uint8_t>(bus)][slot];
    if (m != nullptr)
    {
        m->parse_feedback(frame.data, frame.len);
    }
}

} // namespace motors
