#include "djimotorhandler.hpp"

namespace motors
{

djimotorhandler& djimotorhandler::instance()
{
    static djimotorhandler handler;
    return handler;
}

types::status djimotorhandler::register_motor(motor& device) noexcept
{
    if (device.get_protocol() != protocol::dji)
    {
        return types::status::invalid_arg;
    }
    auto& dji = static_cast<djimotor&>(device);
    if (!dji.valid_can_id())
    {
        return types::status::invalid_arg;
    }

    const auto bus_status = ensure_bus(dji.can_bus());
    if (bus_status != types::status::ok) return bus_status;

    const uint8_t bus = static_cast<uint8_t>(dji.can_bus());
    const uint8_t slot = static_cast<uint8_t>(dji.can_id() - 0x201);
    if (motor_list_[bus][slot] && motor_list_[bus][slot] != &dji) return types::status::busy;
    motor_list_[bus][slot] = &dji;

    return types::status::ok;
}

types::status djimotorhandler::stop() noexcept
{
    for (auto& row : motor_list_)
        for (auto* device : row)
            if (device) device->relax();
    return send_control();
}

types::status djimotorhandler::send_control() noexcept
{
    auto result = types::status::ok;
    // GM6020 loop selection changes the command ID, not its feedback ID.
    constexpr uint32_t ids[] = {0x200, 0x1ff, 0x2ff, 0x1fe, 0x2fe};
    for (uint8_t bus = 0; bus < bsp::can::bus_count; ++bus)
    {
        for (uint32_t id : ids)
        {
            uint8_t data[8]{};
            bool used = false;
            for (djimotor* m : motor_list_[bus])
            {
                if (!m || m->control_id() != id) continue;
                m->set_output();
                const uint16_t value = static_cast<uint16_t>(m->raw_output());
                const auto index = static_cast<uint8_t>(2 * m->control_slot());
                data[index] = static_cast<uint8_t>(value >> 8);
                data[index + 1] = static_cast<uint8_t>(value);
                used = true;
            }
            if (used) retain_first_error(result, bsp::can::transmit(static_cast<bsp::can::bus>(bus), id, data, 8));
        }
    }
    return result;
}

void djimotorhandler::dispatch_rx(bsp::can::bus bus, const bsp::can::rx_frame& frame)
{
    if (frame.id < 0x201 || frame.id > 0x20b)
    {
        return;
    }

    const uint8_t slot = static_cast<uint8_t>(frame.id - 0x201);
    djimotor* m = motor_list_[static_cast<uint8_t>(bus)][slot];
    if (m != nullptr)
    {
        m->parse_feedback(frame.data, frame.len);
    }
}

} // namespace motors
