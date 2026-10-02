#include "log.hpp"
#include "tx_api.h"

#include <cstring>
#include <iostream>
#include <stdexcept>

bool interrupt_writer = false;
types::status nested_write = types::status::ok;
types::status nested_read = types::status::ok;

ULONG tx_time_get()
{
    static ULONG tick = 10U;
    if (interrupt_writer)
    {
        interrupt_writer = false;
        logging::record output{};
        nested_write = logging::write(logging::level::error, "interrupt");
        nested_read = logging::read_latest(0U, output);
    }
    return tick++;
}

namespace
{
void require(bool value, const char* message)
{
    if (!value) throw std::runtime_error(message);
}
}

int main()
{
    try
    {
        require(logging::write(logging::level::error, "early") ==
                    types::status::not_initialized,
                "writes before initialization must fail visibly");
        logging::init();
        require(logging::clear() == types::status::ok, "clear initialized log");
        interrupt_writer = true;
        require(logging::write(logging::level::info, "can recover",
                               types::status::error, 2) == types::status::ok,
                "write record");
        require(nested_write == types::status::busy && nested_read == types::status::busy,
                "interrupt must not overwrite or read an incomplete record");
        logging::record record{};
        require(logging::read_latest(0, record) == types::status::ok,
                "read newest record");
        require(record.sequence == 1U && record.tick == 10U && record.value == 2 &&
                    record.status == types::status::error &&
                    std::strcmp(record.message, "can recover") == 0,
                "record fields");
        const uint32_t boots = logging::state().boot_count;
        logging::init();
        require(logging::state().boot_count == boots &&
                    logging::state().write_count == 1U,
                "repeated initialization keeps existing records and boot count");
        logging::set_level(logging::level::warning);
        require(logging::write(logging::level::info, "filtered") == types::status::ok,
                "filtered writes are intentional");
        require(logging::state().write_count == 1U, "filtered write does not consume a slot");
        for (uint16_t index = 0U; index <= params::logging::capacity; ++index)
            require(logging::write(logging::level::error, "wrap", types::status::ok,
                                   index) == types::status::ok,
                    "ring write");
        require(logging::state().retained_count == params::logging::capacity,
                "ring remains bounded");
        require(logging::read_latest(0, record) == types::status::ok &&
                    record.value == params::logging::capacity,
                "newest record survives wrap");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
