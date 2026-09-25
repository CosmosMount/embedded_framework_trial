#pragma once

#include <cstdint>

namespace types
{
    enum class status : uint8_t 
    {
        ok = 0,
        error,
        not_configured,
        invalid_arg,
        busy,
        not_initialized,
        not_connected,
        empty,
        too_large,
        invalid_context,
    };

};
