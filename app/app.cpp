#include "log.hpp"

// Application entry called by the board's ThreadX initialization hook.
// Add robot-specific module initialization here.
extern "C" void app_start()
{
    logging::init();
    (void)logging::write(logging::level::info, "boot");
}
