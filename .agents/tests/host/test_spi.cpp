#include "bsp_spi.hpp"
#include "bsp_spi_binding.hpp"
#include <stdexcept>
using types::status;
namespace {
SPI_HandleTypeDef handle;
unsigned calls = 0;
uint16_t last_size = 0;
void require(bool c) { if (!c) throw std::runtime_error("SPI length boundary"); }
int transfer(uint16_t size) { ++calls; last_size = size; return HAL_OK; }
}
namespace bsp::spi::detail {
bool binding_for(bus value, binding& out) noexcept {
    if (static_cast<uint8_t>(value) >= bus_count) return false;
    out = {&handle, true, true, true}; return true;
}
}
namespace bsp::dma { bool valid(buffer_view b) noexcept { return b.data() && b.logical_size(); } }
uint32_t HAL_GetTick() { return 0; }
int HAL_SPI_Transmit(SPI_HandleTypeDef*, uint8_t*, uint16_t n, uint32_t) { return transfer(n); }
int HAL_SPI_Receive(SPI_HandleTypeDef*, uint8_t*, uint16_t n, uint32_t) { return transfer(n); }
int HAL_SPI_TransmitReceive(SPI_HandleTypeDef*, uint8_t*, uint8_t*, uint16_t n, uint32_t) { return transfer(n); }
int HAL_SPI_Transmit_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t n) { return transfer(n); }
int HAL_SPI_Receive_IT(SPI_HandleTypeDef*, uint8_t*, uint16_t n) { return transfer(n); }
int HAL_SPI_Transmit_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t n) { return transfer(n); }
int HAL_SPI_Receive_DMA(SPI_HandleTypeDef*, uint8_t*, uint16_t n) { return transfer(n); }
int main() {
    const auto bus = static_cast<bsp::spi::bus>(0);
    static uint8_t bytes[65536]{};
    require(bsp::spi::transmit(bus, bytes, sizeof(bytes), 1) == status::invalid_arg);
    require(bsp::spi::receive(bus, bytes, sizeof(bytes), 1) == status::invalid_arg);
    require(bsp::spi::transmit_receive(bus, bytes, bytes, sizeof(bytes), 1) == status::invalid_arg);
    require(calls == 0);
    require(bsp::spi::transmit(bus, bytes, 65535, 1) == status::ok && last_size == 65535);
    require(bsp::spi::receive(bus, bytes, 1, 1) == status::ok && last_size == 1);
    require(bsp::spi::transmit(bus, bytes, 0, 1) == status::invalid_arg && calls == 2);
}
