#include "main.h"
#include "geometry.hpp"
#include "matrix.hpp"
#include "constrain.hpp"
#include "quantize.hpp"
#include "pid.hpp"
#include "iir.hpp"
#include "kalman_1d.hpp"
#include "crc.hpp"
#include "callback.hpp"
#include "msg.hpp"
#include "log.hpp"
#include "bsp_dwt.hpp"
#include "bsp_dma.hpp"
#include "quaternion_ekf.hpp"
#include "tactical_ekf.hpp"
#include <math.h>
#include <string.h>

struct target_result {
    uint32_t stage, checks, failures, failed_groups, done;
    uint32_t boot_count, retained_marker, prior_boot;
    uint32_t logger_cycles, clock_hz;
};
extern "C" { volatile target_result pnx_core_test{}; }
namespace {
TX_THREAD thread{}, contender{};
alignas(8) uint8_t stack[8192]{}, contender_stack[1024]{};
TX_SEMAPHORE locked{}, release_lock{}, finished{};
msg::channel<uint32_t> channel;
static bsp::dma::buffer<33> dma BSP_DMA_BUFFER;
void check(bool value) {
    ++pnx_core_test.checks;
    if (!value) { ++pnx_core_test.failures; pnx_core_test.failed_groups |= 1U << pnx_core_test.stage; }
}
bool near(float a, float b, float tolerance=1e-5f) { return isfinite(a) && fabsf(a-b)<tolerance; }
int twice(int value) { return value*2; }
void hold_lock(ULONG) {
    tx_mutex_get(&channel.native_layout().lock, TX_WAIT_FOREVER);
    tx_semaphore_put(&locked);
    tx_semaphore_get(&release_lock, TX_WAIT_FOREVER);
    tx_mutex_put(&channel.native_layout().lock);
    tx_semaphore_put(&finished);
}
void run(ULONG) {
    pnx_core_test.stage=1;
    check(math::clamp(5,-1,1)==1 && math::clamp_loop(5,0,4)==1);
    check(math::quantize(0,-1,1,12)==2047 && near(math::dequantize(4095,-1,1,12),1));
    check(near(math::inv_sqrt(4),0.5f) && math::inv_sqrt(0)==0);
    auto vector=math::normalise(math::vector3{3,4,0});
    check(near(math::magnitude(vector),1) && near(vector.x,0.6f));
    auto angles=math::to_euler(math::quaternion{0.70710678f,0,0,0.70710678f});
    check(near(angles.yaw,math::pi/2));
    const float a[3][3]={{1,2,3},{0,1,4},{5,6,0}};
    float inverse[3][3]{}, identity[3][3]{}, transposed[3][3]{};
    check(math::inverse(a,inverse,1e-9f));
    math::multiply(a,inverse,identity); math::transpose(a,transposed);
    for (uint8_t r=0;r<3;++r) for(uint8_t c=0;c<3;++c) check(near(identity[r][c],r==c?1:0));
    check(transposed[2][1]==4);
    const float zero[3][3]{}; check(!math::inverse(zero,inverse,0));

    pnx_core_test.stage=2;
    control::pid pid(2,0,0,10,5); pid.ref=3; pid.update(); check(near(pid.result,6));
    pid.ref=100; pid.update(); check(near(pid.result,10)); pid.clear(); check(pid.result==0);
    filter::iir low(2,1,1); check(near(low.update(1),0.25f)); check(near(low.update(0),0.5f));
    low.reset(); check(near(low.update(1),0.25f));
    auto high=filter::biquad::high_pass(25,100);
    check(near(high.update(1),1/(2+sqrtf(2))));
    for(uint16_t i=0;i<100;++i) high.update(1);
    check(near(high.update(1),0));
    filter::kalman_1d kalman; check(near(kalman.update(1),0.021f/0.564f));

    pnx_core_test.stage=3;
    const uint8_t digits[]="123456789";
    check(crc::get_crc16_checksum(digits,9,0xffff)==0x6f91);
    check(crc::get_crc16_modbus_checksum(digits,9,0xffff)==0x4b37);
    uint8_t frame[12]={'a','b','c'};
    crc::append_crc8_checksum(frame,4); check(crc::verify_crc8_checksum(frame,4));
    frame[1]^=1; check(!crc::verify_crc8_checksum(frame,4));
    auto callback=core::callback<int(int)>::bind<&twice>(); check(callback && callback(7)==14);

    pnx_core_test.stage=4;
    check(msg::init(channel)==types::status::ok);
    auto sub=msg::subscribe(channel); check(sub.valid()); uint32_t payload=99;
    check(msg::read(sub,payload)==types::status::busy && payload==99);
    check(msg::publish(channel,uint32_t{1})==types::status::ok);
    check(msg::publish(channel,uint32_t{2})==types::status::ok);
    check(msg::read(sub,payload)==types::status::ok && payload==2);
    check(!msg::available(sub));
    check(tx_semaphore_create(&locked,const_cast<char*>("locked"),0)==TX_SUCCESS);
    check(tx_semaphore_create(&release_lock,const_cast<char*>("release"),0)==TX_SUCCESS);
    check(tx_semaphore_create(&finished,const_cast<char*>("finished"),0)==TX_SUCCESS);
    check(tx_thread_create(&contender,const_cast<char*>("contender"),hold_lock,0,
        contender_stack,sizeof(contender_stack),4,4,TX_NO_TIME_SLICE,TX_AUTO_START)==TX_SUCCESS);
    check(tx_semaphore_get(&locked,100)==TX_SUCCESS);
    check(msg::publish(channel,uint32_t{3})==types::status::busy);
    check(msg::read(sub,payload)==types::status::busy && payload==2);
    tx_semaphore_put(&release_lock); check(tx_semaphore_get(&finished,100)==TX_SUCCESS);
    check(msg::publish(channel,uint32_t{4})==types::status::ok);
    check(msg::read(sub,payload)==types::status::ok && payload==4);

    pnx_core_test.stage=5;
    check(bsp::dwt::init()==types::status::ok);
    auto began=bsp::dwt::timeline_us(); tx_thread_sleep(2);
    check(bsp::dwt::timeline_us()>began);
    check(bsp::dma::valid(dma.view()) && !bsp::dma::valid({}));
    check(dma.capacity()>=dma.logical_size());

    pnx_core_test.stage=6;
    logging::init(); pnx_core_test.boot_count=logging::state().boot_count;
    logging::record record{};
    if (logging::read_latest(0,record)==types::status::ok && strcmp(record.message,"target marker")==0) {
        pnx_core_test.prior_boot=record.value;
        pnx_core_test.retained_marker=pnx_core_test.boot_count==uint32_t(record.value)+1;
    }
    check(logging::clear()==types::status::ok);
    logging::set_level(logging::level::error);
    check(logging::write(logging::level::debug,"filtered")==types::status::ok);
    check(logging::state().write_count==0);
    for(uint16_t i=0;i<params::logging::capacity+3;++i)
        check(logging::write(logging::level::error,"ring",types::status::error,i)==types::status::ok);
    check(logging::state().retained_count==params::logging::capacity);
    check(logging::read_latest(0,record)==types::status::ok && record.value==params::logging::capacity+2);
    check(logging::write(logging::level::error,nullptr)==types::status::invalid_arg);
    uint32_t before=DWT->CYCCNT;
    const auto log_status=logging::write(logging::level::error,"target marker",types::status::ok,pnx_core_test.boot_count);
    pnx_core_test.logger_cycles=DWT->CYCCNT-before;
    check(log_status==types::status::ok);

    pnx_core_test.stage=7;
    static ahrs::quaternion_ekf original;
    static ahrs::tactical::ekf tactical;
    tactical.reset();
    for(uint16_t i=0;i<2000;++i) {
        original.update_kalman(0,0,math::pi/4,0,0,9.80665f,0.001f);
        tactical.update(0,0,math::pi/4,0,0,9.80665f,0.001f);
    }
    check(near(original.yaw,math::pi/2,0.005f));
    check(near(tactical.state().yaw,math::pi/2,0.005f));
    pnx_core_test.stage=8; pnx_core_test.done=1;
}
}
extern "C" void app_start() {
    pnx_core_test.clock_hz=SystemCoreClock;
    if(tx_thread_create(&thread,const_cast<char*>("core test"),run,0,stack,sizeof(stack),
        5,5,TX_NO_TIME_SLICE,TX_AUTO_START)!=TX_SUCCESS) {
        pnx_core_test.failures=1; pnx_core_test.done=2;
    }
}
