#include "bmi088.hpp"
#include "bsp_spi.hpp"
#include "bsp_gpio.hpp"
#include "bsp_pwm.hpp"
#include "bsp_exti.hpp"
#include "bsp_dwt.hpp"
#include "duration_stats.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace
{
bool locked=false, selected=false, force_timeout=false, fail_transfer=false, fail_select=false;
std::vector<int> trace;
float heater_duty = 1.0f;
bool heater_started = false;
bool fail_pwm = false;
void require(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
}
UINT tx_mutex_create(TX_MUTEX*,CHAR*,UINT inherit)
{ require(inherit==TX_INHERIT,"priority inheritance required"); return TX_SUCCESS; }
UINT tx_mutex_get(TX_MUTEX*,ULONG ticks)
{
    require(ticks==params::bmi088::spi_lock_wait_ticks,"configured finite wait required");
    if(force_timeout) return TX_NOT_AVAILABLE;
    require(!locked,"transaction cannot nest"); locked=true; trace.push_back(1); return TX_SUCCESS;
}
UINT tx_mutex_put(TX_MUTEX*)
{ require(locked&&!selected,"unlock only after CS release"); locked=false;trace.push_back(5);return TX_SUCCESS; }
UINT tx_mutex_delete(TX_MUTEX*) { require(!locked,"delete only after release"); return TX_SUCCESS; }
UINT tx_thread_sleep(ULONG) { require(!locked,"sleep must be outside transaction");return TX_SUCCESS; }
namespace bsp::dwt { types::status init() { return types::status::ok; } }
namespace bsp::gpio
{
types::status set_active(output,bool active)
{
    require(locked,"CS change must be locked");
    if(active&&fail_select) return types::status::error;
    selected=active;trace.push_back(active?2:4);return types::status::ok;
}
}
namespace bsp::spi
{
types::status init(bus) { return types::status::ok; }
types::status transmit(bus,const uint8_t*,size_t,uint32_t)
{ require(locked&&selected,"TX must be inside CS transaction");trace.push_back(3);return fail_transfer?types::status::error:types::status::ok; }
types::status receive(bus,uint8_t* data,size_t len,uint32_t)
{ require(locked&&selected,"RX must be inside CS transaction");std::memset(data,0,len);trace.push_back(3);return types::status::ok; }
}
namespace bsp::pwm
{
types::status start(channel) { require(heater_duty==0.0f,"heater must start at zero"); heater_started=true; return types::status::ok;}
types::status stop(channel) {heater_started=false;return types::status::ok;}
types::status set_duty(channel,float duty) {if(fail_pwm) return types::status::error; heater_duty=duty;return types::status::ok;}
}
namespace bsp::exti
{
types::status attach(bsp::gpio::input,interrupt_callback callback)
{ return callback?types::status::ok:types::status::invalid_arg; }
}
int main()
{
    try
    {
        core::duration_stats stats;
        require(stats.average_us()==0.0f,"empty statistics");
        imu::bmi088 device;
        require(device.enable_heater(true)==types::status::ok && heater_started && heater_duty==0.0f,
                "heater starts at zero without optional timeout hardware");
        require(device.set_heater_duty(0.4f)==types::status::ok && heater_duty==0.4f,"temperature duty retained");
        require(device.enable_heater(false)==types::status::ok && !heater_started,"explicit heater stop retained");
        fail_pwm=true;
        require(device.enable_heater(true)==types::status::error && !heater_started,"failed zero duty prevents start");
        fail_pwm=false;
        float temperature=7;
        require(device.read_temperature(&temperature)==types::status::not_initialized,"uninitialized transport");
        require(device.initialize_transport()==types::status::ok,"transport init");
        require(device.read_temperature(&temperature)==types::status::ok && temperature==23,"temperature decode");
        require(trace==std::vector<int>({1,2,3,3,4,5}),"whole transaction lock ordering");
        require(std::isnan(device.data.temperature),"temperature worker cannot mutate acquisition cache");
        trace.clear();force_timeout=true;temperature=7;
        require(device.read_temperature(&temperature)==types::status::timeout && temperature==7 && trace.empty(),"timeout leaves bus and output untouched");
        force_timeout=false;fail_transfer=true;
        require(device.read_temperature(&temperature)==types::status::error && !locked&&!selected,"HAL failure releases bus");
        fail_transfer=false;fail_select=true;trace.clear();
        require(device.read_temperature(&temperature)==types::status::error && trace==std::vector<int>({1,4,5}),"failed CS skips SPI and releases mutex");
        fail_select=false;trace.clear();
        require(device.configure()==types::status::ok,"configuration waits outside mutex");
        require(device.shutdown()==types::status::ok,"transport shutdown");
        stats.record(25,10);
        stats.record(5,10);
        require(stats.count==2 && stats.last_us==5 && stats.min_us==5 && stats.max_us==25,
                "elapsed extrema and count");
        require(stats.total_us==30 && stats.average_us()==15.0f && stats.overrun_count==1,
                "elapsed average and budget");
        stats.record(10,10);
        require(stats.overrun_count==1,"equal budget is not an overrun");
        stats.record(UINT64_C(0x100000000),0);
        require(stats.last_us==UINT32_MAX && stats.overrun_count==1,"saturate elapsed; zero disables budget");
        return 0;
    }
    catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
