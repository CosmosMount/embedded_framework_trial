#include "iir.hpp"
#include "kalman_1d.hpp"
#include <cmath>
#include <stdexcept>
void near(float a, float b) { if (std::abs(a-b)>1e-6f) throw std::runtime_error("filter state"); }
static_assert(sizeof(filter::iir) == 32);
static_assert(sizeof(filter::kalman_1d) == 16);
static_assert(sizeof(filter::biquad) == 36);
int main() {
    filter::iir lowpass(2.0f, 1.0f, 1.0f);
    near(lowpass.update(1), 0.25f);
    near(lowpass.update(0), 0.5f);
    near(lowpass.update(0), 0.25f);
    near(lowpass.update(0), 0);
    lowpass.reset(); near(lowpass.update(1), 0.25f);
    filter::kalman_1d estimator;
    near(estimator.update(1), 0.021f / 0.564f);
    estimator.set_process_noise(0);
    estimator.set_measurement_noise(0);
    near(estimator.update(3), 3);
    estimator.reset(); near(estimator.update(1), 0.021f / 0.564f);
    auto highpass = filter::biquad::high_pass(25, 100);
    // At one-quarter sample rate the Butterworth coefficients have closed forms.
    const float first = 1.0f / (2.0f + std::sqrt(2.0f));
    near(highpass.update(1), first);
    near(highpass.update(0), -2.0f * first);
    near(highpass.update(0), first * (1.0f - (2.0f - std::sqrt(2.0f)) * first));
    highpass = filter::biquad::high_pass(25, 100);
    for (unsigned i = 0; i < 100; ++i) highpass.update(1);
    near(highpass.update(1), 0);
    auto bypass = filter::biquad::high_pass(0, 100);
    for (float value : {1.0f, -3.0f, 0.0f, 1e10f, 1e-10f}) near(bypass.update(value), value);
}
