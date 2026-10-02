#include "iir.hpp"
#include "kalman_1d.hpp"
#include <cmath>
#include <stdexcept>
void near(float a, float b) { if (std::abs(a-b)>1e-6f) throw std::runtime_error("filter state"); }
static_assert(sizeof(filter::iir) == 32);
static_assert(sizeof(filter::kalman_1d) == 16);
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
}
