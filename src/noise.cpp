#include "noise.hpp"

namespace Noise {

Noise_Generator::Noise_Generator(double mu, double sig)
    : mu(mu), sig(sig), engine(seed()), dist(mu, sig) {}

double Noise_Generator::generate_gaussian_noise() {
  if (sig <= 0.0)
    return 0.0;
  return dist(engine);
}

} // namespace Noise
