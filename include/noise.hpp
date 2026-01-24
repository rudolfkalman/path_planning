#ifndef NOISE_HPP
#define NOISE_HPP

#include <random>
#include <vector>

namespace Noise {
class Noise_Generator {
public:
  Noise_Generator(double mu, double sig);
  double generate_gaussian_noise();
private:
  std::random_device seed;
  std::mt19937 engine;

  double mu = 0.0;
  double sig = 1.0;
  std::normal_distribution<double> dist;
};

} // namespace Noise

#endif
