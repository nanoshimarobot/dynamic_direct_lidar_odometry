#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>

namespace ba = boost::accumulators;

struct AccumulatorData
{
private:
  ba::accumulator_set<double, ba::features<ba::tag::mean, ba::tag::variance, ba::tag::min, ba::tag::max>> acc;

  double last_value;
  std::chrono::time_point<std::chrono::steady_clock> start_time;

public:
  void tick()
  {
    start_time = std::chrono::steady_clock::now();
  }

  void tock()
  {
    const std::chrono::time_point<std::chrono::steady_clock> end_time = std::chrono::steady_clock::now();
    const double wall_clock_ms =
        std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count() * 1000;
    acc(wall_clock_ms);
    last_value = wall_clock_ms;
  }

  double getMean()
  {
    return ba::mean(acc);
  }

  double getVariance()
  {
    return ba::variance(acc);
  }

  double getMin()
  {
    return ba::min(acc);
  }

  double getMax()
  {
    return ba::max(acc);
  }

  double getLast()
  {
    return last_value;
  }
};