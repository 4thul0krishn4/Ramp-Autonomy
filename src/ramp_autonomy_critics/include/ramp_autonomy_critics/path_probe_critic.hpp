#ifndef RAMP_AUTONOMY_CRITICS__PATH_PROBE_CRITIC_HPP_
#define RAMP_AUTONOMY_CRITICS__PATH_PROBE_CRITIC_HPP_

#include <fstream>
#include <string>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/** Observer only - adds NO cost. Appends one CSV row per controller tick with what PathAlign/PathFollow see. */
class PathProbeCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  std::string log_path_;
  std::ofstream out_;
};

}  // namespace mppi::critics

#endif  // RAMP_AUTONOMY_CRITICS__PATH_PROBE_CRITIC_HPP_
