#ifndef RAMP_AUTONOMY_CRITICS__RAMP_CROSS_TRACK_CRITIC_HPP_
#define RAMP_AUTONOMY_CRITICS__RAMP_CROSS_TRACK_CRITIC_HPP_

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"

namespace mppi::critics
{

/**
 * Explicit cross-track critic for ramp DESCENT.
 *
 * nav2's PathAlignCritic scores each trajectory point against the path point at the SAME integrated arc length,
 * only up to the trajectories' furthest reached path point, and returns zero cost at all unless that index is >=
 * offset_from_furthest. Here the cost is the plain geometric lateral error: for every sampled trajectory point,
 * the distance to the nearest SEGMENT of the (pruned) path. Two terms:
 *   cost_weight * mean(d)^cost_power                 - keeps trajectories on the centerline
 *   corridor_weight * mean(max(0, d - corridor)^2)   - steep wall beyond +/- corridor_half_width (explicit constraint)
 */
class RampCrossTrackCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  unsigned int power_{1};
  float weight_{30.0f};
  float corridor_half_width_{0.5f};
  float corridor_weight_{300.0f};
  int trajectory_point_step_{2};
  float threshold_to_consider_{0.5f};
};

}  // namespace mppi::critics

#endif  // RAMP_AUTONOMY_CRITICS__RAMP_CROSS_TRACK_CRITIC_HPP_
