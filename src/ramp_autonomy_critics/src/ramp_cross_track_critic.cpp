#include "ramp_autonomy_critics/ramp_cross_track_critic.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace mppi::critics
{

void RampCrossTrackCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(power_, "cost_power", 1);
  getParam(weight_, "cost_weight", 30.0f);
  getParam(corridor_half_width_, "corridor_half_width", 0.5f);
  getParam(corridor_weight_, "corridor_weight", 300.0f);
  getParam(trajectory_point_step_, "trajectory_point_step", 2);
  getParam(threshold_to_consider_, "threshold_to_consider", 0.5f);
  RCLCPP_INFO(
    logger_, "RampCrossTrackCritic instantiated: weight %f power %d corridor +/-%f m (weight %f)",
    weight_, power_, corridor_half_width_, corridor_weight_);
}

void RampCrossTrackCritic::score(CriticData & data)
{
  if (!enabled_ || data.path.x.shape(0) < 2 ||
    utils::withinPositionGoalTolerance(threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }

  const size_t n_path = data.path.x.shape(0);
  const float * px = data.path.x.data();
  const float * py = data.path.y.data();
  // precompute segments
  std::vector<float> sdx(n_path - 1), sdy(n_path - 1), sinv(n_path - 1);
  for (size_t i = 0; i + 1 < n_path; i++) {
    sdx[i] = px[i + 1] - px[i];
    sdy[i] = py[i + 1] - py[i];
    const float l2 = sdx[i] * sdx[i] + sdy[i] * sdy[i];
    sinv[i] = l2 > 1e-12f ? 1.0f / l2 : 0.0f;
  }

  const size_t batch = data.trajectories.x.shape(0);
  const size_t steps = data.trajectories.x.shape(1);
  const int stride = std::max(1, trajectory_point_step_);
  const float w2 = corridor_half_width_;

  for (size_t t = 0; t < batch; ++t) {
    float sum_d = 0.0f, sum_h = 0.0f;
    unsigned int n = 0;
    for (size_t p = stride; p < steps; p += stride) {
      const float tx = data.trajectories.x(t, p);
      const float ty = data.trajectories.y(t, p);
      float best = 1e30f;
      for (size_t i = 0; i + 1 < n_path; i++) {
        float u = ((tx - px[i]) * sdx[i] + (ty - py[i]) * sdy[i]) * sinv[i];
        u = std::min(1.0f, std::max(0.0f, u));
        const float cx = px[i] + u * sdx[i] - tx;
        const float cy = py[i] + u * sdy[i] - ty;
        const float d2 = cx * cx + cy * cy;
        if (d2 < best) {best = d2;}
      }
      const float d = std::sqrt(best);
      sum_d += d;
      const float over = d - w2;
      if (over > 0.0f) {sum_h += over * over;}
      n++;
    }
    if (n == 0u) {continue;}
    const float mean_d = sum_d / static_cast<float>(n);
    float c = (power_ > 1u) ? std::pow(mean_d * weight_, static_cast<float>(power_)) : mean_d * weight_;
    c += corridor_weight_ * sum_h / static_cast<float>(n);
    data.costs(t) += c;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::RampCrossTrackCritic, mppi::critics::CriticFunction)
