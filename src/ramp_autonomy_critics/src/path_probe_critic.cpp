#include "ramp_autonomy_critics/path_probe_critic.hpp"

#include <cmath>

namespace mppi::critics
{

void PathProbeCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);
  getParam(log_path_, "log_path", std::string("/tmp/ramp_path_probe.csv"));
  out_.open(log_path_, std::ios::out | std::ios::trunc);
  out_ << "t,robot_x,robot_y,robot_yaw,path_pts,path_len_m,furthest_idx,furthest_dist_m,max_traj_len_m,"
          "gate3,gate5,gate20,goal_dist_m\n";
  RCLCPP_INFO(logger_, "PathProbeCritic (observer, zero cost) logging to %s", log_path_.c_str());
}

void PathProbeCritic::score(CriticData & data)
{
  if (!enabled_ || !out_.is_open() || data.path.x.shape(0) < 2) {
    return;
  }
  // The same helper the real critics use - do not mutate cost; it caches into data, which is what PathAlign would compute.
  utils::setPathFurthestPointIfNotSet(data);
  const size_t furthest = *data.furthest_reached_path_point;
  const size_t n = data.path.x.shape(0);
  float len = 0.0f, furthest_dist = 0.0f;
  for (size_t i = 1; i < n; i++) {
    const float dx = data.path.x(i) - data.path.x(i - 1), dy = data.path.y(i) - data.path.y(i - 1);
    len += std::sqrt(dx * dx + dy * dy);
    if (i <= furthest) {furthest_dist = len;}
  }
  // longest sampled trajectory (integrated distance)
  float max_traj = 0.0f;
  const size_t batch = data.trajectories.x.shape(0), steps = data.trajectories.x.shape(1);
  for (size_t t = 0; t < batch; t += 8) {
    float s = 0.0f;
    for (size_t p = 1; p < steps; p++) {
      const float dx = data.trajectories.x(t, p) - data.trajectories.x(t, p - 1);
      const float dy = data.trajectories.y(t, p) - data.trajectories.y(t, p - 1);
      s += std::sqrt(dx * dx + dy * dy);
    }
    if (s > max_traj) {max_traj = s;}
  }
  const auto & pose = data.state.pose.pose;
  const double yaw = std::atan2(
    2.0 * (pose.orientation.w * pose.orientation.z), 1.0 - 2.0 * pose.orientation.z * pose.orientation.z);
  const double gd = std::hypot(pose.position.x - data.goal.position.x, pose.position.y - data.goal.position.y);
  double now = 0.0;
  if (auto node = parent_.lock()) {now = node->now().seconds();}
  out_ << now << ',' << pose.position.x << ',' << pose.position.y << ',' << yaw << ',' << n << ',' << len << ','
       << furthest << ',' << furthest_dist << ',' << max_traj << ',' << (furthest >= 3) << ',' << (furthest >= 5)
       << ',' << (furthest >= 20) << ',' << gd << '\n';
  out_.flush();
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::PathProbeCritic, mppi::critics::CriticFunction)
