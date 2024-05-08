#ifndef POSE_INTERACTIVE_MARKER_HPP__
#define POSE_INTERACTIVE_MARKER_HPP__

#include <Eigen/Dense>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/detail/interactive_marker__struct.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace handles {
class PoseHandle : public rclcpp::Node {
public:
    PoseHandle();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_publisher;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> _marker_server;
    visualization_msgs::msg::InteractiveMarker                    _marker_msg;
    geometry_msgs::msg::PoseStamped                               _current_pose_msg;
    rclcpp::TimerBase::SharedPtr                                  _publisher_timer;

    /**
     * @brief Callback that muves the RViz marker according to the interaction
     *
     * @param[input] fdbk Message that contains the current pose of the marker
     */
    void _on_motion_update(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&
                    fbk
    );

    void _start_publisher();
};

void prepare_motion_marker(
        visualization_msgs::msg::InteractiveMarker& marker,
        const double&                               sphere_size = 0.05
);

/**
 * @brief Add the interactive control (i.e., visualiser arrows) on a marker
 *
 * @param[input/output] marker  Marker to update
 * @param[input] axis Angle-axis direction
 */
void add_axis_control(
        visualization_msgs::msg::InteractiveMarker& marker, const Eigen::Vector3d& axis
);
}  // namespace handles

#endif  // POSE_INTERACTIVE_MARKER_HPP__
