#include "motion_control_handle/pose_handle.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/publisher.hpp>
#include <visualization_msgs/msg/detail/interactive_marker_control__struct.hpp>
#include <visualization_msgs/msg/detail/interactive_marker_feedback__struct.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

using geometry_msgs::msg::PoseStamped;
using handles::PoseHandle;
using interactive_markers::InteractiveMarkerServer;
using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;
using visualization_msgs::msg::Marker;

namespace params {
namespace name {
    const std::string rf_name      = "reference_frame";
    const std::string out_topic    = "target_topic";
    const std::string mrk_name     = "marker_name";
    const std::string mrk_ns       = "marker_namespace";
    const std::string publish_rate = "publish_rate";

}  // namespace name

namespace value {
    const std::string rf_name      = "world";
    const std::string out_topic    = "/target_pose";
    const std::string mrk_name     = "motion_control_handle";
    const std::string mrk_ns       = "";
    const double      publish_rate = 10;
}  // namespace value

}  // namespace params

//  __  __           _       _
// |  \/  | ___   __| |_   _| | ___
// | |\/| |/ _ \ / _` | | | | |/ _ \
// | |  | | (_) | (_| | |_| | |  __/
// |_|  |_|\___/ \__,_|\__,_|_|\___|
//  _____                 _   _
// |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
// | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
// |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
// |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//
void
handles::add_axis_control(InteractiveMarker& marker, const Eigen::Vector3d& axis) {
    // Create quaternion from the rotation around the provided axis
    Eigen::Quaterniond       q(Eigen::AngleAxisd(M_PI / 2, axis));
    InteractiveMarkerControl control;
    control.orientation.w = q.w();
    control.orientation.x = q.x();
    control.orientation.y = q.y();
    control.orientation.z = q.z();

    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);

    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
}

void
handles::prepare_motion_marker(InteractiveMarker& marker, const double& sphere_size) {
    Marker visual_marker;
    visual_marker.type    = Marker::SPHERE;
    visual_marker.scale.x = sphere_size;  //< Set sphere size
    visual_marker.scale.y = sphere_size;
    visual_marker.scale.z = sphere_size;
    visual_marker.color.r = 1.0;  //< Set default color
    visual_marker.color.g = 0.5;
    visual_marker.color.b = 0.0;
    visual_marker.color.a = 1.0;

    InteractiveMarkerControl marker_control;  // Create marker control
    marker_control.always_visible = true;
    marker_control.markers.push_back(visual_marker);  // Register visualisation
    marker.controls.push_back(marker_control);

    add_axis_control(marker, Eigen::Vector3d::UnitX());
    add_axis_control(marker, Eigen::Vector3d::UnitY());
    add_axis_control(marker, Eigen::Vector3d::UnitZ());
}

//  __  __                _
// |  \/  | ___ _ __ ___ | |__   ___ _ __
// | |\/| |/ _ \ '_ ` _ \| '_ \ / _ \ '__|
// | |  | |  __/ | | | | | |_) |  __/ |
// |_|  |_|\___|_| |_| |_|_.__/ \___|_|
//
//  _____                 _   _
// |  ___|   _ _ __   ___| |_(_) ___  _ __  ___
// | |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
// |  _|| |_| | | | | (__| |_| | (_) | | | \__ \
// |_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
//

#define DECLARE_NODE_PARAMS(param)                                                     \
    declare_parameter<decltype(params::value::param)>(                                 \
            params::name::param, params::value::param                                  \
    );

PoseHandle::PoseHandle() : rclcpp::Node("pose_motion_handle") {
    // Node parameter declaration
    DECLARE_NODE_PARAMS(rf_name);
    DECLARE_NODE_PARAMS(out_topic);
    DECLARE_NODE_PARAMS(mrk_name);
    DECLARE_NODE_PARAMS(mrk_ns);
    DECLARE_NODE_PARAMS(publish_rate);

    // Recover parameter value
    const std::string rf_name = get_parameter(params::name::rf_name).as_string();
    const std::string output_topic_name =
            get_parameter(params::name::out_topic).as_string();
    const std::string marker_name = get_parameter(params::name::mrk_name).as_string();
    std::string marker_namespace  = get_parameter(params::name::mrk_ns).as_string();

    if (marker_namespace.empty()) marker_namespace = marker_name;

    RCLCPP_INFO(
            get_logger(),
            "Creating marker '%s' with reference frame '%s' on namespace '%s'",
            marker_name.c_str(),
            rf_name.c_str(),
            marker_namespace.c_str()
    );

    // Set-up pose publisher
    if (!output_topic_name.empty()) {
        RCLCPP_INFO(
                get_logger(), "Pose broadcasted on topic %s", output_topic_name.c_str()
        );
        _pose_publisher =
                create_publisher<PoseStamped>(output_topic_name, rclcpp::QoS(10));
    } else _pose_publisher = nullptr;

    // Prepare marker
    _marker_msg.header.frame_id = rf_name;
    _marker_msg.header.stamp    = now();
    _marker_msg.name            = marker_name;
    _marker_msg.description     = "6D control handle: " + marker_name;
    prepare_motion_marker(_marker_msg, 0.05);

    // Configure marker server
    _marker_server = std::make_shared<InteractiveMarkerServer>(marker_namespace, this);
    _marker_server->insert(_marker_msg);
    _marker_server->setCallback(
            _marker_msg.name,
            [this](auto fbk) { this->_on_motion_update(fbk); },
            InteractiveMarkerFeedback::POSE_UPDATE
    );
    _marker_server->applyChanges();

    // Set-up reference frame on the message
    _current_pose_msg.header.frame_id = rf_name;

    _start_publisher();
}

void
PoseHandle::_on_motion_update(const InteractiveMarkerFeedback::ConstSharedPtr& fbk) {
    // Update RViz marker
    _marker_server->setPose(fbk->marker_name, fbk->pose);
    _marker_server->applyChanges();

    // Store position in message
    _current_pose_msg.pose         = fbk->pose;
    _current_pose_msg.header.stamp = now();
    _pose_publisher->publish(_current_pose_msg);
}

void
PoseHandle::_start_publisher() {
    using std::chrono::microseconds;

    const double pub_rate = get_parameter(params::name::publish_rate).as_double();
    if ((_pose_publisher == nullptr) || (pub_rate <= 0)) return;

    const microseconds dt_pub(static_cast<unsigned int>(1 / pub_rate * 1e6));
    auto pub_msg     = [this]() { _pose_publisher->publish(_current_pose_msg); };
    _publisher_timer = create_wall_timer(dt_pub, pub_msg);
    RCLCPP_INFO(get_logger(), "Starting publisher at a rate of %f.1Hz", pub_rate);
}

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseHandle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
