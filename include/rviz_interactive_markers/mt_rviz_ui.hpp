// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP_
#define RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>

namespace rviz_interactive_markers {

class MTRVizUI : public rviz_common::Panel, public rclcpp::Node {
    Q_OBJECT

public:
    MTRVizUI(QWidget *parent = nullptr);

    void onInitialize() override;

private:
    void createGrid();
    void createBoxMarker(int row, int col);
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
    void toggleCylinderVisibility(const std::string &marker_name);
    void broadcastTransform();

    // Interactive Marker Server
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server_;

    // TF Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Grid settings
    int grid_rows_;
    int grid_cols_;

    // Cylinder properties
    double cylinder_radius_;
    double cylinder_height_;

    // RViz UI elements
    QLineEdit *child_frame_input_;
    QLineEdit *parent_frame_input_;
    QLineEdit *x_input_;
    QLineEdit *y_input_;
    QLineEdit *z_input_;
    QLineEdit *roll_input_;
    QLineEdit *pitch_input_;
    QLineEdit *yaw_input_;
    QPushButton *broadcast_button_;

    // Cylinder visibility mapping
    std::unordered_map<std::string, bool> cylinder_visibility_;
};

}  // namespace rviz_interactive_markers

#endif  // RVIZ_INTERACTIVE_MARKERS_MT_RVIZ_UI_HPP_
