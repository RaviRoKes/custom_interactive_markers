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

#include <rviz_interactive_markers/mt_rviz_ui.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// headers related to the interactive markers used in RViz
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// Qt libraries (GUI)
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox> // Include for error messages

#include "rviz_interactive_markers/mt_rviz_ui.hpp"
#include <QHBoxLayout>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rviz_interactive_markers
{

    MTRVizUI::MTRVizUI(QWidget *parent)
        : rviz_common::Panel(parent),
          rclcpp::Node("mt_rviz_ui"),
          grid_rows_(5),
          grid_cols_(10),
          cylinder_radius_(0.1),
          cylinder_height_(0.3)
    {
        // Marker server
        marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("grid_markers", this);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Create grid of markers
        createGrid();

        // RViz panel layout
        QVBoxLayout *layout = new QVBoxLayout;

        // Child frame input
        layout->addWidget(new QLabel("Child Frame:"));
        child_frame_input_ = new QLineEdit;
        layout->addWidget(child_frame_input_);

        // Parent frame input
        layout->addWidget(new QLabel("Parent Frame:"));
        parent_frame_input_ = new QLineEdit;
        layout->addWidget(parent_frame_input_);

        // Position inputs
        layout->addWidget(new QLabel("Translation (x, y, z):"));
        QHBoxLayout *position_layout = new QHBoxLayout;
        x_input_ = new QLineEdit;
        y_input_ = new QLineEdit;
        z_input_ = new QLineEdit;
        position_layout->addWidget(x_input_);
        position_layout->addWidget(y_input_);
        position_layout->addWidget(z_input_);
        layout->addLayout(position_layout);

        // Orientation inputs
        layout->addWidget(new QLabel("Rotation (roll, pitch, yaw):"));
        QHBoxLayout *rotation_layout = new QHBoxLayout;
        roll_input_ = new QLineEdit;
        pitch_input_ = new QLineEdit;
        yaw_input_ = new QLineEdit;
        rotation_layout->addWidget(roll_input_);
        rotation_layout->addWidget(pitch_input_);
        rotation_layout->addWidget(yaw_input_);
        layout->addLayout(rotation_layout);

        // Broadcast button
        broadcast_button_ = new QPushButton("Broadcast Transform");
        layout->addWidget(broadcast_button_);

        setLayout(layout);

        // Connect broadcast button to slot
        connect(broadcast_button_, SIGNAL(clicked()), this, SLOT(broadcastTransform()));
    }

    void MTRVizUI::onInitialize()
    {
        marker_server_->applyChanges();
    }

    void MTRVizUI::createGrid()
    {
        for (int row = 0; row < grid_rows_; ++row)
        {
            for (int col = 0; col < grid_cols_; ++col)
            {
                createBoxMarker(row, col);
            }
        }
    }

    void MTRVizUI::createBoxMarker(int row, int col)
    {
        std::string marker_name = "box_" + std::to_string(row) + "_" + std::to_string(col);

        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_link";
        int_marker.header.stamp = this->now();
        int_marker.name = marker_name;
        int_marker.description = "Grid Marker";
        int_marker.pose.position.x = col * 0.5;
        int_marker.pose.position.y = row * 0.5;
        int_marker.pose.position.z = 0.0;

        visualization_msgs::msg::Marker box_marker;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.scale.x = 0.4;
        box_marker.scale.y = 0.4;
        box_marker.scale.z = 0.1;
        box_marker.color.r = 0.5f;
        box_marker.color.g = 0.5f;
        box_marker.color.b = 0.5f;
        box_marker.color.a = 1.0f;

        visualization_msgs::msg::InteractiveMarkerControl box_control;
        box_control.always_visible = true;
        box_control.markers.push_back(box_marker);

        int_marker.controls.push_back(box_control);

        visualization_msgs::msg::InteractiveMarkerControl button_control;
        button_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        button_control.name = "toggle_cylinder";
        int_marker.controls.push_back(button_control);

        marker_server_->insert(int_marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        cylinder_visibility_[marker_name] = false;
    }

    void MTRVizUI::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            toggleCylinderVisibility(feedback->marker_name);
        }
    }

    void MTRVizUI::toggleCylinderVisibility(const std::string &marker_name)
    {
        if (cylinder_visibility_.find(marker_name) == cylinder_visibility_.end())
        {
            RCLCPP_WARN(get_logger(), "Unknown marker: %s", marker_name.c_str());
            return;
        }

        bool &visible = cylinder_visibility_[marker_name];
        visible = !visible;

        visualization_msgs::msg::Marker cylinder_marker;
        cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder_marker.scale.x = cylinder_radius_ * 2;
        cylinder_marker.scale.y = cylinder_radius_ * 2;
        cylinder_marker.scale.z = cylinder_height_;
        cylinder_marker.color.r = 0.0f;
        cylinder_marker.color.g = 0.5f;
        cylinder_marker.color.b = 0.8f;
        cylinder_marker.color.a = 1.0f;
        cylinder_marker.pose.position.z = cylinder_height_ / 2;

        // Erase the existing marker
        marker_server_->erase(marker_name);

        if (visible)
        {
            // Create a new marker with the cylinder
            visualization_msgs::msg::InteractiveMarker int_marker;
            int_marker.header.frame_id = "base_link";
            int_marker.header.stamp = this->now();
            int_marker.name = marker_name;
            int_marker.pose = cylinder_marker.pose;

            // Create a control and add the cylinder marker
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.always_visible = true;
            control.markers.push_back(cylinder_marker);

            // Add the control to the interactive marker
            int_marker.controls.push_back(control);

            // Insert the updated marker
            marker_server_->insert(int_marker, std::bind(&MTRVizUI::processFeedback, this, std::placeholders::_1));
        }

        // Apply the changes to update the markers
        marker_server_->applyChanges();
    }

    void MTRVizUI::broadcastTransform()
    {
        if (child_frame_input_->text().isEmpty() || parent_frame_input_->text().isEmpty())
        {
            RCLCPP_WARN(get_logger(), "Child or Parent frame name is empty.");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = parent_frame_input_->text().toStdString();
        transform.child_frame_id = child_frame_input_->text().toStdString();
        transform.transform.translation.x = x_input_->text().toDouble();
        transform.transform.translation.y = y_input_->text().toDouble();
        transform.transform.translation.z = z_input_->text().toDouble();

        double roll = roll_input_->text().toDouble();
        double pitch = pitch_input_->text().toDouble();
        double yaw = yaw_input_->text().toDouble();
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(get_logger(), "Broadcasted transform from %s to %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    }

} // namespace rviz_interactive_markers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_interactive_markers::MTRVizUI, rviz_common::Panel)
