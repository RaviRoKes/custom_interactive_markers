// #include <custom_interactive_markers/mt_rviz_ui.hpp>
// #include <rviz_common/logging.hpp>
// #include <rviz_common/properties/string_property.hpp>
// #include <rviz_common/properties/status_property.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>

// // RViz related includes
// #include <interactive_markers/interactive_marker_server.hpp>
// #include <interactive_markers/menu_handler.hpp>
// #include <rviz_common/visualization_manager.hpp>

// // Qt libraries (GUI)
// #include <QVBoxLayout>
// #include <QLineEdit>
// #include <QPushButton>
// #include <QLabel>
// #include <QDoubleSpinBox>

// namespace custom_interactive_markers
// {
//     MTRvizUI::MTRvizUI(QWidget *parent)
//         : rviz_common::Panel(parent),
//           node_(std::make_shared<rclcpp::Node>("mt_rviz_ui_node")),
//           tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
//           interactive_marker_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mt_rviz_ui_server", node_))
//     //  interactive_marker_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_marker_server"))
//     {
//         // Layout to organize UI components
//         QVBoxLayout *layout = new QVBoxLayout;

//         // Frame name and parent frame UI
//         QLabel *frame_name_label = new QLabel("Frame Name:", this);
//         layout->addWidget(frame_name_label);
//         frame_name_input_ = new QLineEdit(this);
//         frame_name_input_->setPlaceholderText("Enter frame name");
//         layout->addWidget(frame_name_input_);

//         QLabel *parent_frame_label = new QLabel("Parent Frame:", this);
//         layout->addWidget(parent_frame_label);
//         parent_frame_input_ = new QLineEdit(this);
//         parent_frame_input_->setPlaceholderText("Enter parent frame name");
//         layout->addWidget(parent_frame_input_);

//         // Cylinder dimensions input
//         QLabel *radius_label = new QLabel("Cylinder Radius:", this);
//         layout->addWidget(radius_label);
//         radius_input_ = new QDoubleSpinBox(this);
//         radius_input_->setRange(0.0, 10.0);
//         layout->addWidget(radius_input_);

//         QLabel *height_label = new QLabel("Cylinder Height:", this);
//         layout->addWidget(height_label);
//         height_input_ = new QDoubleSpinBox(this);
//         height_input_->setRange(0.0, 10.0);
//         layout->addWidget(height_input_);

//         // Publish button
//         publish_button_ = new QPushButton("Publish Frame", this);
//         layout->addWidget(publish_button_);
//         connect(publish_button_, &QPushButton::clicked, this, &MTRvizUI::publishFrame);

//         // Set the layout for the panel
//         setLayout(layout);

//         // Create the interactive markers grid
//         createInteractiveMarkers();
//     }

//     void MTRvizUI::onInitialize()
//     {
//         rviz_common::Panel::onInitialize();
//         RCLCPP_INFO(node_->get_logger(), "MTRvizUI plugin initialized.");
//     }

//     void MTRvizUI::createInteractiveMarkers()
//     {
//         // Create an InteractiveMarker
//         visualization_msgs::msg::InteractiveMarker int_marker;
//         int_marker.header.frame_id = "base_frame";
//         int_marker.name = "example_marker";
//         int_marker.description = "Example Interactive Marker";

//         // Set the position of the marker
//         int_marker.pose.position.x = 0.0;
//         int_marker.pose.position.y = 0.0;
//         int_marker.pose.position.z = 0.0;

//         // Add controls to the marker (e.g., a simple button control)
//         visualization_msgs::msg::InteractiveMarkerControl control;
//         control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
//         control.always_visible = true;

//         // Attach a marker (e.g., a cylinder) to the control
//         visualization_msgs::msg::Marker marker = createCylinderMarker(0.5, 1.0); // Assuming you already have this function
//         control.markers.push_back(marker);

//         // Add the control to the InteractiveMarker
//         int_marker.controls.push_back(control);

//         // Add the InteractiveMarker to the server
//         interactive_marker_server_->insert(int_marker);
//         interactive_marker_server_->applyChanges();
//     }

//     void MTRvizUI::publishFrame()
//     {
//         RCLCPP_INFO(node_->get_logger(), "Publish button pressed.");
//         std::string frame_name = frame_name_input_->text().toStdString();
//         std::string parent_frame = parent_frame_input_->text().toStdString();

//         if (frame_name.empty() || parent_frame.empty())
//         {
//             RCLCPP_ERROR(node_->get_logger(), "Frame name or parent frame cannot be empty.");
//             return;
//         }

//         // Publish the transformation between the frames
//         geometry_msgs::msg::TransformStamped transform;
//         transform.header.stamp = node_->now();
//         transform.header.frame_id = parent_frame;
//         transform.child_frame_id = frame_name;
//         transform.transform.translation.x = 0.0;
//         transform.transform.translation.y = 0.0;
//         transform.transform.translation.z = 0.0;
//         transform.transform.rotation.x = 0.0;
//         transform.transform.rotation.y = 0.0;
//         transform.transform.rotation.z = 0.0;
//         transform.transform.rotation.w = 1.0;

//         tf_broadcaster_->sendTransform(transform);
//         RCLCPP_INFO(node_->get_logger(), "Published frame \"%s\" under parent frame \"%s\".",
//                     frame_name.c_str(), parent_frame.c_str());

//         // Add cylinders as interactive markers
//         addCylinders();
//     }

//     void MTRvizUI::addCylinders()
//     {
//         double radius = radius_input_->value();
//         double height = height_input_->value();

//         // Loop through all interactive markers and add a cylinder at their position
//         for (int i = 0; i < 5; ++i)
//         {
//             for (int j = 0; j < 10; ++j)
//             {
//                 std::string marker_name = "box_" + std::to_string(i) + "_" + std::to_string(j);
//                 visualization_msgs::msg::InteractiveMarker int_marker;
//                 int_marker.header.frame_id = "world";
//                 int_marker.pose.position.x = i * 2.0;
//                 int_marker.pose.position.y = j * 2.0;
//                 int_marker.pose.position.z = height / 2.0;

//                 // Add a cylinder at the position of each marker
//                 visualization_msgs::msg::InteractiveMarkerControl control;
//                 control.name = "cylinder";
//                 control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
//                 control.always_visible = true;

//                 // Here we define the cylinder properties
//                 control.markers.push_back(createCylinderMarker(radius, height));

//                 // Insert the interactive marker
//                 int_marker.controls.push_back(control);
//                 interactive_marker_server_->insert(int_marker);
//             }
//         }

//         // Apply changes to the server
//         interactive_marker_server_->applyChanges();
//     }

//     visualization_msgs::msg::Marker MTRvizUI::createCylinderMarker(double radius, double height)
//     {
//         visualization_msgs::msg::Marker cylinder_marker;
//         cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
//         cylinder_marker.scale.x = radius * 2; // Diameter
//         cylinder_marker.scale.y = radius * 2;
//         cylinder_marker.scale.z = height;

//         // Set color of the cylinder
//         cylinder_marker.color.r = 0.0f;
//         cylinder_marker.color.g = 0.0f;
//         cylinder_marker.color.b = 1.0f;
//         cylinder_marker.color.a = 1.0f;

//         return cylinder_marker;
//     }

// } // namespace custom_interactive_markers

// // Register the RViz plugin
// #include <pluginlib/class_list_macros.hpp>
// PLUGINLIB_EXPORT_CLASS(custom_interactive_markers::MTRvizUI, rviz_common::Panel)




#include <custom_interactive_markers/mt_rviz_ui.hpp>
#include <rviz_common/logging.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// RViz related includes
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rviz_common/visualization_manager.hpp>

// Qt libraries (GUI)
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QMessageBox> // Include for error messages

namespace custom_interactive_markers
{
    MTRvizUI::MTRvizUI(QWidget *parent)
        : rviz_common::Panel(parent),
          node_(std::make_shared<rclcpp::Node>("mt_rviz_ui_node")),
          tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(node_)),
          interactive_marker_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mt_rviz_ui_server", node_))
    {
        // Layout to organize UI components
        QVBoxLayout *layout = new QVBoxLayout;

        // Frame name and parent frame UI
        QLabel *frame_name_label = new QLabel("Frame Name:", this);
        layout->addWidget(frame_name_label);
        frame_name_input_ = new QLineEdit(this);
        frame_name_input_->setPlaceholderText("Enter frame name");
        layout->addWidget(frame_name_input_);

        QLabel *parent_frame_label = new QLabel("Parent Frame:", this);
        layout->addWidget(parent_frame_label);
        parent_frame_input_ = new QLineEdit(this);
        parent_frame_input_->setPlaceholderText("Enter parent frame name");
        layout->addWidget(parent_frame_input_);

        // Cylinder dimensions input
        QLabel *radius_label = new QLabel("Cylinder Radius:", this);
        layout->addWidget(radius_label);
        radius_input_ = new QDoubleSpinBox(this);
        radius_input_->setRange(0.1, 10.0); // Minimum radius should be positive
        layout->addWidget(radius_input_);

        QLabel *height_label = new QLabel("Cylinder Height:", this);
        layout->addWidget(height_label);
        height_input_ = new QDoubleSpinBox(this);
        height_input_->setRange(0.1, 10.0); // Minimum height should be positive
        layout->addWidget(height_input_);

        // Publish button
        publish_button_ = new QPushButton("Publish Frame", this);
        layout->addWidget(publish_button_);
        connect(publish_button_, &QPushButton::clicked, this, &MTRvizUI::publishFrame);

        // Set the layout for the panel
        setLayout(layout);

        // Create the interactive markers grid
        createInteractiveMarkers();
    }

    void MTRvizUI::onInitialize()
    {
        rviz_common::Panel::onInitialize();
        RCLCPP_INFO(node_->get_logger(), "MTRvizUI plugin initialized.");
    }

    void MTRvizUI::createInteractiveMarkers()
    {
        // Create an InteractiveMarker
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "base_frame";
        int_marker.name = "example_marker";
        int_marker.description = "Example Interactive Marker";

        // Set the position of the marker
        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        // Add controls to the marker (e.g., a simple button control)
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
        control.always_visible = true;

        // Attach a marker (e.g., a cylinder) to the control
        visualization_msgs::msg::Marker marker = createCylinderMarker(0.5, 1.0); // Assuming you already have this function
        control.markers.push_back(marker);

        // Add the control to the InteractiveMarker
        int_marker.controls.push_back(control);

        // Add the InteractiveMarker to the server
        interactive_marker_server_->insert(int_marker);
        interactive_marker_server_->applyChanges();
    }

    void MTRvizUI::publishFrame()
    {
        RCLCPP_INFO(node_->get_logger(), "Publish button pressed.");
        std::string frame_name = frame_name_input_->text().toStdString();
        std::string parent_frame = parent_frame_input_->text().toStdString();

        // Validate inputs
        if (frame_name.empty() || parent_frame.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Frame name or parent frame cannot be empty.");
            QMessageBox::warning(this, "Input Error", "Frame name or parent frame cannot be empty.");
            return;
        }

        // Publish the transformation between the frames
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = node_->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = frame_name;
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(node_->get_logger(), "Published frame \"%s\" under parent frame \"%s\".",
                    frame_name.c_str(), parent_frame.c_str());

        // Add cylinders as interactive markers
        addCylinders();
    }

    void MTRvizUI::addCylinders()
    {
        double radius = radius_input_->value();
        double height = height_input_->value();

        // Validate input
        if (radius <= 0.0 || height <= 0.0)
        {
            QMessageBox::warning(this, "Invalid Input", "Radius and Height must be positive values.");
            return;
        }

        // Loop through all interactive markers and add a cylinder at their position
        for (int i = 0; i < 5; ++i)
        {
            for (int j = 0; j < 10; ++j)
            {
                std::string marker_name = "box_" + std::to_string(i) + "_" + std::to_string(j);
                visualization_msgs::msg::InteractiveMarker int_marker;
                int_marker.header.frame_id = "world";
                int_marker.pose.position.x = i * 2.0;
                int_marker.pose.position.y = j * 2.0;
                int_marker.pose.position.z = height / 2.0;

                // Add a cylinder at the position of each marker
                visualization_msgs::msg::InteractiveMarkerControl control;
                control.name = "cylinder";
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::BUTTON;
                control.always_visible = true;

                // Here we define the cylinder properties
                control.markers.push_back(createCylinderMarker(radius, height));

                // Insert the interactive marker
                int_marker.controls.push_back(control);
                interactive_marker_server_->insert(int_marker);
            }
        }

        // Apply changes to the server
        interactive_marker_server_->applyChanges();
    }

    visualization_msgs::msg::Marker MTRvizUI::createCylinderMarker(double radius, double height)
    {
        visualization_msgs::msg::Marker cylinder_marker;
        cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        cylinder_marker.scale.x = radius * 2; // Diameter
        cylinder_marker.scale.y = radius * 2;
        cylinder_marker.scale.z = height;

        // Set color of the cylinder
        cylinder_marker.color.r = 0.0f;
        cylinder_marker.color.g = 0.0f;
        cylinder_marker.color.b = 1.0f;
        cylinder_marker.color.a = 1.0f;

        return cylinder_marker;
    }

} // namespace custom_interactive_markers

// Register the RViz plugin
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(custom_interactive_markers::MTRvizUI, rviz_common::Panel)
