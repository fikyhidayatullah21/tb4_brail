#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "irobot_create_msgs/msg/audio_note_vector.hpp"
#include "irobot_create_msgs/msg/audio_note.hpp"

struct Waypoint
{
    Waypoint(double x_in, double y_in) : x(x_in), y(y_in), z(0.0) {}
    Waypoint() : x(0.0), y(0.0), z(0.0) {}

    double x;
    double y;
    double z;
};

class PickPlaceNavigator : public rclcpp::Node
{
public:
    PickPlaceNavigator()
    : rclcpp::Node("pick_place_navigator"),
      current_goal_index_(0)
    {
        // Action client Nav2
        nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose");

        // Publisher untuk audio buzzer
        audio_pub_ = this->create_publisher<irobot_create_msgs::msg::AudioNoteVector>(
            "/cmd_audio",
            10);

        // Inisialisasi waypoint pick & place
        pick_point_  = Waypoint(-12.5, -13.19);
        place_point_ = Waypoint(  9.41,  11.21);

        waypoints_ = { pick_point_, place_point_ };

        RCLCPP_INFO(this->get_logger(), "Waiting for navigate_to_pose action server...");
        nav_client_->wait_for_action_server();
        RCLCPP_INFO(this->get_logger(), "Action server available. Start sending goals.");

        send_next_goal();
    }

private:
    // ===== Members =====
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Publisher<irobot_create_msgs::msg::AudioNoteVector>::SharedPtr audio_pub_;

    Waypoint pick_point_;
    Waypoint place_point_;
    std::vector<Waypoint> waypoints_;
    std::size_t current_goal_index_;

    // ===== Methods =====
    void send_next_goal()
    {
        if (current_goal_index_ >= waypoints_.size())
        {
            RCLCPP_INFO(this->get_logger(), "All goals have been reached.");
            return;
        }

        const auto &target = waypoints_[current_goal_index_];

        nav2_msgs::action::NavigateToPose::Goal goal_msg;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        goal_msg.pose.pose.position.x = target.x;
        goal_msg.pose.pose.position.y = target.y;
        goal_msg.pose.pose.orientation.z = target.z;
        goal_msg.pose.pose.orientation.w = 1.0;  // tanpa rotasi (yaw = 0)

        auto send_goal_options =
            rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            std::bind(&PickPlaceNavigator::handle_result, this, std::placeholders::_1);

        nav_client_->async_send_goal(goal_msg, send_goal_options);

        RCLCPP_INFO(
            this->get_logger(),
            "Sending goal #%zu: (x=%.2f, y=%.2f)",
            current_goal_index_ + 1, target.x, target.y);
    }

    void handle_result(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(this->get_logger(),
                        "Goal #%zu reached successfully!",
                        current_goal_index_ + 1);

            // Kirim bunyi sesuai posisi (pick: 1 kali, place: 2 kali)
            publish_audio_for_current_goal();

            // Lanjut ke goal berikutnya
            current_goal_index_++;
            send_next_goal();
            break;
        }

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),
                         "Goal #%zu aborted!",
                         current_goal_index_ + 1);
            // Coba lanjut ke goal berikutnya (opsional, bisa diubah sesuai kebutuhan)
            current_goal_index_++;
            send_next_goal();
            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),
                        "Goal #%zu canceled!",
                        current_goal_index_ + 1);
            break;

        default:
            RCLCPP_ERROR(this->get_logger(),
                         "Unknown result code for goal #%zu!",
                         current_goal_index_ + 1);
            break;
        }
    }

    void publish_audio_for_current_goal()
    {
        irobot_create_msgs::msg::AudioNoteVector audio_vector_msg;
        irobot_create_msgs::msg::AudioNote audio_note;

        // durasi 1.5 detik (1 detik + 0.5 detik)
        audio_note.max_runtime.sec = 1;
        audio_note.max_runtime.nanosec = 500000000;

        if (current_goal_index_ == 0)
        {
            // Di point "Pick": bunyi 1 kali
            audio_note.frequency = 523;  // ~C5
            audio_vector_msg.notes.push_back(audio_note);
        }
        else if (current_goal_index_ == 1)
        {
            // Di point "Place": bunyi 2 kali
            audio_note.frequency = 523;  // nada pertama
            audio_vector_msg.notes.push_back(audio_note);

            audio_note.frequency = 440;  // nada kedua
            audio_vector_msg.notes.push_back(audio_note);
        }

        if (!audio_vector_msg.notes.empty())
        {
            audio_pub_->publish(audio_vector_msg);
            RCLCPP_INFO(this->get_logger(),
                        "Published audio pattern for goal #%zu",
                        current_goal_index_ + 1);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

