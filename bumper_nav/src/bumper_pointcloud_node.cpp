#include <memory>
#include <cmath>
#include <limits>
#include <array>
#include <vector>
#include <cstdlib>  // per rand()

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "create_msgs/msg/bumper.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

class BumperPointCloudNode : public rclcpp::Node
{
public:
  BumperPointCloudNode()
  : Node("bumper_pointcloud_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    is_moving_backward_(false),
    backward_start_time_(0),
    show_bumper_points_(false),
    bumper_points_start_time_(0)
  {
    this->declare_parameter<float>("radius", 0.22);
    this->declare_parameter<float>("height", 0.0);
    this->declare_parameter<int>("light_detection_threshold", 200);
    this->declare_parameter<float>("light_range", 0.075);
    this->declare_parameter<float>("backward_speed", -0.1);
    this->declare_parameter<double>("backward_duration", 1.0);
    this->declare_parameter<double>("bumper_points_duration", 1.0);

    radius_ = this->get_parameter("radius").as_double();
    height_ = this->get_parameter("height").as_double();
    light_detection_threshold_ = this->get_parameter("light_detection_threshold").as_int();
    light_range_ = this->get_parameter("light_range").as_double();
    backward_speed_ = this->get_parameter("backward_speed").as_double();
    backward_duration_ = this->get_parameter("backward_duration").as_double();
    bumper_points_duration_ = this->get_parameter("bumper_points_duration").as_double();

    initializePoints();

    bumper_sub_ = this->create_subscription<create_msgs::msg::Bumper>(
      "/bumper", 10, std::bind(&BumperPointCloudNode::bumperCallback, this, _1));

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/bumper/pointcloud", 10);

    // Publisher per i comandi di velocità di emergenza
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // Timer aggiornato a 13.3Hz (75ms)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(75),
      std::bind(&BumperPointCloudNode::timerCallback, this));
  }

private:
  struct PointIndex {
    static constexpr size_t ContactFront = 0;
    static constexpr size_t ContactLeft = 1;
    static constexpr size_t ContactRight = 2;
    static constexpr size_t LightLeft = 3;
    static constexpr size_t LightLeftFront = 4;
    static constexpr size_t LightLeftCenter = 5;
    static constexpr size_t LightRight = 6;
    static constexpr size_t LightRightFront = 7;
    static constexpr size_t LightRightCenter = 8;
    static constexpr size_t Invalid = 9;
  };

  const float INVALID = std::numeric_limits<float>::quiet_NaN();

  float radius_;
  float height_;
  int light_detection_threshold_;
  float light_range_;
  float backward_speed_;
  double backward_duration_;
  double bumper_points_duration_;

  rclcpp::Subscription<create_msgs::msg::Bumper>::SharedPtr bumper_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::array<geometry_msgs::msg::Point, 10> points_;

  bool is_left_pressed_ = false;
  bool is_right_pressed_ = false;
  bool prev_left_pressed_ = false;
  bool prev_right_pressed_ = false;

  // Variabili per il movimento all'indietro
  bool is_moving_backward_;
  rclcpp::Time backward_start_time_;

  // Variabili per mantenere i punti del bumper visibili
  bool show_bumper_points_;
  rclcpp::Time bumper_points_start_time_;
  bool last_left_state_ = false;
  bool last_right_state_ = false;

  uint16_t light_signal_left_ = 0;
  uint16_t light_signal_front_left_ = 0;
  uint16_t light_signal_center_left_ = 0;
  uint16_t light_signal_right_ = 0;
  uint16_t light_signal_front_right_ = 0;
  uint16_t light_signal_center_right_ = 0;

void initializePoints()
{
  // Punto invalido NaN
  points_[PointIndex::Invalid].x = INVALID;
  points_[PointIndex::Invalid].y = INVALID;
  points_[PointIndex::Invalid].z = INVALID;

  // Posizioni dei punti di contatto del bumper - più lontani dal robot
  float bumper_distance = radius_ + light_range_ * 0.3; // Leggermente più vicini dei sensori di luce
  
  points_[PointIndex::ContactFront] = getPointOnRadius(bumper_distance, 0.0f, height_);
  points_[PointIndex::ContactLeft] = getPointOnRadius(bumper_distance, M_PI / 4.0f, height_);
  points_[PointIndex::ContactRight] = getPointOnRadius(bumper_distance, -M_PI / 4.0f, height_);

  // Posizioni dei sensori di luce - più lontani dei bumper
  float base_x = radius_ + light_range_ * 0.4;

  points_[PointIndex::LightLeft].x = base_x;
  points_[PointIndex::LightLeft].y = 0.12;
  points_[PointIndex::LightLeft].z = height_ + 0.05;

  points_[PointIndex::LightLeftFront].x = base_x + 0.01;
  points_[PointIndex::LightLeftFront].y = 0.06;
  points_[PointIndex::LightLeftFront].z = height_ + 0.05;

  points_[PointIndex::LightLeftCenter].x = base_x + 0.02;
  points_[PointIndex::LightLeftCenter].y = 0.02;
  points_[PointIndex::LightLeftCenter].z = height_ + 0.05;

  points_[PointIndex::LightRight].x = base_x;
  points_[PointIndex::LightRight].y = -0.12;
  points_[PointIndex::LightRight].z = height_ + 0.05;

  points_[PointIndex::LightRightFront].x = base_x + 0.01;
  points_[PointIndex::LightRightFront].y = -0.06;
  points_[PointIndex::LightRightFront].z = height_ + 0.05;

  points_[PointIndex::LightRightCenter].x = base_x + 0.02;
  points_[PointIndex::LightRightCenter].y = -0.02;
  points_[PointIndex::LightRightCenter].z = height_ + 0.05;
}

  geometry_msgs::msg::Point getPointOnRadius(float radius, float angle, float height)
  {
    geometry_msgs::msg::Point p;
    p.x = radius * cos(angle);
    p.y = radius * sin(angle);
    p.z = height;
    return p;
  }

  // Funzione per generare cluster di punti attorno a un punto centrale
  std::vector<geometry_msgs::msg::Point> generateClusterPoints(const geometry_msgs::msg::Point &center, float cluster_size = 0.05f, int num_points = 5)
  {
    std::vector<geometry_msgs::msg::Point> cluster;
    cluster.reserve(num_points);

    cluster.push_back(center);

    for (int i = 1; i < num_points; ++i) {
      geometry_msgs::msg::Point p;
      p.x = center.x + ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * cluster_size;
      p.y = center.y + ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * cluster_size;
      p.z = center.z + ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * cluster_size;
      cluster.push_back(p);
    }

    return cluster;
  }

  void startBackwardMovement()
  {
    is_moving_backward_ = true;
    backward_start_time_ = this->now();
    
    // Inizia anche a mostrare i punti del bumper
    show_bumper_points_ = true;
    bumper_points_start_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Bumper attivato! Movimento all'indietro per %.1f secondi", backward_duration_);
  }

  void updateBumperPointsVisibility()
  {
    if (show_bumper_points_) {
      auto current_time = this->now();
      double elapsed_time = (current_time - bumper_points_start_time_).seconds();
      
      if (elapsed_time >= bumper_points_duration_) {
        show_bumper_points_ = false;
      }
    }
  }
  void updateMovement()
  {
    // Pubblica comandi di velocità SOLO quando si sta muovendo all'indietro
    if (is_moving_backward_) {
      auto current_time = this->now();
      double elapsed_time = (current_time - backward_start_time_).seconds();
      
      geometry_msgs::msg::Twist cmd_vel;
      
      if (elapsed_time < backward_duration_) {
        // Continua a muoversi all'indietro
        cmd_vel.linear.x = backward_speed_;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
      } else {
        // Ferma il movimento e smetti di pubblicare
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
        
        is_moving_backward_ = false;
        RCLCPP_INFO(this->get_logger(), "Movimento all'indietro completato");
      }
    }
    // Non pubblicare nulla quando non si sta muovendo all'indietro
  }

  void bumperCallback(const create_msgs::msg::Bumper::SharedPtr msg)
  {
    // Salva gli stati precedenti
    prev_left_pressed_ = is_left_pressed_;
    prev_right_pressed_ = is_right_pressed_;
    
    // Aggiorna gli stati attuali
    is_left_pressed_ = msg->is_left_pressed;
    is_right_pressed_ = msg->is_right_pressed;

    // Rileva la pressione del bumper (transizione da false a true)
    bool bumper_just_pressed = false;
    if ((is_left_pressed_ && !prev_left_pressed_) || 
        (is_right_pressed_ && !prev_right_pressed_)) {
      bumper_just_pressed = true;
    }

    // Se il bumper è appena stato premuto e non ci stiamo già muovendo all'indietro
    if (bumper_just_pressed && !is_moving_backward_) {
      startBackwardMovement();
    }

    // Salva lo stato per i punti persistenti
    if (is_left_pressed_ || is_right_pressed_) {
      last_left_state_ = is_left_pressed_;
      last_right_state_ = is_right_pressed_;
    }

    // Aggiorna i segnali luminosi
    light_signal_left_ = msg->light_signal_left;
    light_signal_front_left_ = msg->light_signal_front_left;
    light_signal_center_left_ = msg->light_signal_center_left;

    light_signal_right_ = msg->light_signal_right;
    light_signal_front_right_ = msg->light_signal_front_right;
    light_signal_center_right_ = msg->light_signal_center_right;
  }

  void timerCallback()
  {
    // Aggiorna il movimento del robot
    updateMovement();
    
    // Aggiorna la visibilità dei punti del bumper
    updateBumperPointsVisibility();
    
    // Genera la point cloud
    std::vector<geometry_msgs::msg::Point> cloud_points;
    std::vector<geometry_msgs::msg::Point> base_points;

    // Aggiungi punti del bumper se devono essere mostrati (premuto o ancora visibili per durata)
    if (show_bumper_points_ || is_left_pressed_ || is_right_pressed_) {
      bool show_left = (show_bumper_points_ && last_left_state_) || is_left_pressed_;
      bool show_right = (show_bumper_points_ && last_right_state_) || is_right_pressed_;
      
      if (show_left && show_right) {
        // Entrambi i bumper: mostra sinistro, destro E centrale
        base_points.push_back(points_[PointIndex::ContactLeft]);
        base_points.push_back(points_[PointIndex::ContactRight]);
        base_points.push_back(points_[PointIndex::ContactFront]);
      } else if (show_left) {
        // Solo sinistro: mostra sinistro E centrale
        base_points.push_back(points_[PointIndex::ContactLeft]);
        base_points.push_back(points_[PointIndex::ContactFront]);
      } else if (show_right) {
        // Solo destro: mostra destro E centrale
        base_points.push_back(points_[PointIndex::ContactRight]);
        base_points.push_back(points_[PointIndex::ContactFront]);
      }
    }

    auto check_light = [&](int idx, uint16_t val) {
      if (val >= light_detection_threshold_) {
        base_points.push_back(points_[idx]);
      }
    };

    check_light(PointIndex::LightLeft, light_signal_left_);
    check_light(PointIndex::LightLeftFront, light_signal_front_left_);
    check_light(PointIndex::LightLeftCenter, light_signal_center_left_);
    check_light(PointIndex::LightRight, light_signal_right_);
    check_light(PointIndex::LightRightFront, light_signal_front_right_);
    check_light(PointIndex::LightRightCenter, light_signal_center_right_);

    if (base_points.empty()) {
      geometry_msgs::msg::Point invalid;
      invalid.x = INVALID;
      invalid.y = INVALID;
      invalid.z = INVALID;
      cloud_points.push_back(invalid);
    } else {
      for (const auto& pt : base_points) {
        auto cluster = generateClusterPoints(pt, 0.05f, 5);
        cloud_points.insert(cloud_points.end(), cluster.begin(), cluster.end());
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = rclcpp::Time(0);
    cloud_msg.header.frame_id = "base_link";

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(cloud_points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (auto &pt : cloud_points) {
      *iter_x = pt.x;
      *iter_y = pt.y;
      *iter_z = pt.z;
      ++iter_x; ++iter_y; ++iter_z;
    }

    pointcloud_pub_->publish(cloud_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BumperPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
