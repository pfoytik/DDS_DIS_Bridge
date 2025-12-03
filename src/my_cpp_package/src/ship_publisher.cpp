#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "my_cpp_package/msg/ship_entity_state.hpp"

using namespace std::chrono_literals;

class ShipPublisher : public rclcpp::Node
{
public:
  ShipPublisher()
  : Node("ship_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<my_cpp_package::msg::ShipEntityState>("ship_entity_states", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ShipPublisher::timer_callback, this));
    
    // Initialize ship positions and velocities
    ships_.resize(3);
    
    // Ship 1: USS Enterprise
    ships_[0].entity_id = 1001;
    ships_[0].entity_name = "USS Enterprise";
    ships_[0].latitude = 36.8970;    // Norfolk, VA area
    ships_[0].longitude = -76.0230;
    ships_[0].altitude = 0.0;
    ships_[0].heading = 45.0;        // degrees
    ships_[0].pitch = 0.0;
    ships_[0].roll = 0.0;
    ships_[0].velocity_x = 5.0;      // m/s eastward
    ships_[0].velocity_y = 5.0;      // m/s northward  
    ships_[0].velocity_z = 0.0;
    ships_[0].force_id = 1;          // Blue force
    
    // Ship 2: USS Nimitz
    ships_[1].entity_id = 1002;
    ships_[1].entity_name = "USS Nimitz";
    ships_[1].latitude = 36.9000;
    ships_[1].longitude = -76.0000;
    ships_[1].altitude = 0.0;
    ships_[1].heading = 90.0;
    ships_[1].pitch = 0.0;
    ships_[1].roll = 0.0;
    ships_[1].velocity_x = 8.0;      // m/s eastward
    ships_[1].velocity_y = 0.0;      // m/s northward
    ships_[1].velocity_z = 0.0;
    ships_[1].force_id = 1;          // Blue force
    
    // Ship 3: Red Force Destroyer  
    ships_[2].entity_id = 2001;
    ships_[2].entity_name = "Red Destroyer";
    ships_[2].latitude = 37.0000;
    ships_[2].longitude = -75.8000;
    ships_[2].altitude = 0.0;
    ships_[2].heading = 225.0;
    ships_[2].pitch = 0.0;
    ships_[2].roll = 0.0;
    ships_[2].velocity_x = -3.0;     // m/s westward
    ships_[2].velocity_y = -3.0;     // m/s southward
    ships_[2].velocity_z = 0.0;
    ships_[2].force_id = 2;          // Red force
  }

private:
  void timer_callback()
  {
    auto current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Update positions based on velocity (simple dead reckoning)
    double dt = 0.5; // 500ms in seconds
    
    for(auto& ship : ships_) {
      // Convert velocity to lat/lon delta (very simplified)
      double lat_delta = (ship.velocity_y * dt) / 111320.0; // meters to degrees lat
      double lon_delta = (ship.velocity_x * dt) / (111320.0 * cos(ship.latitude * M_PI / 180.0)); // meters to degrees lon
      
      ship.latitude += lat_delta;
      ship.longitude += lon_delta;
      ship.timestamp = current_time;
      
      // Publish the updated ship state
      publisher_->publish(ship);
      
      RCLCPP_INFO(this->get_logger(), "Published ship %d: lat=%.6f, lon=%.6f", 
                  ship.entity_id, ship.latitude, ship.longitude);
    }
    
    count_++;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<my_cpp_package::msg::ShipEntityState>::SharedPtr publisher_;
  std::vector<my_cpp_package::msg::ShipEntityState> ships_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShipPublisher>());
  rclcpp::shutdown();
  return 0;
}