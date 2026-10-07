// MIT License

// Copyright (c) 2023 Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define BOOST_BIND_NO_PLACEHOLDERS

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "motor_controller/vel_parser_node.hpp"
#include "rover_msgs/msg/motors_command.hpp"

using std::placeholders::_1;
using namespace motor_controller;

VelParserNode::VelParserNode() : rclcpp::Node("vel_parser_node") {

  // declaring params
  this->declare_parameter<std::vector<double>>(
      "hardware_distances", std::vector<double>({23.0, 25.5, 28.5, 26.0}));

  this->declare_parameter<int>("enc_min", 250);
  this->declare_parameter<int>("enc_max", 750);

  // Speed [-100, +100] * 6 = [-600, +600]
  this->declare_parameter<int>("speed_factor", 10);

  this->declare_parameter<float>("linear_limit", 1.0);
  this->declare_parameter<float>("angular_limit", 1.0);
  this->declare_parameter<float>("angular_factor", 0.0);

  // Turn in place when only a rotation is commanded (|linear.x| below the
  // threshold). Without it a pure angular.z only steered the corner wheels:
  // the drive speed sqrt(linear^2 + (angular * angular_factor)^2) was 0, so
  // Nav2's rotate-to-heading never turned the rover and it never drove off.
  this->declare_parameter<bool>("pivot_enabled", true);
  this->declare_parameter<float>("pivot_linear_threshold", 0.02);
  // Farthest wheel speed floor while pivoting, normalized [0, 100] like the
  // linear speed. 50 (x speed_factor = 500) is what drives the rover well
  // from the keyboard (linear 0.5); weaker commands barely move the servos.
  this->declare_parameter<float>("pivot_min_speed", 50.0);
  // Same floor for driving: when the fastest wheel of a non-zero command is
  // below this, all wheels are scaled up together (the turn geometry stays).
  // Nav2 sent 0.3 m/s on 0.55 m arcs = 100-300 servo units, which did not move
  // the rover. 0 disables it.
  this->declare_parameter<float>("min_drive_speed", 50.0);

  // getting params
  std::vector<double> hardware_distances;
  this->get_parameter("hardware_distances", hardware_distances);

  this->get_parameter("enc_min", this->enc_min);
  this->get_parameter("enc_max", this->enc_max);
  this->get_parameter("speed_factor", this->speed_factor);

  this->get_parameter("linear_limit", this->linear_limit);
  this->get_parameter("angular_limit", this->angular_limit);
  this->get_parameter("angular_factor", this->angular_factor);
  this->get_parameter("pivot_enabled", this->pivot_enabled);
  this->get_parameter("pivot_linear_threshold", this->pivot_linear_threshold);
  this->get_parameter("pivot_min_speed", this->pivot_min_speed);
  this->get_parameter("min_drive_speed", this->min_drive_speed);

  this->d1 = hardware_distances[0];
  this->d2 = hardware_distances[1];
  this->d3 = hardware_distances[2];
  this->d4 = hardware_distances[3];

  // pubs and subs
  this->publisher = this->create_publisher<rover_msgs::msg::MotorsCommand>(
      "motors_command", 10);

  this->subscription = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&VelParserNode::callback, this, _1));
}

void VelParserNode::callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

  auto motors_command = rover_msgs::msg::MotorsCommand();

  if (this->pivot_enabled &&
      std::abs(msg->linear.x) < this->pivot_linear_threshold &&
      std::abs(msg->angular.z) > 1e-3) {
    std::vector<float> speeds, angles;
    this->calculate_pivot(
        std::clamp((float)msg->angular.z, -this->angular_limit, this->angular_limit),
        speeds, angles);
    std::vector<float> ticks = this->calculate_target_tick(angles);
    for (unsigned i = 0; i < speeds.size(); i++) {
      motors_command.drive_motor.push_back(int(speeds.at(i)) * this->speed_factor);
    }
    for (unsigned i = 0; i < ticks.size(); i++) {
      motors_command.corner_motor.push_back(int(ticks.at(i)));
    }
    this->publisher->publish(motors_command);
    return;
  }

  // normalize speed and steering
  float linear = std::min((float)msg->linear.x, this->linear_limit);
  float angular = std::min((float)msg->angular.z, this->angular_limit);
  float speed = sqrt(pow(linear, 2) + pow(angular * this->angular_factor, 2));

  if (msg->linear.x < 0) {
    speed *= -1;
  }

  float norm_speed = this->normalize(speed, -this->linear_limit,
                                     this->linear_limit, -100, 100);
  float norm_steering = this->normalize(angular, -this->angular_limit,
                                        this->angular_limit, -100, 100) *
                        -1;

  // calculate new speeds and steerings
  std::vector<float> new_speeds =
      this->calculate_velocity(norm_speed, norm_steering);
  std::vector<float> new_ticks =
      this->calculate_target_tick(this->calculate_target_deg(norm_steering));

  float fastest = 0.0f;
  for (float v : new_speeds) {
    fastest = std::max(fastest, std::abs(v));
  }
  if (fastest > 1e-3f && fastest < this->min_drive_speed) {
    for (float &v : new_speeds) {
      v *= this->min_drive_speed / fastest;
    }
  }

  // convert to int
  for (unsigned i = 0; i < new_speeds.size(); i++) {
    motors_command.drive_motor.push_back(int(new_speeds.at(i)) *
                                         this->speed_factor);
  }

  for (unsigned i = 0; i < new_ticks.size(); i++) {
    motors_command.corner_motor.push_back(int(new_ticks.at(i)));
  }

  // publish
  this->publisher->publish(motors_command);
}

float VelParserNode::normalize(float value, float old_min, float old_max,
                               float new_min, float new_max) {
  return (new_max - new_min) * ((value - old_min) / (old_max - old_min)) +
         new_min;
}

float VelParserNode::deg_to_tick(float deg, float e_min, float e_max) {
  float temp = (e_max + e_min) / 2 + ((e_max - e_min) / 90) * deg;

  if (temp < e_min)
    temp = e_min;
  else if (temp > e_max)
    temp = e_max;

  return temp;
}

float VelParserNode::radians_to_deg(float radians) {
  float pi = atan(1) * 4;
  return radians * 180.0 / pi;
}

std::vector<float> VelParserNode::calculate_velocity(float velocity,
                                                     float radius) {

  std::vector<float> new_velocity = {0, 0, 0, 0, 0, 0};
  float new_radius = 0;

  if (velocity == 0)
    return new_velocity;

  if (abs(radius) <= 5) {
    // No turning radius, all wheels same speed
    // Go ahead / Go back
    new_velocity = {velocity,  velocity,  velocity,
                    -velocity, -velocity, -velocity};

  } else {
    // Get radius in centimeters(MAX_RADIUS(255) to MIN_RADIUS(55))
    new_radius =
        MAX_RADIUS - (((MAX_RADIUS - MIN_RADIUS) * abs(radius)) / 100.0);

    float a = pow(this->d2, 2); // Back - D2
    float b = pow(this->d3, 2); // Front - D3

    float c = pow(new_radius + this->d1, 2); // Front / Back - Farthest
    float d = pow(new_radius - this->d1, 2); // Front / Back - Closest

    float e = new_radius - this->d4; // Center - Closest
    float f = new_radius + this->d4; // Center - Farthest

    float rx = 1;

    if (new_radius < 111) {
      // Front - Farthest wheel is the Farthest
      rx = sqrt(b + c);
    } else {
      // Center - Farthest wheel is the Farthest
      rx = f;
    }

    //  Get speed of each wheel
    float abs_v1 = abs(velocity) * sqrt(b + c) / rx;
    float abs_v2 = abs(velocity) * (f / rx);
    float abs_v3 = abs(velocity) * sqrt(a + c) / rx;
    float abs_v4 = abs(velocity) * sqrt(b + d) / rx;
    float abs_v5 = abs(velocity) * (e / rx);
    float abs_v6 = abs(velocity) * sqrt(a + d) / rx;

    if (velocity < 0) { //#Go back

      if (radius < 0) { // Turn Left
        new_velocity = {-abs_v4, -abs_v5, -abs_v6, abs_v1, abs_v2, abs_v3};

      } else { // Turn Right
        new_velocity = {-abs_v1, -abs_v2, -abs_v3, abs_v4, abs_v5, abs_v6};
      }

    } else { // Go ahead

      if (radius < 0) { // Turn Left
        new_velocity = {abs_v4, abs_v5, abs_v6, -abs_v1, -abs_v2, -abs_v3};

      } else { // Turn Right
        new_velocity = {abs_v1, abs_v2, abs_v3, -abs_v4, -abs_v5, -abs_v6};
      }
    }
  }

  // Set the speeds between the range[-max_speed, +max_speed]
  return new_velocity;
}

// Turn in place about the rover centre. The corner wheels are steered tangent
// to their circle around the centre and each wheel runs at a speed
// proportional to its distance from it; the farthest wheel moves at
// |angular| * r_max (m/s), normalized like the linear speed.
//
// Conventions of this node: wheel order front, middle, back, left side first;
// driving ahead is + on the left and - on the right; a steering angle is + to
// the right (turning left puts the front wheels negative). angular > 0 is a
// counter-clockwise (left) turn: all six wheels then run negative, clockwise
// all positive. Steering beyond the servo range is clamped by deg_to_tick.
void VelParserNode::calculate_pivot(float angular, std::vector<float> &speeds,
                                    std::vector<float> &angles) {
  const float r_front = std::hypot(this->d1, this->d3); // [cm]
  const float r_back = std::hypot(this->d1, this->d2);
  const float r_mid = this->d4;
  const float r_max = std::max({r_front, r_back, r_mid});

  float norm = std::abs(angular) * (r_max / 100.0f) / this->linear_limit * 100.0f;
  norm = std::min(std::max(norm, this->pivot_min_speed), 100.0f);
  const float sign = angular > 0 ? -1.0f : 1.0f;

  const float v_front = sign * norm * r_front / r_max;
  const float v_mid = sign * norm * r_mid / r_max;
  const float v_back = sign * norm * r_back / r_max;
  speeds = {v_front, v_mid, v_back, v_front, v_mid, v_back};

  const float a_front = this->radians_to_deg(atan(this->d3 / this->d1));
  const float a_back = this->radians_to_deg(atan(this->d2 / this->d1));
  // front-left, front-right, back-left, back-right
  angles = {a_front, -a_front, -a_back, a_back};
}

std::vector<float> VelParserNode::calculate_target_deg(float radius) {

  float new_radius = 0;
  std::vector<float> angles = {0, 0, 0, 0};

  // Scaled from MAX_RADIUS (255) to MIN_RADIUS (55) centimeters
  if (radius == 0) {
    new_radius = MAX_RADIUS;
  } else if (-100 <= radius && radius <= 100) {
    new_radius = MAX_RADIUS - abs(radius) * int(MAX_RADIUS / 100);
  } else {
    new_radius = MAX_RADIUS;
  }

  if (new_radius == MAX_RADIUS) {
    return angles;
  }

  // Turn Right - Turn Left
  // Front Left - Front Right
  float ang7 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) + this->d1)));

  // Front Right - Front Left
  float ang8 =
      this->radians_to_deg(atan(this->d3 / (abs(new_radius) - this->d1)));

  // Back Left - Back Right
  float ang9 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) + this->d1)));

  // Back Right - Back Left
  float ang10 =
      this->radians_to_deg(atan(this->d2 / (abs(new_radius) - this->d1)));

  if (radius < 0) { // Turn Left
    angles = {-ang8, -ang7, ang10, ang9};

  } else { // Turn Right
    angles = {ang7, ang8, -ang9, -ang10};
  }

  return angles;
}

std::vector<float>
VelParserNode::calculate_target_tick(std::vector<float> target_angles) {
  std::vector<float> tick;

  for (int i = 0; i < 4; i++) {
    tick.push_back(
        this->deg_to_tick(target_angles[i], this->enc_min, this->enc_max));
  }

  return tick;
}