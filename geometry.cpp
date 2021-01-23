#include "geometry.hpp"
#include <math.h>

Node::Node(const float &x, const float &y) {
  v.x_ = x;
  v.y_ = y;
}

void Node::set_parent(Node parent) { parent_ = std::make_unique<Node>(parent); }

std::shared_ptr<Node> Node::get_parent() const { return parent_; }

float Node::x() const { return v.x_; }

float Node::y() const { return v.y_; }

float Node::get_distance(const float &x, const float &y) {
  return sqrt(pow((v.x_ - x), 2) + pow((v.y_ - y), 2));
}

bool Node::is_goal_reached(const Node &goal) {
  float dist = get_distance(goal.x(), goal.y());
  if (dist < 0.1)
    return true;
  else
    return false;
}
