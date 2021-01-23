#include <iostream>
#include <memory>

struct Vertex {
  float x_;
  float y_;
};

class Node {
public:
  Node(const float &x, const float &y);

  void set_parent(Node parent);

  std::shared_ptr<Node> get_parent() const;

  float x() const;

  float y() const;

  float get_distance(const float &x, const float &y);

  bool is_goal_reached(const Node &goal);

private:
  Vertex v;
  std::shared_ptr<Node> parent_;
};
