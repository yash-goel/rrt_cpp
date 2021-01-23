#include "geometry.hpp"
#include <iostream>
#include <vector>

class RRT {
public:
  explicit RRT(const std::string &file);

  void load();

  void add_to_visited(const Node &n);

  void add_obstacle(const std::vector<Vertex> &p);

  int visited_length() const;

  int obstacles_length() const;

  std::vector<Node> get_visited_list() const;

  std::vector<std::vector<Vertex>> get_obstacles() const;

  Node get_goal() const;

  Node get_start() const;

  int get_nearest_node_ind(const Vertex &r);

  bool check_edge(const Vertex &p, const Vertex &q, const Vertex &u,
                  const Vertex &v);

  bool check_collision(const Node &p, const float &x, const float &y);

  void get_sampling_box();

  Vertex get_sample();

  Node run_RRT();

  std::vector<Node> return_path(Node n);

  void plot(const std::vector<Node> &RRT_path);

private:
  std::string config_file;
  Node start = {0, 0};
  Node goal = {0, 0};
  Vertex sampl_max;
  Vertex sampl_min;
  std::vector<Node> visited_list;
  std::vector<std::vector<Vertex>> obstacles;
};