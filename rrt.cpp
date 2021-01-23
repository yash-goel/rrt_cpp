#include "rrt.hpp"
#include "matplotlibcpp.h"
#include <fstream>
#include <random>
#include <sstream>

namespace plt = matplotlibcpp;

RRT::RRT(const std::string &file) { config_file = file; };

void RRT::load() {
  std::cout << "Loading RRT configuration . . ." << std::endl;
  std::ifstream config_read(config_file);

  bool start_read = false;
  bool goal_read = false;
  float x, y;

  std::string line;
  if (config_read.is_open()) {
    std::vector<Vertex> polygon;
    while (std::getline(config_read, line)) {
      std::stringstream ss(line);

      // read the start position
      if (ss >> x >> y) {
        if (!start_read) {
          std::cout << "Reading start position . . ." << std::endl;
          start = {x, y};
          start_read = true;
        }
        // read goal position
        else if (!goal_read) {
          std::cout << "Reading goal position . . ." << std::endl;
          goal = {x, y};
          goal_read = true;

          // get sampling space
          get_sampling_box();
        }
        // read the obstacles
        else {
          // reading obstacle
          Vertex v = {x, y};
          polygon.push_back(v);
        }
      } else {
        // closing obstacle
        if (polygon.size() > 0) {
          polygon.push_back(polygon[0]);
          obstacles.push_back(polygon);
          polygon = std::vector<Vertex>();
        }
      }
    }
  } else {
    std::cout << "Could not open file" << std::endl;
  }
}

void RRT::add_to_visited(const Node &n) { visited_list.push_back(n); }

void RRT::add_obstacle(const std::vector<Vertex> &p) { obstacles.push_back(p); }

int RRT::visited_length() const { return visited_list.size(); }

int RRT::obstacles_length() const { return obstacles.size(); }

std::vector<Node> RRT::get_visited_list() const { return visited_list; }

std::vector<std::vector<Vertex>> RRT::get_obstacles() const {
  return obstacles;
}

Node RRT::get_goal() const { return goal; }

Node RRT::get_start() const { return start; }

int RRT::get_nearest_node_ind(const Vertex &r) {
  std::vector<float> distances;
  for (int i = 0; i < visited_list.size(); i++) {
    float distance = visited_list[i].get_distance(r.x_, r.y_);
    distances.push_back(distance);
  }
  int index = std::distance(
      distances.begin(), std::min_element(distances.begin(), distances.end()));

  return index;
}

bool RRT::check_edge(const Vertex &p, const Vertex &q, const Vertex &u,
                     const Vertex &v) {
  float s1_x = q.x_ - p.x_;
  float s1_y = q.y_ - p.y_;
  float s2_x = v.x_ - u.x_;
  float s2_y = v.y_ - u.y_;

  float s, t;

  s = (-s1_y * (p.x_ - u.x_) + s1_x * (p.y_ - u.y_)) /
      (-s2_x * s1_y + s1_x * s2_y);
  t = (s2_x * (p.y_ - u.y_) - s2_y * (p.x_ - u.x_)) /
      (-s2_x * s1_y + s1_x * s2_y);

  if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
    return true;
  }
  return false;
}

bool RRT::check_collision(const Node &p, const float &x, const float &y) {
  bool inside = false;
  Vertex v = {x, y};
  Vertex u = {p.x(), p.y()};
  for (int i = 0; i < obstacles.size(); i++) {
    std::vector<Vertex> obstacle = obstacles[i];
    for (int j = 0; j < obstacle.size() - 1; j++) {
      bool intersect = check_edge(obstacle[j], obstacle[j + 1], u, v);
      if (intersect) {
        inside = true;
        break;
      }
    }
  }
  return inside;
}

void RRT::get_sampling_box() {
  float minx = std::min(start.x(), goal.x()) - 1.0;
  float miny = std::min(start.y(), goal.y()) - 1.0;
  float maxx = std::max(start.x(), goal.x()) + 1.0;
  float maxy = std::max(start.y(), goal.y()) + 1.0;

  sampl_max = {maxx, maxy};
  sampl_min = {minx, miny};
}

Vertex RRT::get_sample() {
  // get random point on the area
  float rx =
      sampl_min.x_ +
      ((float)rand()) / (((float)RAND_MAX / (sampl_max.x_ - sampl_min.x_)));
  float ry =
      sampl_min.y_ +
      ((float)rand()) / (((float)RAND_MAX / (sampl_max.y_ - sampl_min.y_)));
  Vertex r = {rx, ry};
  return r;
}

Node RRT::run_RRT() {

  Node current_node = start;

  add_to_visited(current_node);

  std::cout << "Running RRT . . ." << std::endl;

  srand(time(0));

  while (!current_node.is_goal_reached(goal)) {
    Vertex r = get_sample();

    // get the nearest node index
    int near_ind = get_nearest_node_ind(r);

    Node parent_node = visited_list[near_ind];

    // move a little towards the point
    float dir_mag = sqrt(pow((r.x_ - parent_node.x()), 2) +
                         pow((r.y_ - parent_node.y()), 2));
    float next_x = parent_node.x() + 0.2 * ((r.x_ - parent_node.x()) / dir_mag);
    float next_y = parent_node.y() + 0.2 * ((r.y_ - parent_node.y()) / dir_mag);

    // check for collision before letting in

    if (check_collision(parent_node, next_x, next_y)) {
      continue;
    } else {
      Node next_node(next_x, next_y);
      next_node.set_parent(parent_node);

      add_to_visited(next_node);
      current_node = next_node;

      if (current_node.is_goal_reached(goal)) {
        std::cout << "Goal Reached . . ." << std::endl;
        break;
      }
    }
  }
  return current_node;
}

std::vector<Node> RRT::return_path(Node n) {
  std::vector<Node> path;
  path.push_back(n);
  while (!(n.x() == start.x() && n.y() == start.y())) {
    n = *n.get_parent();
    path.push_back(n);
  }
  return path;
}

void RRT::plot(const std::vector<Node> &RRT_path) {
  std::vector<float> rrt_x, rrt_y;

  for (int j = 0; j < RRT_path.size(); j++) {
    rrt_x.push_back(RRT_path[j].x());
    rrt_y.push_back(RRT_path[j].y());
  }

  plt::plot(rrt_x, rrt_y);
  for (int k = 0; k < obstacles_length(); k++) {
    std::vector<float> px, py;
    std::vector<Vertex> poly = get_obstacles()[k];
    for (int l = 0; l < poly.size(); l++) {
      px.push_back(poly[l].x_);
      py.push_back(poly[l].y_);
    }
    plt::plot(px, py);
  }
  plt::show();
}
