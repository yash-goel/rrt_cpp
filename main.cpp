#include "rrt.hpp"

int main() {

  RRT rrt("../input.txt");
  rrt.load();

  Node current_node = rrt.run_RRT();

  std::cout << "Extracting path . . .";
  std::vector<Node> RRT_path = rrt.return_path(current_node);

  rrt.plot(RRT_path);

  return 0;
}