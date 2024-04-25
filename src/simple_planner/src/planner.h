
#include <iostream>
#include <queue>
#include <unordered_set>
#include <vector>

#include "rviz_handler.h"
#include "server_map_handler.h"

// A node is a coordinate in the map with come other informations
class Node : public MapCoord {
 public:
  Node* parent;  // In order to know from where I'm coming.

  Node(const MapCoord& coord, Node* p);
};

class BFS {
  Map map;

 public:
  BFS(Map map);
  std::vector<RvizCoord> reconstruct_path(Node* goalNode);

  // A* algorithm
  std::vector<RvizCoord> plan(MapCoord start);
};
