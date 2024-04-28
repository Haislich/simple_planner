#include "planner.h"

/* #region Node Definition */
Node::Node(const MapCoord &coord, Node *p = nullptr)
    : MapCoord(coord), parent(p) {}

/* #endregion */

/* #region A* Definition */
BFS::BFS(Map map) : map(map) {}

std::vector<RvizCoord> BFS::reconstruct_path(Node *goal_node) {
  std::vector<RvizCoord> path;
  Node *current_node = goal_node;

  while (current_node != nullptr) {
    path.push_back(
        RvizCoord(current_node->x, current_node->y, map.get_height()));
    current_node = current_node->parent;
  }

  reverse(path.begin(), path.end());
  return path;
}

std::vector<RvizCoord> BFS::plan(MapCoord start) {
  if (map.get_element_at(start) == MapElement::Goal) {
    return std::vector<RvizCoord>(
        1, RvizCoord(start.x, start.y, map.get_height()));
  }

  // Frontier is a  fifo queue.
  std::queue<Node *> frontier;
  std::vector<std::vector<bool>> visited(
      map.get_height(), std::vector<bool>(map.get_width(), false));
  visited[start.x][start.y] = true;
  frontier.push(new Node(start));
  std::vector<std::pair<int, int>> directions = {
      {-1, 0},  // Up
      {1, 0},   // Down
      {0, -1},  // Left
      {0, 1}    // Right
  };
  while (!frontier.empty()) {
    Node *current_node = frontier.front();
    frontier.pop();
    if (map.get_element_at(*current_node) == MapElement::Goal) {
      return reconstruct_path(current_node);
    }
    for (const auto &direction : directions) {
      int x = current_node->x + direction.first;
      int y = current_node->y + direction.second;

      // Check if the new cell is within the matrix boundaries and not visited
      // yet
      if (x >= 0 && x < map.get_width() && y >= 0 && y < map.get_height() &&
          !visited[x][y] &&
          map.get_element_at(*current_node) != MapElement::Obstacle) {
        visited[x][y] = true;  // Mark the new cell as visited

        frontier.push(new Node(MapCoord(x, y, map.get_height()),
                               current_node));  // Enqueue the new cell
      }
    }
  }
  return {};
}

/* #endregion */