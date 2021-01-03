#include <stdlib.h>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <random>
#ifndef OBSTACLES
#include "obstacles.hpp"
#endif
#ifndef GRAPH_HDR
#include "graph.hpp"
#endif

class RRT : public Graph {
public:
  Obstacles obstacles;
  const double MAP_WIDTH;
  const double MAP_HEIGHT;
  const double ROBOT_RADIUS;
  //Obstacles obstacle;
  //TODO you need to put nodes/opened in Graph, and with start/end, etc getters.
  //TODO generalize the definitions
  RRT(std::vector<Node*> nodes_, Obstacles obs, double map_width, double map_height, int step_size=1, int max_iter=3000, int epsilon=0.01, double robot_radius=0) :
    Graph(nodes_[0], nodes_[nodes_.size()-1]),
    MAP_WIDTH(map_width),
    MAP_HEIGHT(map_height),
    STEP_SIZE(step_size),
    MAX_ITER(max_iter),
    EPSILON(epsilon),
    ROBOT_RADIUS(robot_radius) {
    for (int i =0; i < nodes_.size(); i++) {
      nodes.push_back(nodes_[i]);
    }
    for (int i =0; i < obs.size(); i++) {
      obstacles.push_back(obs[i]);
    }
    last = nodes_[0];
    std::cout << "RRT inistantiated!" << std::endl;
  }

  void search() {
    int c=0;
    //TODO (fix) choose appropriate point!, or add current_node variable to rrt 
    nodes.push_back(getRandomNode());
    do {
      //std::cout << " searching..." << std::endl;
      Node *rand = getRandomNode();
      //std::cout << " rand node: " << *rand << std::endl;
      Node *near=nearest(rand);
      //std::cout << " near node found: " << *near << std::endl;
      Node *close=newConfig(rand, near);
      //std::cout << " new config: " << *close << std::endl;
      for (auto obs : obstacles) {
        //TODO (fix) bad logic, it shouldn't itersect with any of the obstacles, assure this before adding!
        if (!obs->intersect(near->get_point(), close->get_point()),
            ROBOT_RADIUS)  {
          // if the new configuration close doens't itersect
          // with the linaer near-close, then adapt it.
          add(near, close);
          std::cout << "|||--->added node: " << *close << std::endl;
        }
      }
    } while (!reached() && c++<MAX_ITER);
    std::cout << "finished!" << std::endl;
  }

private:
  double distance(Node *p, Node *q) {
    return *p-*q;
  }
  /** 2D sampling
   *
   */
  //TODO (impl) 3d sampling
  Node* getRandomNode() {
    //std::cout << "getrandomnode" << std::endl;
    /*
      Node* ret;
      Vector2f point(drand48() * WORLD_WIDTH, drand48() * WORLD_HEIGHT);
      if (point.x() >= 0 && point.x() <= WORLD_WIDTH && point.y() >= 0 && point.y() <= WORLD_HEIGHT) {
      ret = new Node;
      ret->position = point;
      return ret;
      }
      return NULL;
    */
    double x, y;
    std::random_device rd;
    x=rd();
    y=rd();
    //std::cout << "getrandomNodes: " << "randomly sampled (" << x << "," << y << ")" << std::endl;
    //demean(shift) the point to the center of the map at (0,0)
    x -= (rd.max() - rd.min())/2;
    y -= (rd.max() - rd.min())/2;
    //scale point to map boundaries
    x = x/rd.max() * MAP_WIDTH; 
    y = y/rd.max() * MAP_HEIGHT;
    //
    Eigen::VectorXd  p_vec(2);
    p_vec << x, y;
    std::cout << "scaled (" << x << "," << y << ")" << std::endl;
    return new Node(p_vec);
  }
  Node* nearest(Node *n) {
    double min=1e9; //TODO (enh) add this to configuration
    Node *closest = nullptr;
    for (int i=0; i < nodes.size(); i++) {
      double dist = distance(n, nodes[i]);
      min=std::min(dist, min);
      closest=nodes[i];
    }
    return closest;
  }
  //TODO (fix) adjust steps_size
  Node* newConfig(Node *q, Node *qNearest) {
    Eigen::VectorXd to = q->get_point().vector();
    Eigen::VectorXd from = qNearest->get_point().vector();
    Eigen::VectorXd intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Eigen::VectorXd ret = from + STEP_SIZE * intermediate;
    return new Node(ret);
  }
  Node* get_last() {
    return last;
  }
  void add(Node *qNearest, Node *qNew) {
    qNearest->add_edge(new Edge(qNew));
    qNew->set_parent(qNearest);
    nodes.push_back(qNew);
    //
    
    //TODO (res) is vector gurantee to maintain order?
    //last node is nodes[-1]
    last=qNew;
  }
  /** wither it reached the target node
   *
   * verify the distance between the target and current node = 0
   * @return true if reached, false otherwise.
   */
  bool reached() {
    //std::cout << "is reached? " << std::endl;
    //std::cout << "nodes size: " << nodes.size() << std::endl;
    //std::cout << "end node: " << *nodes[nodes.size()-1]<<std::endl;
    //std::cout << "target node: " << *get_end() << std::endl;
    auto dist = distance(get_end(), get_last());
    std::cout << "||--> dist: " << dist << std::endl;
    return (dist <= EPSILON) ? true : false;
  }
  Node* last;
  std::vector<Node*> nodes;
  const int STEP_SIZE;
  const int MAX_ITER;
  const int EPSILON = 1; //TODO (adjust) END_DIST_THRESHOLD
};
