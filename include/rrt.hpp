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
#include <assert.h>

class RRT : public Graph {
public:
  Obstacles obstacles;
  const double MAP_WIDTH;
  const double MAP_HEIGHT;
  const double ROBOT_RADIUS;
  //Obstacles obstacle;
  //TODO you need to put nodes/opened in Graph, and with start/end, etc getters.
  //TODO generalize the definitions
  //TODO replace max_iter with allowable proximity distance
  RRT(Node* begining, Node* target,  Obstacles obs, double map_width, double map_height, double step_size=0.05, int max_iter=3000, int epsilon=0.01, double robot_radius=0.009) :
    Graph(begining, target),
    MAP_WIDTH(map_width),
    MAP_HEIGHT(map_height),
    STEP_SIZE(step_size),
    MAX_ITER(max_iter),
    EPSILON(epsilon),
    ROBOT_RADIUS(robot_radius),
    last(new Node(begining)) {
    nodes.push_back(begining);
    for (int i =0; i < obs.size(); i++) {
      obstacles.push_back(obs[i]);
    }
    std::cout << "RRT inistantiated!" << std::endl;
  }

  void search() {
    int c=0;
    while (!reached() && c++<MAX_ITER) {
      //std::cout << " searching..." << std::endl;
      Node *rand = getRandomNode();
      //std::cout << " rand node: " << *rand << std::endl;
      Node *near=nearest(rand);
      //std::cout << " near node found: " << *near << std::endl;
      Node *sample=newConfig(rand, near);
      //std::cout << " new config: " << *sample << std::endl;
      bool intersect=false;
      for (int i = 0; i < obstacles.size() && !intersect; i++) {
        intersect=obstacles[i]->intersect(near->get_point(),
                                          sample->get_point(),
                                          ROBOT_RADIUS);
      }
      if(!intersect) {
        // if the new configuration sample doens't itersect
        // with the linaer near-sample, then adapt it.
        add(near, sample);
        std::cout << "|||--->added node: " << *sample << std::endl;
      }
    }
    //connect the last found node with the target
    Node *target = get_end();
    Node *near_to_target=nearest(target);
    add(near_to_target, target);
    std::cout << "finished!" << std::endl;
  }

  //TODO (fix) add nodes to graph instead
  std::vector<Node*> get_nodes() {
    return nodes;
  }

protected:
  double distance(Node *p, Node *q) {
    return std::abs(*p-*q);
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
    assert(x <= MAP_WIDTH/2 && x >= -1*MAP_WIDTH/2);
    assert(y <= MAP_HEIGHT/2 && y >= -1*MAP_HEIGHT/2);
    //
    Eigen::VectorXd  p_vec(2);
    p_vec << x, y;
    //std::cout << "scaled (" << x << "," << y << ")" << std::endl;
    return new Node(p_vec);
  }
  //
  Node* nearest(Node *target) {
    double min=1e9; //TODO (enh) add this to configuration
    Node *closest = nullptr;
    Node *cur = nullptr;
    double dist;
    for (int i=0; i < nodes.size(); i++) {
      cur=nodes[i];
      dist = distance(target, cur);
      if (dist < min) {
        min=dist;
        closest=cur;
      }
    }
    return closest;
  }
  //TODO (fix) adjust steps_size
  //such that newconfig is at x% of the distance
  //so you can calculate it.
  Node* newConfig(Node *q, Node *qNearest) {
    double x,y;
    x=INFINITY;
    y=INFINITY;
    Eigen::VectorXd to = q->get_point().vector();
    Eigen::VectorXd from = qNearest->get_point().vector();
    Eigen::VectorXd intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Eigen::VectorXd ret;
    /////////////////
    // TODO learn the highest step_size possible
    /////////////////
    double tmp_step=STEP_SIZE;
    do {
      ret = from + tmp_step * intermediate;
      x = ret(0);
      y = ret(1);
      tmp_step/=2;
    } while ((x > MAP_WIDTH/2 || x < -1*MAP_WIDTH/2) || 
             (y > MAP_HEIGHT/2 || y < -1*MAP_HEIGHT/2));
    
    assert(x <= MAP_WIDTH/2 && x >= -1*MAP_WIDTH/2);
    assert(y <= MAP_HEIGHT/2 && y >= -1*MAP_HEIGHT/2);
    
    /////////////////
    return new Node(ret);
  }
  Node* get_last() {
    //note that vector isn't guranteed to keep elements sorted, instead use last point.
    //TODO (fix) what is more appropriate container to use for nodes?
    return last;
  }
  void add(Node *parent, Node *sampled) {
    static int node_index=2; // begining has id 1
    sampled->set_id(node_index++);
    sampled->set_parent(parent);
    nodes.push_back(sampled);
    parent->add_edge(new Edge(sampled));
    //
    //TODO (res) is vector gurantee to maintain order?
    //last node is nodes[-1]
    last = new Node(sampled);
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
    bool reached=(dist <= EPSILON) ? true : false;
    //if(!reached)
    //std::cout << "||--> dist: " << dist << std::endl;
    return reached;
  }
  Node *last;
  std::vector<Node*> nodes;
  const double STEP_SIZE;
  const int MAX_ITER;
  const int EPSILON = 1; //TODO (adjust) END_DIST_THRESHOLD
};
