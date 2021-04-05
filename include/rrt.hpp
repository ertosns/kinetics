#pragma once
#define RRT_HDR
#include <stdlib.h>
#include <vector>
#include <memory>
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

using namespace std;

class RRT : public Graph {
public:

    Obstacles obstacles;
    const double MAP_WIDTH;
    const double MAP_HEIGHT;
    const double ROBOT_RADIUS;
    //Obstacles obstacle;
    RRT(shared_ptr<Node> begining, shared_ptr<Node> target,  Obstacles obs, double map_width, double map_height, double step_size=0.05, int max_iter=3000, int epsilon=0.01, double robot_radius=0.009) :
        Graph(begining, target),
        MAP_WIDTH(map_width),
        MAP_HEIGHT(map_height),
        STEP_SIZE(step_size),
        MAX_ITER(max_iter),
        EPSILON(epsilon),
        ROBOT_RADIUS(robot_radius),
        last(begining) {
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
            auto rand = getRandomNode();
            //std::cout << " rand node: " << *rand << std::endl;
            auto near=nearest(rand);
            //std::cout << " near node found: " << *near << std::endl;
            auto sample=newConfig(rand, near);
      //std::cout << " new config: " << *sample << std::endl;
      bool intersect=false;
      for (int i = 0; i < obstacles.size() && !intersect; i++) {
          intersect=obstacles[i]->intersect(near->get_point(),
                                            sample->get_point(),
                                            ROBOT_RADIUS);
      }
      if(!intersect) {
        // if the new configuration sample doens't intersect
        // with the line near-sample, then add it to the graph.
        add(near, sample);
        std::cout << "|||--->added node: " << *sample << std::endl;
      }
    }
    //connect the last found node with the target
    //TODO .get() temporarily
        auto target = get_end();
        auto near_to_target=nearest(target);
        add(near_to_target, target);
        std::cout << "finished!" << std::endl;
  }

  //TODO (fix) add nodes to graph instead
  std::vector<shared_ptr<Node>> get_nodes() {
    return nodes;
  }

protected:
    double distance(shared_ptr<Node> p, shared_ptr<Node> q) {
        return std::abs(*p-*q);
    }
  /** 2D sampling
   *
   */
  //TODO (impl) 3d sampling
  shared_ptr<Node> getRandomNode() {
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

    //assert(x <= MAP_WIDTH/2 && x >= -1*MAP_WIDTH/2);
    //assert(y <= MAP_HEIGHT/2 && y >= -1*MAP_HEIGHT/2);

    Eigen::VectorXd  p_vec(2);
    p_vec << x, y;
    //std::cout << "scaled (" << x << "," << y << ")" << std::endl;
    return make_shared<Node>(p_vec);
  }
    //
  shared_ptr<Node> nearest(shared_ptr<Node> target) {
    double min=1e9; //TODO (enh) add this to configuration
    shared_ptr<Node> closest = nullptr;
    shared_ptr<Node> cur = nullptr;
    double dist;
    for (auto &&node : nodes) {
        cur=node;
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
  shared_ptr<Node> newConfig(shared_ptr<Node> q, shared_ptr<Node> qNearest) {
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
      return make_shared<Node>(ret);
  }
    shared_ptr<Node> get_last() {
        //note that vector isn't guranteed to keep elements sorted, instead use last point.
        //TODO (fix) what is more appropriate container to use for nodes?
        return last;
    }
    void add(shared_ptr<Node> parent, shared_ptr<Node> sampled) {
        static int node_index=2; // begining has id 1
        sampled->set_id(node_index++);
        sampled->set_parent(std::make_shared<Node>(parent));
        nodes.push_back(sampled);
        std::shared_ptr<Edge> edge_ptr = std::make_shared<Edge>(sampled);
        parent->add_edge(edge_ptr);
        //TODO (res) is vector gurantee to maintain order?
        //last node is nodes[-1]
        last = sampled;
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
        //TODO .get temporarily
        auto dist = distance(get_end(), get_last());
        bool reached=(dist <= EPSILON) ? true : false;
        //if(!reached)
        //std::cout << "||--> dist: " << dist << std::endl;
        return reached;
    }
    shared_ptr<Node> last;
    std::vector<shared_ptr<Node>> nodes;
    const double STEP_SIZE;
    const int MAX_ITER;
    const int EPSILON = 1; //TODO (adjust) END_DIST_THRESHOLD
};
