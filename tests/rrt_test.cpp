#include <iostream>
#include <Eigen/Dense>
#include "gmock/gmock.h"
#include <list>
#include <vector>
#include <cmath>
#include "../include/logger.hpp"

#ifndef OBSTACLES
#include "../include/obstacles.hpp"
#endif

#ifndef RRT_HDR
#include "../include/rrt.hpp"
#endif

#ifndef UTILS
#include "../include/utils.hpp"
#endif


using namespace testing;

TEST(RRT, ObstacleIntersect) {
    Eigen::VectorXd center_vec(2);
    center_vec << 2,2;
    Point center(center_vec);
    double radius=1.5;
    auto C = CircleObs(center, radius);
    //
    Eigen::VectorXd p1_vec(2);
    p1_vec << 0,3;
    Point p1(p1_vec);
    //
    Eigen::VectorXd p2_vec(2);
    p2_vec << 3,3;
    Point p2(p2_vec);
    //
    //Tangent line is considered to be intersecting.
    ASSERT_TRUE(C.intersect(p1, p2));
}


class RRTTest : public RRT {
public:
    RRTTest(shared_ptr<Node> begining, shared_ptr<Node> target,
            Obstacles obs, double map_width=1, double map_height=1) :
        RRT(begining, target, obs, map_width, map_height) {
    }
    shared_ptr<Node> get_nearest(shared_ptr<Node> target) {
        return nearest(target);
    }
    double get_distance(shared_ptr<Node> st, shared_ptr<Node> ed) {
        return distance(st, ed);
    }
    void do_add(shared_ptr<Node> parent, shared_ptr<Node> sampled) {
        add(parent, sampled);
    }
};

TEST(RRT, nearestNode) {
  auto rg = new ReadGraph_CSV();
  Obstacles obs =rg->read_obstacles();
  // read begin/end
  Eigen::VectorXd beg_vec(2);
  beg_vec << -0.5, -0.5;
  auto begin = make_shared<Node>(beg_vec, INFINITY, 1);
  // end
  Eigen::VectorXd end_vec(2);
  end_vec << 0.5, 0.5;
  auto end = make_shared<Node>(end_vec);
  //
  RRTTest rrttest(begin, end, obs);
  //
  // p3
  Eigen::VectorXd p3_vec(2);
  p3_vec << 0, 0;
  auto p3 = make_shared<Node>(p3_vec);
  // p4
  Eigen::VectorXd p4_vec(2);
  p4_vec << 0, 0.5;
  auto p4 = make_shared<Node>(p4_vec);
  // p5
  Eigen::VectorXd p5_vec(2);
  p5_vec << 0.5, 0;
  auto p5 = make_shared<Node>(p5_vec);
  // p6
  Eigen::VectorXd p6_vec(2);
  p6_vec << 0.3, 0.3;
  auto p6 = make_shared<Node>(p6_vec);
  // p7
  Eigen::VectorXd p7_vec(2);
  p7_vec << -0.3, -0.3;
  auto p7 = make_shared<Node>(p7_vec);
  //
  rrttest.do_add(begin, p7); // -0.5 -- -0.3
  rrttest.do_add(p7, p3); // -0.3 -- 0
  rrttest.do_add(p3, p4); // 0 -- 0.5
  rrttest.do_add(p3, p5); // 0 -- 0.5
  rrttest.do_add(p3, p6); // 0 -- 0.3
  //
  // assert distance
  ASSERT_THAT(rrttest.get_distance(begin, end), Eq(std::sqrt(2)));
  ASSERT_THAT(rrttest.get_distance(p3, end), Eq(std::sqrt(2)/2));
  // assert nearest
  ASSERT_TRUE(*(rrttest.get_nearest(end))==*p6);
}

TEST(RRT, RRTSearch) {
  auto pathlog = Logger("path.csv");
  auto nodelog = Logger("nodes.csv");
  auto edgelog = Logger("edges.csv");
  std::vector<double> path_vec;
  auto rg = new ReadGraph_CSV();
  RRT *rrt = rg->construct_rrt();
  rrt->search();
  //
  //add nodes
  //
  std::vector<shared_ptr<Node>> nodes=rrt->get_nodes();
  for (auto &&node : nodes) {
    std::vector<double> nodeline;
    auto cur=node;
    Eigen::VectorXd p_vec=cur->get_point().vector();
    //TODO update ctg to the Euclidean distance to goal
    nodeline.push_back(cur->get_id());
    nodeline.push_back(p_vec(0));
    nodeline.push_back(p_vec(1));
    nodeline.push_back(0); //heuristic cost to go
    nodelog.csv_line(nodeline);
    //
    //add edges
    //
    for (auto &&e: node->get_edges()) {
      std::vector<double> edgeline;
      edgeline.push_back(cur->get_id());
      edgeline.push_back(e->get_id());
      edgeline.push_back(0); // weight
      edgelog.csv_line(edgeline);
    }
  }
  nodelog.close();
  edgelog.close();
  //
  //add path
  //
  for (auto &&n_ptr : rrt->get_path()) {
      path_vec.push_back(n_ptr->get_id());
  }
  pathlog.csv_line(path_vec);
  pathlog.close();
  std::cout << std::endl;
}


TEST(RRT, ConRRTSearch) {
  auto nodelog = Logger("nodes.csv");
  auto edgelog = Logger("edges.csv");
  auto rg = new ReadGraph_CSV();
  ConRRT *rrt = rg->construct_conrrt();
  rrt->run();

  //
  //add nodes
  //

  std::vector<shared_ptr<Node>> nodes=rrt->get_nodes();
  for (int i = 0; i < nodes.size(); i++) {
    std::vector<double> nodeline;

    shared_ptr<Node> cur=nodes[i];
    Eigen::VectorXd p_vec=cur->get_point().vector();
    //TODO update ctg to the Euclidean distance to goal
    nodeline.push_back(cur->get_id());
    nodeline.push_back(p_vec(0));
    nodeline.push_back(p_vec(1));
    nodeline.push_back(0); //heuristic cost to go
    nodelog.csv_line(nodeline);
    //
    //add edges
    //
    for (auto &&e: nodes[i]->get_edges()) {
      std::vector<double> edgeline;
      edgeline.push_back(cur->get_id());
      edgeline.push_back(e->get_id());
      edgeline.push_back(0); // weight
      edgelog.csv_line(edgeline);
    }
  }
  nodelog.close();
  edgelog.close();
  //
  //add path
  //
  std::vector<double> path_vec;
  vector<shared_ptr<Node>> beginings = rrt->get_start();
  int c = 0;
  for (auto &&begin : beginings) {
      auto pathlog = Logger("path"+to_string(c)+".csv");
      for (auto &&n_ptr : rrt->get_path(begin)) {
          path_vec.push_back(n_ptr->get_id());
      }
      pathlog.csv_line(path_vec);
      pathlog.close();
      c++;
  }
  std::cout << std::endl;
}
