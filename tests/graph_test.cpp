#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <vector>
#include "gmock/gmock.h"
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <vector>
#include <cmath>
#include "../include/logger.hpp"
#ifndef OBSTACLES
#include "../include/obstacles.hpp"
#endif
#ifndef UTILS
#include "../include/utils.hpp"
#endif
#include <exception>

using namespace testing;

TEST(GRAPH, PointZero) {
  Eigen::VectorXd v1(2);
  v1 << 0,0;
  Eigen::VectorXd v2(2);
  v2 << 0,0;
  Point p1(v1);
  Point p2(v2);
  ASSERT_THAT(p1-p2, Eq(0));
  ASSERT_TRUE(p1==p2);
}

TEST(GRAPH, PointDistance) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);
  ASSERT_THAT(p1-p2, Eq(1));
  ASSERT_FALSE(p1==p2);
}

TEST(GRAPH, PointDistance2) {
  //try negative coordinates
  Eigen::VectorXd v1(2);
  v1 << -1,-1;
  Eigen::VectorXd v2(2);
  v2 << 1,1;
  Point p1(v1);
  Point p2(v2);
  ASSERT_THAT(p1-p2, Eq(2*std::sqrt(2)));
  ASSERT_FALSE(p1==p2);
}


TEST(GRAPH, NodeEdges) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);

  //  construct a graph (n1) <--0.3--> (n2) ,
//    weighted by 0.3,
  //  where n1(ctg)=3, n2(ctg)=4.

  auto n1 = make_shared<Node>(p1, 3, 1);
  auto n2 = make_shared<Node>(p2, 4, 2);
  //auto e1 = make_shared<Edge>(n2, 0.3);
  //auto e2 = make_shared<Edge>(n1, 0.3);
  (*n1)+n2;
  (*n2)+n1;
  //verify the node's edges sizes
  ASSERT_THAT(n1->get_edges().size(), Eq(1));
  ASSERT_THAT(n2->get_edges().size(), Eq(1));
  //  test node traversing
  ASSERT_THAT((*n1->get_edges().begin())->id, Eq(2));
  ASSERT_THAT((*n2->get_edges().begin())->id, Eq(1));
  // test Node equality operator
  ASSERT_TRUE(*((*n1->get_edges().begin()))==*n2);
  ASSERT_TRUE(*((*n2->get_edges().begin()))==*n1);
  //
}

/*
TEST(GRAPH, GraphCost) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);

  // construct a graph (n1) <--0.3--> (n2) ,
  // weighted by 0.3,
  // where n1(ctg)=3, n2(ctg)=4.

  auto n1 = make_shared<Node>(p1, 3, 1);
  n1->set_cost(0);
  auto n2 = make_shared<Node>(p2, 4, 2);
  n2->set_cost(INFINITY);
  //
  //auto e1 = make_shared<Node>(n2, 0.3);
  //auto e2 = make_shared<Node>(n1, 0.3);
  //
  (*n1)+n2;
  (*n2)+n1;
  std::vector<shared_ptr<Node>> nodes;
  nodes.push_back(n1);
  nodes.push_back(n2);
  AsGraph *g = new AsGraph(nodes);
  g->search();
  double cost = g->cost();
  ASSERT_THAT(cost, Eq(0.3));
}
*/
TEST(GRAPH, GraphPath) {
    //TODO (fix) move readgraph readnodes, readedges to here!
    ReadGraph_CSV rg;
    AsGraph *g = rg.construct_graph();
    //
    std::vector<int> truth_table= {1,3,4,7,10,12};
    //secondly get the path
    g->search();
    std::cout << "||*_*|| graph searched! ||*_*||" << std::endl;
    auto path = g->get_path();
    Logger pathlog = Logger("path.csv");
    int c=0;
    std::vector<int> pathline;
    for (int i = 0; i < path.size(); i++) {
        pathline.push_back(path[i]->get_id());
    }
    pathlog.csv_line(pathline);
    pathlog.close();
}
