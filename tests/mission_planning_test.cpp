#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gmock/gmock.h"
//#include "gtest/gtest.h"
#include "../include/mission_planning.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <vector>
#include <cmath>

using namespace testing;


TEST(MISSIONING, PointZero) {
  Eigen::VectorXd v1(2);
  v1 << 0,0;
  Eigen::VectorXd v2(2);
  v2 << 0,0;
  Point p1(v1);
  Point p2(v2);
  ASSERT_THAT(p1-p2, Eq(0));
  ASSERT_TRUE(p1==p2);
}

TEST(MISSIONING, PointDistance) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);
  ASSERT_THAT(p1-p2, Eq(1));
  ASSERT_FALSE(p1==p2);
}

TEST(MISSIONING, NodeEdges) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);
  /*
    construct a graph (n1) <--0.3--> (n2) ,
    weighted by 0.3, 
    where n1(ctg)=3, n2(ctg)=4.
  */
  auto n1 = std::shared_ptr<Node>(new Node(p1, 3, 1));
  auto n2 = std::shared_ptr<Node>(new Node(p2, 4, 2));
  auto e1 = std::shared_ptr<Edge>(new Edge(n2, 0.3));
  auto e2 = std::shared_ptr<Edge>(new Edge(n1, 0.3));
  (*n1)+e1;
  (*n2)+e2;
  //verify the node's edges sizes
  ASSERT_THAT(n1->get_edges().size(), Eq(1));
  ASSERT_THAT(n2->get_edges().size(), Eq(1));
  //  test node traversing
  ASSERT_THAT((*n1->get_edges().begin())->get_node()->id, Eq(2));
  ASSERT_THAT((*n2->get_edges().begin())->get_node()->id, Eq(1));
  // test Node equality operator
  ASSERT_TRUE(*(*n1->get_edges().begin())->get_node()==n2);
  ASSERT_TRUE(*(*n2->get_edges().begin())->get_node()==n1);
  //
}

TEST(MISSIONING, GraphCost) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);
  /*
    construct a graph (n1) <--0.3--> (n2) ,
    weighted by 0.3, 
    where n1(ctg)=3, n2(ctg)=4.
  */
  auto n1 = std::shared_ptr<Node>(new Node(p1, 3, 1));
  n1->set_cost(0);
  auto n2 = std::shared_ptr<Node>(new Node(p2, 4, 2));
  n2->set_cost(INFINITY);
  //
  auto e1 = std::shared_ptr<Edge>(new Edge(n2, 0.3));
  auto e2 = std::shared_ptr<Edge>(new Edge(n1, 0.3));
  //
  (*n1)+e1;
  (*n2)+e2;
  Graph g = Graph(n1, n2);
  // test the graph
  double cost = g.cost();
  ASSERT_THAT(cost, Eq(0.3));
}


class ReadGraph_CSV {
public:
  std::string edges_path;
  std::string nodes_path;
  std::vector<std::shared_ptr<Node>> nodes;
  ReadGraph_CSV() : edges_path("/tmp/edges.csv"),
                    nodes_path("/tmp/nodes.csv") {
    /*
    std::cout << "enter edges_path: ";
    std::cin >> edges_path;
    //TODO verify that the path is valid
    std::cout << "enter nodes_path: ";
    std::cin >> nodes_path;
    //TODO verify that the path is valid
    */
  }

  //TODO use regex
  /** read csv node file
   *
   * each node lines consists of
   * ID,x,y,ctg
   */
  void read_nodes() {
    std::cout << "reading nodes" << std::endl;
    std::ifstream f;
    f.open(nodes_path);
    if (!f) {
      std::cerr <<  "filed to read: " << nodes_path;
      return;
    }
    std::stringstream buff;
    buff << f.rdbuf();
    int id;
    double x,y,ctg;
    Eigen::VectorXd pt(2);
    int c;
    char s[1024];
    while (buff) {
      if ((c=buff.get())=='#') {
        buff.getline(s, 1024);
        continue;
      } else {
        buff.putback(c);
      }
      buff >> id; buff.ignore(); // ','
      buff >> x; buff.ignore(); // ','
      buff >> y; buff.ignore(); // ','
      buff >> ctg; buff.ignore(); // '\n'
      pt << x, y;
      Node *node = new Node(pt, ctg, id);
      std::shared_ptr<Node> node_ptr(node);
      std::cout << *node << std::endl;
      if (id==1) {
        node_ptr->set_cost(0);
      } else {
        node_ptr->set_cost(INFINITY);
      }
      nodes.push_back(node_ptr);
    }
    f.close();
  }
  
  /** read csv edges file
   *
   */
  void read_edges() {
    std::cout << "reading edges" << std::endl;
    std::ifstream f;
    f.open(edges_path);
    if (!f) {
      std::cerr << "failed to read: " << edges_path;
      return;
    }
    std::stringstream buff;
    buff << f.rdbuf();
    int id1, id2;
    double weight;
    int c;
    char s[1024];
    while(buff) {
      if ((c=buff.get())=='#') {
        buff.getline(s, 1024);
        continue;
      } else {
        buff.putback(c);
      }
      //auto line = buff.readline();
      buff >> id1; buff.ignore(); // ','
      buff >> id2; buff.ignore(); // ','
      buff >> weight; buff.ignore(); // '\n'
      //add edge to node idx
      //std::cout << "id1: " << id1 << ", id2: " << id2 << std::endl;
      std::shared_ptr<Edge> edge1_ptr(new Edge(nodes[id1-1], weight));
      std::shared_ptr<Edge> edge2_ptr(new Edge(nodes[id2-1], weight));
      std::cout << *edge1_ptr;
      std::cout << *edge2_ptr;

      //TODO (ref)
      //TODO (nodes[id1-1].get())+edge2_ptr; fails
      (*nodes[id1-1].get())+edge2_ptr;
      (*nodes[id2-1].get())+edge1_ptr;
      //nodes[id1]->+edge2;
      //nodes[id2]->+edge1;
    }
    f.close();
    return;
  }
  std::unique_ptr<Graph> construct_graph() {
    read_nodes();
    read_edges();
    std::cout << "finished reading!" << std::endl;
    //TODO why does using *nodes.begin(), *nodes.end() fails?!
    auto g=std::unique_ptr<Graph>(new Graph(nodes[0], nodes[nodes.size()-1]));
    return g;
  }
};

TEST(MISSIONING, Graph) {
  //TODO (fix) move readgraph readnodes, readedges to here!
  ReadGraph_CSV rg;
  std::unique_ptr<Graph> g = rg.construct_graph();
  
}
