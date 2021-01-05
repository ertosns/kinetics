#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "gmock/gmock.h"
//#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <vector>
#include <cmath>
#ifndef GRAPH_HDR
#include "../include/graph.hpp"
#endif
#include "../include/astar.hpp"
#include "../include/logger.hpp"
#ifndef OBSTACLES
#include "../include/obstacles.hpp"
#endif
#include "../include/rrt.hpp"

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

TEST(MISSIONING, PointDistance2) {
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

TEST(MISSIONING, NodeEdges) {
  Eigen::VectorXd v1(3);
  v1 << 1,2,3;
  Eigen::VectorXd v2(3);
  v2 << 1,3,3;
  Point p1(v1);
  Point p2(v2);
  
  //  construct a graph (n1) <--0.3--> (n2) ,
//    weighted by 0.3, 
  //  where n1(ctg)=3, n2(ctg)=4.
  
  auto n1 = new Node(p1, 3, 1);
  auto n2 = new Node(p2, 4, 2);
  auto e1 = new Edge(n2, 0.3);
  auto e2 = new Edge(n1, 0.3);
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
  
  // construct a graph (n1) <--0.3--> (n2) ,
  // weighted by 0.3, 
  // where n1(ctg)=3, n2(ctg)=4.
  
  auto n1 = new Node(p1, 3, 1);
  n1->set_cost(0);
  auto n2 = new Node(p2, 4, 2);
  n2->set_cost(INFINITY);
  //
  auto e1 = new Edge(n2, 0.3);
  auto e2 = new Edge(n1, 0.3);
  //
  (*n1)+e1;
  (*n2)+e2;
  std::vector<Node*> nodes;
  nodes.push_back(n1);
  nodes.push_back(n2);
  AsGraph *g = new AsGraph(nodes);
  g->search();
  double cost = g->cost();
  ASSERT_THAT(cost, Eq(0.3));
}



class ReadGraph_CSV {
public:
  std::string edges_path;
  std::string nodes_path;
  std::string obs_path;
  std::vector<Node*> nodes;
  Obstacles obstacles;
  ReadGraph_CSV() : edges_path("/opt/Scene5_example/edges.csv"),
                    nodes_path("/opt/Scene5_example/nodes.csv"),
                    obs_path("/opt/planning_coursera/obstacles.csv"){
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
    //std::cout << "reading nodes" << std::endl;
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
      //std::unique_ptr<Node> node_ptr(node);
      //std::cout << *node << std::endl;
      if (id==1) {
        node->set_cost(0);
      } else {
        node->set_cost(INFINITY);
      }
      nodes.push_back(node);
    }
    f.close();
  }
  
  /** read csv edges file
   *
   */
  void read_edges() {
    //std::cout << "reading edges" << std::endl;
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
      auto edge1_ptr = new Edge(nodes[id1-1], weight);
      auto edge2_ptr = new Edge(nodes[id2-1], weight);
      //std::cout << *edge1_ptr;
      //std::cout << *edge2_ptr;

      //TODO (ref)
      //TODO (nodes[id1-1].get())+edge2_ptr; fails
      //(*nodes[id1-1])+edge2_ptr.get();
      //(*nodes[id2-1])+edge1_ptr.get();
      nodes[id1-1]->add_edge(edge2_ptr);
      nodes[id2-1]->add_edge(edge1_ptr);
      //nodes[id1]->+edge2;
      //nodes[id2]->+edge1;
    }
    f.close();
    return;
  }
  Obstacles read_obstacles() {
    std::ifstream f;
    f.open(obs_path);
    if (!f) {
      std::cerr <<  "filed to read: " << obs_path;
    }
    std::stringstream buff;
    buff << f.rdbuf();
    double x,y,radius;
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
      buff >> x; buff.ignore(); // ','
      buff >> y; buff.ignore(); // ','
      buff >> radius; buff.ignore(); // '\n'
      pt << x, y;
      auto obs = new CircleObs(pt, radius/2.0);
      obstacles.push_back(obs);
    }
    f.close();
    return obstacles;
  }
  AsGraph* construct_graph() {
    read_nodes();
    read_edges();
    //std::cout << "finished reading!" << std::endl;
    //TODO why does using *nodes.begin(), *nodes.end() fails?!
    std::cout << "start node: " << *nodes[0] <<
      "end node: " << *nodes[nodes.size()-1] << std::endl;
    AsGraph *g = new AsGraph(nodes);
    return g;
  }
  RRT* construct_rrt() {
    // read obs
    read_obstacles();
    // read begin/end
    //

    Eigen::VectorXd beg_vec(2);
    beg_vec << -0.5, -0.5;
    Node *begin = new Node(beg_vec, INFINITY, 1);
    
    Eigen::VectorXd end_vec(2);
    end_vec << 0.5, 0.5;
    Node *end = new Node(end_vec);
    
    auto rrt = new RRT(begin, end, obstacles, 1, 1);
    return rrt;
  }
};

TEST(MISSIONING, GraphPath) {
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
  std::vector<Node*> nodes=rrt->get_nodes();
  for (int i = 0; i < nodes.size(); i++) {
    std::vector<double> nodeline;
    
    Node *cur=nodes[i];
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
    for (auto e: nodes[i]->get_edges()) {
      std::vector<double> edgeline;
      edgeline.push_back(cur->get_id());
      edgeline.push_back(e->get_node()->get_id());
      edgeline.push_back(0); // weight
      edgelog.csv_line(edgeline);
    }
  }
  nodelog.close();
  edgelog.close();
  //
  //add path
  //
  for (auto n_ptr : rrt->get_path()) {
    path_vec.push_back(n_ptr->get_id());
  }
  pathlog.csv_line(path_vec);
  pathlog.close();
  std::cout << std::endl;
}


class RRTTest : public RRT {
public:
  RRTTest(Node* begining, Node* target,  Obstacles obs, double map_width=1, double map_height=1) :
    RRT(begining, target, obs, map_width, map_height) {
    
  }
  Node* get_nearest(Node *target) {
    return nearest(target);
    
  }
  double get_distance(Node *st, Node *ed) {
    return distance(st, ed);
  }
  void do_add(Node *parent, Node *sampled) {
    add(parent, sampled);
  }
};

TEST(RRT, nearestNode) {
  auto rg = new ReadGraph_CSV();
  Obstacles obs =rg->read_obstacles();
  // read begin/end
  Eigen::VectorXd beg_vec(2);
  beg_vec << -0.5, -0.5;
  Node *begin = new Node(beg_vec, INFINITY, 1);
  // end
  Eigen::VectorXd end_vec(2);
  end_vec << 0.5, 0.5;
  Node *end = new Node(end_vec);
  //
  RRTTest rrttest(begin, end, obs);
  //
  // p3
  Eigen::VectorXd p3_vec(2);
  p3_vec << 0, 0;
  Node *p3 = new Node(p3_vec);
  // p4
  Eigen::VectorXd p4_vec(2);
  p4_vec << 0, 0.5;
  Node *p4 = new Node(p4_vec);
  // p5
  Eigen::VectorXd p5_vec(2);
  p5_vec << 0.5, 0;
  Node *p5 = new Node(p5_vec);
  // p6
  Eigen::VectorXd p6_vec(2);
  p6_vec << 0.3, 0.3;
  Node *p6 = new Node(p6_vec);
  // p7
  Eigen::VectorXd p7_vec(2);
  p7_vec << -0.3, -0.3;
  Node *p7 = new Node(p7_vec);
  //
  rrttest.do_add(begin, p7); // -0.5 -- -0.3
  rrttest.do_add(p3, p4); // 0 -- 0.5
  rrttest.do_add(p3, p5); // 0 -- 0.5
  rrttest.do_add(p3, p6); // 0 -- 0.3
  rrttest.do_add(p7, p3); // -0.3 -- 0
  //
  // assert distance
  ASSERT_THAT(rrttest.get_distance(begin, end), Eq(std::sqrt(2)));
  ASSERT_THAT(rrttest.get_distance(p3, end), Eq(std::sqrt(2)/2));
  // assert nearest 
  ASSERT_TRUE(rrttest.get_nearest(end)->equal(p6));
}
