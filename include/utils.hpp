#pragma once
#define UTILS
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <vector>
#include "gmock/gmock.h"
//#include "gtest/gtest.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <filesystem>
#include <string>
#include <list>
#include <vector>
#include <cmath>
#ifndef GRAPH_HDR
#include "../include/graph.hpp"
#endif
#ifndef ASTAR
#include "../include/astar.hpp"
#endif
#include "../include/logger.hpp"
#ifndef OBSTACLES
#include "../include/obstacles.hpp"
#endif
#ifndef RRT_HDR
#include "../include/rrt.hpp"
#endif
#include <igl/active_set.h>
#include <igl/linprog.h>
#include "algebra/algebra.hpp"
#include <exception>

namespace fs = std::filesystem;

class ReadGraph_CSV {
public:
    std::string edges_path;
    std::string nodes_path;
    std::string obs_path;
    std::vector<shared_ptr<Node>> nodes;
    Obstacles obstacles;
    ReadGraph_CSV() : edges_path(static_cast<string>(fs::current_path())+"/../tests/data/Scene5_example/edges.csv"),
                      nodes_path(static_cast<string>(fs::current_path())+"/../tests/data/Scene5_example/nodes.csv"),
                      obs_path(static_cast<string>(fs::current_path())+"/../tests/data/planning_coursera/obstacles.csv"){
    }

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
            auto node = make_shared<Node>(pt, ctg, id);
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

            auto edge1_ptr = make_shared<Edge>(nodes[id1-1], weight);
            auto edge2_ptr = make_shared<Edge>(nodes[id2-1], weight);

            nodes[id1-1]->add_edge(edge2_ptr);
            nodes[id2-1]->add_edge(edge1_ptr);
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
        auto begin = make_shared<Node>(beg_vec, INFINITY, 1);

        Eigen::VectorXd end_vec(2);
        end_vec << 0.5, 0.5;
        auto end = make_shared<Node>(end_vec);

        auto rrt = new RRT(begin, end, obstacles, 1, 1);
        return rrt;
    }
};
