#define ASTAR
#ifndef GRAPH_HDR
#include "graph.hpp"
#endif
#include <memory>

using namespace std;
/**
 * \brief class AsGraph is a graph for Astar algorithm
 *
 */
class AsGraph : public Graph {
public:
    /** construct a Astar graph
     *
     * start and end will be provided at the cost function
     * st node cost to go = 0, the rest are INFINITY
     * expect that nodes[0] to be st, and nodes.end() to be ed
     * @param st start node
     * @param ed end node
     */
    AsGraph(std::vector<shared_ptr<Node>> opened_) :
        Graph(opened_[0], opened_[opened_.size()-1]) {
        for (auto &&node : opened_) {
            opened.push_back(node);
        }
    }

  void search() {
      Astar(get_start());
  }

private:
    std::vector<shared_ptr<Node>> opened;
    //shared_ptr<Node> path_e;
    //std::vector<shared_ptr<Node>> path;
    /** Astar search: the minimal cost between node st, ed.
     *
     * @param current current node to search
     */
    /*TODO assigning node to st_, returns error invalid use of non-static data member Graph::st_ */
    void Astar(shared_ptr<Node> current) {
        std::cout << "current: " << *current << std::endl;
        if (current==nullptr) {
            current=get_start();
        }
        std::cout << "start of astar!" << std::endl;
        update_neighbours(current);
        current->close();

        for (auto &&e : current->get_edges()) {
            if (e->has_parent())
                std::cout << "||verify parent of: ("<< e->id
                          << ") is (" <<  e->get_parent_id()
                          << ")" << std::endl;
            else
                std::cout << "||verify parent of: ("<< e->id
                          << ") is (" <<  " no parent! " << ")"
                          << std::endl;
        }
        for (auto &&open_node : opened) {
            //for (int i = 0; i < opened.size(); i++)  {
            //for (auto n : opened)
            if (!open_node->open())
                continue;
            //traverse through the general opened nodes instead.
            Astar(open_node);
        }
    }
    //
    void update_neighbours(shared_ptr<Node> current) {
        //current->apply_edge([current] //
        for (auto &&e : current->get_edges()) {
            if (!e->open())
                return;
            double tentative_cost = e->weight() + current->get_cost();
            if (tentative_cost < e->get_cost()) {
                e->set_cost(tentative_cost);
                //
                e->set_parent(current);
                if (e->id==get_end()->id) {
                    //path_e=e;
                    set_end(e);
                }
                std::cout << "|--> parent of (" << e->id << ") is (" << e->get_parent_id() << ")" << std::endl;
            }
        }
        std::sort(opened.begin(), opened.end(),
                  [](shared_ptr<Node>  n_ptr1,
                     shared_ptr<Node>  n_ptr2) {
                      return n_ptr1->total_cost() < n_ptr2->total_cost();
                  });
    }
};
