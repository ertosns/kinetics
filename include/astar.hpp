#ifndef GRAPH_HDR
#include "graph.hpp"
#endif

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
  AsGraph(std::vector<Node*> opened_) :
    Graph(opened_[0], opened_[opened_.size()-1]) {
    for (int i =0; i < opened_.size(); i++) {
      opened.push_back(opened_[i]);
    }
  }

  void search() {
    Astar(get_start());
  }

private:
  std::vector<Node*> opened;
  Node *path_e;
  std::vector<Node*> path;
  /** Astar search: the minimal cost between node st, ed.
   *
   * @param current current node to search
   */
  /*TODO assigning node to st_, returns error invalid use of non-static data member Graph::st_ */
  void Astar(Node* current) {
    std::cout << "current: " << *current << std::endl;
    if (current==nullptr)
      current=get_start();

    std::cout << "start of astar!" << std::endl;
    update_neighbours(current);
    current->close();
    
    for (Edge *e : current->get_edges()) {
      if (e->get_node()->has_parent())
        std::cout << "||verify parent of: ("<< e->get_node()->id
                  << ") is (" <<  e->get_node()->get_parent_id()
                  << ")" << std::endl;
      else
        std::cout << "||verify parent of: ("<< e->get_node()->id
                  << ") is (" <<  " no parent! " << ")"
                  << std::endl;
    }
    
    for (int i = 0; i < opened.size(); i++)  {
      //for (auto n : opened) 
      if (!opened[i]->open())
        continue;
      //traverse through the general opened nodes instead.
      Astar(opened[i]);
    }
  }
  //
  void update_neighbours(Node *current) {
    //current->apply_edge([current] //
    for (Edge* e : current->get_edges()) {
      if (!e->get_node()->open())
        return;
      double tentative_cost = e->weight() + current->get_cost();
      if (tentative_cost < e->get_node()->get_cost()) {
        e->get_node()->set_cost(tentative_cost);
        //
        //search if e->get_node() exist in path

        e->get_node()->set_parent(current);
        //TODO (fix) why get_end()!=e->get_node() ?!!!
        if (e->get_node()->id==get_end()->id) {
          //path_e=e->get_node();
          set_end(e->get_node());
        }
        std::cout << "|--> parent of (" << e->get_node()->id << ") is (" << e->get_node()->get_parent_id() << ")" << std::endl;
      }
    }
    //);
    std::sort(opened.begin(), opened.end(),
              [](Node*  n_ptr1,
                 Node*  n_ptr2) {
                return n_ptr1->total_cost() <
                  n_ptr2->total_cost();
              });
  }
};
