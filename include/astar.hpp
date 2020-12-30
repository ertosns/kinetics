#include "graph.hpp"

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
  AsGraph(std::shared_ptr<Node> st, std::shared_ptr<Node> ed,
          std::vector<std::shared_ptr<Node>> opened_) :
    Graph(st, ed), opened(opened_) {
  }
  
  void Astar() {
    Astar(get_start());
  }
  
private:
  std::vector<std::shared_ptr<Node>> opened;

  //TODO (res) calling this function as a wrapper instead of opened directly result in segmentation fault
  /*
  std::vector<std::shared_ptr<Node>> get_opened() {
    return opened;
  }
  */
  
  /** Astar search: the minimal cost between node st, ed.
   *
   * @param st starting node
   * @param ed ending node
   */
  /*TODO assigning node to st_, returns error invalid use of non-static data member Graph::st_ */
  //TODO it's depth first search, implement breadth first seach.
  void Astar(std::shared_ptr<Node> current) {
    if (current==nullptr)
      current=get_start();
    if ((*current)==get_end()) {
      return;
      //TODO how do you extract the path in this case.
    }
    update_neighbours(current);
    current->close();
    //TODO (fix) remove it from the opened list
    //opened.erase(get_current_iterator_pointer);
    
    //you can either close it, or remove is from current_edges.
    //TODO ALL THIS TROUBLE IS FROM THE DEPTH FIRST SEARCH, IT SUPPOSED TO BE BREADTH FIRST SEARCH.
    /*
      for (std::shared_ptr<Edge> e : edges) {
      if (!e->get_node()->open()) {
      continue;
      }
      std::cout << std::endl << "A* traversing to node: " <<
      * e->get_node() << std::endl << std::endl;
      Astar(e->get_node());
      }*/
    
    for (std::shared_ptr<Node> n : opened) {
      if (!n->open())
        continue;
      //traverse through the general opened nodes instead.
      Astar(n);
    }
  }
  
  void update_neighbours(std::shared_ptr<Node> &current) {
    for (std::shared_ptr<Edge> e : current->get_edges()) {
      if (!e->get_node()->open())
        continue;
      double tentative_cost = e->weight() + current->get_cost();
      if (tentative_cost < e->node->get_cost()) {
        //std::cout << " updating the past cost of edge: " << *e << " with past-cost: " << tentative_cost << std::endl;
        e->get_node()->set_cost(tentative_cost);
        e->get_node()->set_parent(current);
        
        std::cout << "|--> parent of (" << e->get_node()->id << ") is (" << e->get_node()->get_parent()->id << ")" << std::endl;
        /*
        //this is depth first search
        current->get_edges().
        sort([] (std::shared_ptr<Edge>  e_ptr1,
        std::shared_ptr<Edge>  e_ptr2) {
        // e_ptr1 < e_ptr2
        return e_ptr1->get_node()->total_cost() <
        e_ptr2->get_node()->total_cost();
        });
        */
        std::sort(opened.begin(), opened.end(),
                  [](std::shared_ptr<Node>  n_ptr1,
                     std::shared_ptr<Node>  n_ptr2) {
                    // e_ptr1 < e_ptr2
                    return n_ptr1->total_cost() <
                      n_ptr2->total_cost();
                  });
      }
    }
  }
};
