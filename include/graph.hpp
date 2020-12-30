#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <memory>
#include <exception>
#include "obstacles.hpp"

class Edge;

//TODO (res) a branch without smartr ptrs.
class Node {
public:
  //heuristic estimate of the cost to goal
  //used in A* search graph
  const double CTG; 
  Point p;
  int id; //node name/id
  /** Graph node class
   *
   * past cost: is the shortest cost so far.
   * optimistic ctg: is constant
   * parent_node: the ordered edges list illuminated the need to keep parent node index, as it's orderd by the "estimated total cost", and it's equivalent to edges.begin() value.
   * @param _p node coordinates
   * @param heuristic_ctg is the estimated cost to go
   * @param _id node name/id
   * @param _open state of the node, it's always opend initially.
   * 
   */
  Node(Point _p, double heuristic_ctg, int _id=-1, bool _open=true) : CTG(heuristic_ctg), p(_p), id(_id) {
    //TODO why i can't do that!!
    //p=_p; i get error there is not Point::Point() !!
    open_=_open;
    cost_=INFINITY;
  }

  /** add new edge
   *
   *@param n output Node
   */
  void operator+(std::shared_ptr<Edge> n) {
    // this keep the parent node
    edges.push_back(n);
    //edges.sort(edges.begin(), edges.end(), lambda edge : edge.node.total_cost());
  }

  //TODO why this discards qualifiers?! (dereferencing)
  //bool operator==(std::shared_ptr<Node> n) const 
  bool operator==(std::shared_ptr<Node> n) {
    return p==n->p;
  }

  bool operator==(Node& n) {
    return p==n.p;
  }
  int get_id() const {
    return id;
  }
  /** get estimated total cost = optimistic ctg + past cost
   *
   */
  double total_cost() const {
    return CTG+get_cost();
  }
  /** get past cost
   *
   *@return cost_ past cost
   */
  //TODO (fix) change name to past_cost
  double get_cost() const {
    return cost_;
  }
  /** set past cost
   *
   */
  //TODO (fix) change name to past_cost
  void set_cost(double c) {
    cost_=c;
  }
  /** set the Node to close state
   *
   */
  //TODO (fix) change name to isclose
  void close() {
    open_=false;
  }
  
  void set_parent(std::shared_ptr<Node> n) {
    //TODO (res) i think i should use reset!
    //parent=n;
    parent.reset(n.get());
  }
  
  std::shared_ptr<Node> get_parent() {
    return parent;
  }
  
  /** get node state open/close
   *
   *@return open_ true for opened, false for closed
   */
  //TODO (fix) change name to isopen
  bool open() const {
    return open_;
  }
  
  std::list<std::shared_ptr<Edge>> get_edges() {
    return edges;
  }
  
  friend std::ostream& operator<<(std::ostream&, Node&);
  
  //private:
protected:
  std::list<std::shared_ptr<Edge>> edges;
  std::shared_ptr<Node> parent;
  bool open_;
  double cost_;
};


std::ostream& operator<<(std::ostream& os, Node& n) {
  os << "node (" << n.id << ")" << " - ctg: " << n.CTG;
  return os;
}

class Edge {
public:
  std::shared_ptr<Node> node;
  //TODO (res) is it required to keep track of the current id, and next id (n.id)?
  Edge(std::shared_ptr<Node> &n, double weight) : weight_(weight) {
    node=n;
  }
  double weight() {
    return weight_;
  }
  std::shared_ptr<Node> get_node() {
    return node;
  }
  friend std::ostream& operator<<(std::ostream&, Edge&);
private:  
  const double weight_;
};

std::ostream& operator<<(std::ostream& os, Edge& e) {
  os << " -> " << *(e.node) << " -< weight: " << e.weight();
  return os;
}

//////////////////////////////////////////////////////////
// construct a general weighted undirected search graph
//////////////////////////////////////////////////////////


/**
 * \brief base class Graph is weighted undirected search graph 
 */

class Graph {
public:

  Graph(std::shared_ptr<Node> st,
        std::shared_ptr<Node> ed) {
    //TODO why does constant initialization st_(st), ed_(ed) fails?
    st_ = st;
    ed_ = ed;
    std::cout << "graph (" << *st << ") -> (" << *ed << ") instantiated" << std::endl;
  }

  std::shared_ptr<Node> get_start() {
    return st_;
  }

  std::shared_ptr<Node> get_end() {
    return ed_;
  }
  /** get path starting from ed backward to st, should be called after the searching algorithm.
   * first sort the graph from st_(start) to ed_(end)
   *
   *@return path list<Node> of nodes.
   */
  std::list<std::shared_ptr<Node>> get_path() {
    std::list<std::shared_ptr<Node>> path;
    //sort the graph from st_(start) to ed_(end)
    //TODO (fix) consider if there is no path.
    //TODO (fix) if the graph isn't searched, then return an error, or better, you should initlize the parents randomly, or set the parent to any edge, and consider if there is no path ot the end.
    std::shared_ptr<Node> current=get_end();
    path.push_back(current);
    //TODO (fix) the path from the ed backward to the st,
    // and it's dereference the parent of each node rather than,
    // the smallest neighbouring (in undirected graph) total cost.
    while (!(*current==*get_start())) {
      std::cout << "PATH: getting parent of : " << *current  << std::endl;
      // TODO get segmentation error from this line, i don't understand why?! and i'm sure it was set with a parent, but this means it doesn't actually, so this current doesn't have parent, but other reference does!!, but how this is possible in case of shared_ptr?!
      std::shared_ptr<Node> parent = current->get_parent();
      path.push_back(parent);
      //TODO (res) a difference?
      //current=parent;
      std::cout << "to be current: " << parent << std::endl;
      current.reset(parent.get());
      std::cout << "current: " << current << std::endl;
    }
    return path;
  }
  //TODO (fix) no longer valid, read cost from corresponding parent
  //TODO change name to get_cost
  //after searching
  double cost() {
    //sort the graph from st_(start) to ed_(end)
    std::cout << "searching the path start " << *st_ << " to node " << *ed_ << std::endl;
    double cost;
    std::shared_ptr<Node> current=st_;
    //path.push_back(current);
    while (!(*current==*ed_)) {
      auto current_optimal_edge=(*current->get_edges().begin());
      std::shared_ptr<Node> shortest = current_optimal_edge->node;
      cost +=current_optimal_edge->weight();
      std::cout << "passes through : " << *shortest << std::endl;
      //path.push_back(shortest);
      current=shortest;
    }
    return cost;
  }
  
  friend std::ostream& operator<<(std::ostream&, Graph&);
  
private:
  //TODO can the shared_ptr be const?
  std::shared_ptr<Node> st_; /*entry*/
  std::shared_ptr<Node> ed_; /*target*/
 
};

std::ostream& operator<<(std::ostream &os, Graph &g) {
  int i=0;
  
  for (std::shared_ptr<Node> n : g.get_path()) {
    os << " -> " <<  *n << i++;
  }
  
  return os;
}
