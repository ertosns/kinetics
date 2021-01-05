#define GRAPH_HDR
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <memory>
#include <exception>
#ifndef OBSTACLES
#include "obstacles.hpp"
#endif

class Edge;

//TODO (res) a branch without smartr ptrs.
class Node {
public:
  //heuristic estimate of the cost to goal
  //used in A* search graph
  const double CTG; 
  Point p;
  int id;
  //node name/id
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

  Node(Point _p, double heuristic_ctg=INFINITY,
       int _id=-1, bool _open=true) :
    CTG(heuristic_ctg), p(_p), id(_id), open_(_open),
    cost_(INFINITY) {
    
  }
  Node(Eigen::VectorXd v) : Node(Point(v)) {}

  /** shallow copy of Node
   */
  Node(Node *n) : Node(n->get_point(),
                       n->get_ctg(),
                       n->get_id(),
                       n->open()) {
    auto edges = n->get_edges();
    for(int i = 0; i < edges.size(); i++) {
      add_edge(edges[i]);
    }
  }


  /** shallow copy of Node
   */
  
  Node(Node &n) : Node(n.get_point(),
                      n.get_ctg(),
                      n.get_id(),
                      n.open()) {
    auto edges = n.get_edges();
    for(int i = 0; i < edges.size(); i++) {
      add_edge(edges[i]);
    }
  }

  double get_ctg() {
    return CTG;
  }
  
  Point get_point() {
    return p;
  }

  /** add new edge
   *
   *@param n output Node
   */
  void add_edge(Edge* e) {
    edges.push_back(e);
  }
  //
  void operator+(Edge* e) {
    edges.push_back(e);
  }

  /** calculate the distance between this node, and the given n
   *
   * distance is evaluated = this_point - n_point
   *@param n is pointer to Node
   *@return distance.
   */
  double operator-(Node* n) {
    return get_point() - n->get_point();
  }
  
  double operator-(Node n) {
    return get_point() - n.get_point();
  }

  //TODO why this discards qualifiers?! (dereferencing)
  //bool operator==(std::unique_ptr<Node> n) const
  /*
  bool operator==(std::unique_ptr<Node> n) {
    return p==n->p;
  }
  */

  bool operator==(Node n) {
    return p==n.p;
  }

  bool equal(Node n) {
    return p==n.p;
  }

  bool operator==(Node* n) {
    return p==n->p;
  }

  bool equal(Node* n) {
    return p==n->p;
  }


  void set_id(int idx) {
    id=idx;
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
  
  void set_parent(Node* n) {
    parent=n;
  }

  
  Node* get_parent() {
    return parent;
  }
  

  bool has_parent() {
    return parent!=nullptr;
  }

  int get_parent_id() {
    int id=-1;
    if (has_parent())
      id=parent->id;
    else
      std::cerr << "Node: " << *this << " doesn't have parent!" << std::endl;
    return id;
  }
  
  /** get node state open/close
   *
   *@return open_ true for opened, false for closed
   */
  //TODO (fix) change name to isopen
  bool open() const {
    return open_;
  }

  
  std::vector<Edge*>& get_edges() {
    return edges;
  }
  

  void apply_edge(std::function<void(Edge*)> act) {
    for (auto it=edges.begin(); it!=edges.end(); it++) {
      act(*it);
    }
  }
  
  friend std::ostream& operator<<(std::ostream&, Node&);
  
  //private:
protected:
  std::vector<Edge*> edges;
  Node* parent;
  bool open_;
  double cost_;
};


std::ostream& operator<<(std::ostream& os, Node &n) {
  //TODO (fix) if id, or ctg aren't set (infinity) then don't print them!
  Point p = n.get_point();
  os << "node(" << n.id << ")" << p << " - ctg: " << n.CTG;
  return os;
}

class Edge {
public:
  //TODO (res) is it required to keep track of the current id, and next id (n.id)?
  Edge(Node* n, double weight=INFINITY) : weight_(weight) {
    node=n;
  }
  double weight() {
    return weight_;
  }
  Node* get_node() {
    return node;
  }
  friend std::ostream& operator<<(std::ostream&, Edge&);

private:
  Node* node;
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

  Graph(Node* st,
        Node* ed) : st_(st), ed_(ed){
    std::cout << "graph (" << *st << ") -> (" << *ed << ") instantiated" << std::endl;
  }

  Node* get_start() {
    return st_;
  }

  void set_end(Node *n) {
    ed_=n;
  }
  
  Node* get_end() {
    return ed_;
  }

  //virtual void search()=0;
  
  /** get path starting from ed backward to st, should be called after the searching algorithm.
   * first sort the graph from st_(start) to ed_(end)
   *
   *@return path list<Node> of nodes.
   */

  //TODO (why virtual definition of search result in segfault upon call from the test case, notice that in this case you call seach for the graph's get_path, cost function.
  
  
  std::vector<Node*> get_path() {
    std::cout << "get_path!" << std::endl;
    std::vector<Node*> path;
    //sort the graph from st_(start) to ed_(end)
    //TODO (fix) consider if there is no path.
    //TODO (fix) if the graph isn't searched, then return an error, or better, you should initlize the parents randomly, or set the parent to any edge, and consider if there is no path ot the end.
    //Node* current=path_e;
    Node* current=get_end();
    path.push_back(current);
    //TODO (fix) the path from the ed backward to the st,
    // and it's dereference the parent of each node rather than,
    // the smallest neighbouring (in undirected graph) total cost.
    while (!(*current==*get_start())) {
      std::cout << "PATH: getting parent of : " << *current  << std::endl;
      // TODO get segmentation error from this line, i don't understand why?! and i'm sure it was set with a parent, but this means it doesn't actually, so this current doesn't have parent, but other reference does!!, but how this is possible in case of unique_ptr?!
      Node* parent = current->get_parent();
      path.push_back(parent);
      //TODO (res) a difference?
      //current=parent;
      std::cout << "to be current: " << parent << std::endl;
      current=parent;
      std::cout << "current: " << current << std::endl;
    }
    return path;
  }
  
  //TODO (fix) no longer valid, read cost from corresponding parent
  //TODO change name to get_cost
  double cost() {
    double cost;
    Node* current=get_end();
    while (!(*current==*get_start())) {
      auto current_optimal_edge=(*current->get_edges().begin());
      Node* shortest = current->get_parent();
      cost +=current_optimal_edge->weight();
      current=shortest;
    }
    return cost;
  }

  friend std::ostream& operator<<(std::ostream&, Graph&);
  
private:
  //TODO can the unique_ptr be const?
  Node* st_; /*entry*/
  Node* ed_; /*target*/
 
};

/*
std::ostream& operator<<(std::ostream &os, Graph &g) {
  int i=0;
  
  for (Node* n : g.get_path()) {
    os << " -> " <<  *n << i++;
  }
  
  return os;
}
*/
