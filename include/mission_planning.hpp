#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <memory>

class Point {  
public:
  Point(Eigen::VectorXd p): p_(p){
  }
  
  /** calculate the distance from the current point to point p.
   *@param p given point
   *@return distance
   */
  double operator-(Point &p) {
    return sqrt((p.p_.array() - p_.array()).square().sum());
  }

  double operator-(Point *p) {
    return sqrt((p->p_.array() - p_.array()).square().sum());
  }
  /** verify that p is the same as the current point
   *@param p given point
   *@return boolean check
   */
  bool operator==(Point &p) {
    return *this-p==0; //return p-this==0
  }
  friend std::ostream& operator<<(std::ostream&, const Point&);
private:
  const Eigen::VectorXd p_;
};

class Edge;

//TODO use shared_ptr for in/out Nodes
/*
 * past cost: is the shortest cost so far.
 * optimistic ctg: is constant
 * parent_node: the ordered edges list illuminated the need to keep parent node index, as it's orderd by the "estimated total cost", and it's equivalent to edges.begin() value.
 * 
 */
class Node {
public:
  const double CTG; //heuristic estimate of the cost to goal
  Point p;
  int id; //node name/id
  Node(Point _p, double heuristic_ctg, int _id=-1, bool _open=true) : CTG(heuristic_ctg), p(_p), id(_id) {
    //TODO why i can't do that
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

  //TODO why this discards qualifiers?!
  //bool operator==(std::shared_ptr<Node> n) const 
  bool operator==(std::shared_ptr<Node> n) {
    return p==n->p;
  }

  bool operator==(Node& n) {
    return p==n.p;
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
  double get_cost() const {
    return cost_;
  }
  /** set past cost
   *
   */
  void set_cost(double c) {
    cost_=c;
  }
  /** set the Node to close state
   *
   */
  void close() {
    open_=false;
  }

  void set_parent(std::shared_ptr<Node> p) {
    parent=p;
  }
  std::shared_ptr<Node> get_parent() {
    return parent;
  }
  /** get node state open/close
   *
   *@return open_ true for opened, false for closed
   */
  bool open() const {
    return open_;
  }
  std::list<std::shared_ptr<Edge>> get_edges() {
    return edges;
  }

  friend std::ostream& operator<<(std::ostream&, Node&);
  
private:
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
  os << " -> " << *(e.node) << " -< weight: " << e.weight() << std::endl;
  return os;
}

class Graph {
public:
  
  /** construct a graph
   *
   * start and end will be provided at the cost function
   * st node cost to go = 0, the rest are INFINITY
   * expect that nodes[0] to be st, and nodes.end() to be ed
   *@param st start node
   *@param ed end node
   */
  Graph(std::shared_ptr<Node> st,
        std::shared_ptr<Node> ed) {
    //TODO why does constant initialization st_(st), ed_(ed) fails?
    st_ = st;
    ed_ = ed;
    std::cout << "graph (" << *st << ") -> (" << *ed << ") instantiated" << std::endl;
  }
  
  /** Astar search: the minimal cost between node st, ed.
   *
   * @param st starting node
   * @param ed ending node
   */
  /*TODO assigning node to st_, returns error invalid use of non-static data member Graph::st_ */
  //TODO it's depth first search, implement breadth first seach
  void Astar(std::shared_ptr<Node> current) {
    std::cout << "A* search with current node: " << *current << std::endl;
    // if no node provided, start from the st node.
    if (current==nullptr)
      current=st_;
    if ((*current)==ed_) {
      return;
      //TODO how do you extract the path in this case.
    }
    std::list<std::shared_ptr<Edge>> edges=current->get_edges();
    update_neighbours(current);
    //you can either close it, or remove is from current_edges
    current->close();
    for (std::shared_ptr<Edge> e : edges) {
      if (!current->open())
        continue;
      Astar(e->node);
    }
    std::cout << "A* finished ..." << std::endl;
  }

  /** get shortest A* path
   * first sort the graph from st_(start) to ed_(end)
   *
   *@return path list<Node> of nodes.
   */
  std::list<std::shared_ptr<Node>> get_path() {
    //sort the graph from st_(start) to ed_(end)
    std::cout << "searching the path start " << *st_ << " to node " << *ed_ << std::endl;
    Astar(st_);
    std::shared_ptr<Node> current=st_;
    path.push_back(current);
    //TODO (fix) the path from the ed backward to the st,
    // and it's dereference the parent of each node rather than,
    // the smallest neighbouring (in undirected graph) total cost.
    while (!(*current==*ed_)) {
      std::shared_ptr<Node> shortest = (*current->get_edges().begin())->node;
      std::cout << "passes through : " << *shortest << std::endl;
      path.push_back(shortest);
      current=shortest;
    }
    return path;
  }

  double cost() {
    //sort the graph from st_(start) to ed_(end)
    std::cout << "searching the path start " << *st_ << " to node " << *ed_ << std::endl;
    Astar(st_);
    double cost;
    std::shared_ptr<Node> current=st_;
    path.push_back(current);
    while (!(*current==*ed_)) {
      auto current_optimal_edge=(*current->get_edges().begin());
      std::shared_ptr<Node> shortest = current_optimal_edge->node;
      cost +=current_optimal_edge->weight();
      std::cout << "passes through : " << *shortest << std::endl;
      path.push_back(shortest);
      current=shortest;
    }
    return cost;
  }
  
  friend std::ostream& operator<<(std::ostream&, Graph&);
  
private:
  std::list<std::shared_ptr<Node>> path;
  //TODO can the shared_ptr be const?
  std::shared_ptr<Node> st_; /*entry*/
  std::shared_ptr<Node> ed_; /*target*/
  //
  void update_neighbours(std::shared_ptr<Node> &current) {
    for (std::shared_ptr<Edge> e : current->get_edges()) {
      /*
      if (!e->get_node()->open())
        continue;
      */
      double tentative_cost = e->weight() + current->get_cost();
      if (tentative_cost < e->node->get_cost()) {
        std::cout << " updating the past cost of edge: " << *e <<
          " with past-cost: " << tentative_cost << std::endl;
        e->get_node()->set_cost(tentative_cost);
        //make sure that the sort exclude the closed nodes.
        //TODO perhabs this should be outside the loop
        //set parent
        e->get_node()->set_parent(current);
        
        //TODO(fix) this sort is now redundant, or need to be fixed to replace the extra parent variable
        current->get_edges().
          sort([] (std::shared_ptr<Edge>  e_ptr1,
                   std::shared_ptr<Edge>  e_ptr2) {
                 // e_ptr1 < e_ptr2
                 return e_ptr1->get_node()->total_cost() <
                   e_ptr2->get_node()->total_cost();
               });
      }
      std::cout << "|--> parent of (" << e->get_node()->id << ") is (" << current->id << ")" << std::endl;
    }
  }
};

std::ostream& operator<<(std::ostream &os, Graph &g) {
  int i=0;
  
  for (std::shared_ptr<Node> n : g.get_path()) {
    os << " -> " <<  *n << i++;
  }
  
  return os;
}
