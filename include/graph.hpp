#define GRAPH_HDR
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <list>
#include <vector>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <exception>
#ifndef OBSTACLES
#include "obstacles.hpp"
#endif

using namespace std;

class SingleParent : public exception {
    string what () {
        std::cout << "can't have more than one parent, already has a parent" << std::endl;
        return "Node can has only a single parent, parent is already set!";
    }
};

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
        cost_(INFINITY),
        weight_(INFINITY) {
    }
    Node(Point _p, double weight, double heuristic_ctg=INFINITY,
         int _id=-1, bool _open=true) :
        CTG(heuristic_ctg), p(_p), id(_id), open_(_open),
        cost_(INFINITY),
        weight_(weight) {
    }
    Node(Eigen::VectorXd v) : Node(Point(v)) {}
    Node (Node *n) = delete;
    Node (Node &n) = delete;
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
    void add_edge(shared_ptr<Node> e) {
        edges.push_back(e);
    }

    void operator+(shared_ptr<Node> e) {
        edges.push_back(e);
    }

    /** calculate the distance between this node, and the given n
     *
     * distance is evaluated = this_point - n_point
     *@param n is pointer to Node
     *@return distance.
     */
    double operator-(shared_ptr<Node> n) {
        return get_point() - n->get_point();
    }

    double operator-(Node &n) {
        return get_point() - n.get_point();
    }

    bool operator==(Node &n) {
        return p==n.p;
    }

    bool operator==(shared_ptr<Node> n) {
        return p==n->p;
    }

    void set_id(int idx) {
        id=idx;
    }

    int get_id() const {
        return id;
    }
    /**  total_cost
     *
     * get estimated total cost = optimistic ctg + past cost
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

    void set_parent(shared_ptr<Node> n) {
        if (has_parent())
            throw SingleParent();
        parent=n;
    }

    shared_ptr<Node> get_parent() {
        return parent;
    }

    bool has_parent() {
        return parent!=nullptr;
    }

    int get_parent_id() {
        int id=-1;
        if (has_parent())
            id=parent->id;
        return id;
    }

    /** get node state open/close
     *
     *@return open_ true for opened, false for closed
     */
    bool open() const {
        return open_;
    }

    std::vector<shared_ptr<Node>> get_edges() {
        return edges;
    }

    shared_ptr<Node> get_first_edge() {
        return edges[0];
    }

    friend std::ostream& operator<<(std::ostream&, Node&);

    double weight() {
        return weight_;
    }
    void set_weight(double w) {
        weight_=w;
    }
protected:
    std::vector<shared_ptr<Node>> edges;
    std::shared_ptr<Node> parent;
    bool open_;
    double cost_;
    double weight_;
};

//////////////////////////////////////////////////////////
// construct a general weighted undirected search graph
//////////////////////////////////////////////////////////


/**
 * \brief base class Graph is weighted undirected search graph
 */

class Graph {
public:

    Graph(shared_ptr<Node> st, shared_ptr<Node> ed) : st_(st), ed_(ed) {
        std::cout << "graph (" << *st << ") -> (" << *ed << ") instantiated" << std::endl;
    }

    shared_ptr<Node> get_start() {
        return st_;
    }

    void set_end(shared_ptr<Node> n) {
        ed_=n;
    }

    shared_ptr<Node> get_end() {
        return ed_;
    }

    virtual void search() {};

    /** get path starting from ed backward to st, should be called after the searching algorithm.
     * first sort the graph from st_(start) to ed_(end)
     *
     *@return path list<Node> of nodes.
     */
    std::vector<shared_ptr<Node>> get_path() {
        std::vector<shared_ptr<Node>> path;
        //sort the graph from st_(start) to ed_(end)
        shared_ptr<Node> start = get_start();
        shared_ptr<Node> current= get_end();
        path.push_back(current);
        while (!(*current==*start)) {
            current = current->get_parent();
            path.push_back(current);
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    double cost() {
        double cost;
        auto current = get_end();
        while (!(*current==*get_start())) {
            shared_ptr<Node> edg = current->get_first_edge();
            shared_ptr<Node> current_optimal_edge=edg;
            shared_ptr<Node> shortest = current->get_parent();
            cost +=current_optimal_edge->weight();
            current=shortest;
        }
        return cost;
    }
    std::vector<shared_ptr<Node>> get_nodes() {
        return nodes;
    }
    void add_node(shared_ptr<Node> node) {
        nodes.push_back(node);
    }
    double distance(shared_ptr<Node> &p, shared_ptr<Node> &q) {
        auto p_pt = p->get_point().vector();
        auto q_pt = q->get_point().vector();
        //
        double p_x = p_pt(0);
        double p_y = p_pt(1);
        //
        double q_x = q_pt(0);
        double q_y = q_pt(1);
        double dist  = sqrt(pow(p_x-q_x,2) + pow(p_y-q_y,2));
        //return std::abs(*p-*q);
        return dist;
    }
    //note not thread safe
    shared_ptr<Node> get_last() {
        return nodes[nodes.size()-1];
    }
  friend std::ostream& operator<<(std::ostream&, Graph&);

private:
    shared_ptr<Node> st_; /*entry*/
    shared_ptr<Node> ed_; /*target*/
    std::vector<shared_ptr<Node>> nodes; //TODO illuminate last, replace vector with list, last is last index
};

/* ConGraph
 *
 * concurrent graph with multiply starting points, and single target.
 */
class ConGraph {
    std::mutex mu;
    std::condition_variable cond;
    vector<shared_ptr<Node>> st_; /*entry*/
    shared_ptr<Node> ed_; /*target*/
    std::vector<shared_ptr<Node>> nodes;
public:

    ConGraph(std::vector<shared_ptr<Node>> st, shared_ptr<Node> ed) : st_(st), ed_(ed) {
    }

    std::vector<shared_ptr<Node>> get_start() {
        std::unique_lock<std::mutex> locker(mu);
        return st_;
    }

    shared_ptr<Node> get_end() {
        return ed_;
    }

    virtual void search(shared_ptr<Node>) {};

    /** get path starting from ed backward to st, should be called after the searching algorithm.
     * first sort the graph from st_(start) to ed_(end)
     *
     *@return path list<Node> of nodes.
     */
    std::vector<shared_ptr<Node>> get_path(shared_ptr<Node> start) {
        std::unique_lock<std::mutex> locker(mu);
        std::cout << "get_path!" << std::endl;
        std::vector<shared_ptr<Node>> path;
        //sort the graph from st_(start) to ed_(end)
        auto current= get_end();
        path.push_back(current);
        std::cout << "getting path between start: "
                  << *start << ", and target: "
                  << *current << std::endl;
        while (!(*current==*start)) {
            shared_ptr<Node> parent =current->get_parent();
            std::cout << "current: " << *current << ", parent: " << *parent << std::endl;
            path.push_back(parent);
            current=parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    double cost(shared_ptr<Node> start) {
        std::unique_lock<std::mutex> locker(mu);
        double cost;
        auto current = get_end();
        while (!(*current==*start)) {
            shared_ptr<Node> edg = current->get_first_edge();
            shared_ptr<Node> current_optimal_edge=edg;
            shared_ptr<Node> shortest = current->get_parent();
            cost +=current_optimal_edge->weight();
            current=shortest;
        }
        return cost;
    }
    std::vector<shared_ptr<Node>> get_nodes() {
        std::unique_lock<std::mutex> locker(mu);
        return nodes;
    }
    void add_node(shared_ptr<Node> node) {
        std::lock_guard<std::mutex> locker(mu);
        nodes.push_back(node);
    }
    shared_ptr<Node> nearest(shared_ptr<Node> target) {
        double min=1e9;
        shared_ptr<Node> closest(nullptr);
        shared_ptr<Node> cur(nullptr);
        double dist;
        vector<shared_ptr<Node>> G = get_nodes();
        std::unique_lock<std::mutex> locker(mu);
        for (auto &&node : G) {
            cur=node;
            dist = distance(target, cur);
            if (dist < min) {
                min=dist;
                closest=cur;
            }
        }
        return closest;
    }

    double distance(shared_ptr<Node> &p, shared_ptr<Node> &q) {
        return std::abs(*p-*q);
    }
    shared_ptr<Node> get_last() {
        std::lock_guard<std::mutex> locker(mu);
        return nodes[nodes.size()-1];
    }
};
