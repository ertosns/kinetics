#include <iostream>
#include "../include/graph.hpp"

using namespace std;

std::ostream& operator<<(std::ostream& os, Point& p) {
    auto vec = p.vector();
    os << "(" << vec(0) << "," << vec(1) << ")";
    return os;
}

std::ostream& operator<<(std::ostream& os, Node &n) {
     //TODO (fix) if id, or ctg aren't set (infinity) then don't print them!
     Point p = n.get_point();
     os << "node(" << n.id << ")" << p << " - ctg: " << n.CTG;
     return os;
}

std::ostream& operator<<(std::ostream &os, Graph &g) {
    int i=0;
    for (auto &&n : g.get_path()) {
        os << " -> " <<  *n << i++;
    }
    return os;
}
