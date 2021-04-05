#include "graph.hpp"

struct Environment {
    double width;
    double height;
};

class Robot {

};

class Kilobot : public Robot {
public:
    std::shared_ptr<Node> pos;
    std::shared_ptr<Node> end;
    double radius;
    double step;
    Kilobot(std::shared_ptr<Node> begining, std::shared_ptr<Node> target,
            double _radius=0.009) :
        pos(begining),
        end(target),
        step(step_size),
        radius(_radius) {
        //
    }
};
