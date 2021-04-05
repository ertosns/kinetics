#include "graph.hpp"

struct Environment {
    double width;
    double height;
};

class Robot {

};

class Kilobot : public Robot {
public:
    std::unique_ptr<Node> pos;
    std::unique_ptr<Node> end;
    double radius;
    double step;
    Kilobot(std::unique_ptr<Node> begining, std::unique_ptr<Node> target,
            double step_size=0.05, double _radius=0.009) :
        pos(std::move(begining)),
        end(std::move(target)),
        step(step_size),
        radius(_radius)
        {
            //
        }
};
