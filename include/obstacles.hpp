#define OBSTACLES
#include <vector>
#include <Eigen/Dense>
//#include <QDebug>
//#include <QLine>

class Point {  
public:
  Point(Eigen::VectorXd p): p_(p){
  }
  
  /** calculate the distance from the current point to point p.
   *
   *@param p given point
   *@return distance
   */
  double operator-(Point p) {
    return sqrt((p.vector().array() - vector().array()).square().sum());
  }
  /*
  double operator-(Point *p) {
    return sqrt((p->vector().array() - vector().array()).square().sum());
  }
  */
  /** verify that p is the same as the current point
   *
   *@param p given point
   *@return boolean check
   */
  bool operator==(Point &p) {
    return *this-p==0;
  }

  Eigen::VectorXd vector() {
    return p_;
  }
  
  friend std::ostream& operator<<(std::ostream&, const Point&);
private:
  const Eigen::VectorXd p_;
};

std::ostream& operator<<(std::ostream& os, Point& p) {
  auto vec = p.vector();
  os << "(" << vec(0) << "," << vec(1) << ")";
  return os;
}

class Obstacle
{
public:
  Obstacle() {
  }
  /** does this give line with either of the two points are ouside the obstacle lies inside it?
   *
   * @param first point (outside the obstacle
   * @param second point (uncertain)
   */
  //TODO (the same problem virtual fails to compile!
  //virtual bool intersect(Point &p1, Point &p2);
};

class CircleObs : public Obstacle {
public:
  /** Construct a circular obstacle with:
   * using the 
   * @param c coordinates of the center
   * @param r radius of the circle
   */
  CircleObs(Point c, double r): Obstacle(), center(c), radius(r) {
    
  }

  /** calculate the distance of the perpendicular to the center.
   * for line equation Ax + By + C = 0
   * and circle of point (m, n)
   * d = |Am + Bn + c| / sqrt(A^2 + B^2)
   * equation of line with slop s = y-y1/(x-x1)
   * line P2-P1 equation = y - sx - y1 + sx1=0
   * B = 1, A = -s, C = -y1 + sx1
   */
  bool intersect(Point p1, Point p2, double robot_radius=0) {
    
    Eigen::VectorXd p1_vec = p1.vector();
    double x1=p1_vec(0);
    double y1=p1_vec(1);
    Eigen::VectorXd p2_vec = p2.vector();
    double x2=p2_vec(0);
    double y2=p2_vec(1);
    double s = (y2 - y1)/(x2 - x1);
    double A = -1*s;
    double B = 1;
    double C = -1*y1 + s*x1;
    Eigen::VectorXd c_vec = center.vector();
    double m = c_vec(0);
    double n = c_vec(1);
    double distance = std::abs(-1*s*m + n - y1 + s*x1) /
      std::sqrt(1+std::pow(s,2));
    bool inter = distance <= radius+robot_radius;
    if(inter)
      std::cout << "*** the distance between line P1P2 and center::" << center << " is: "  << distance << std::endl;
    return inter;
  }
  
private:
  Point center;
  double radius;
};

class RectangleObs : public Obstacle {
public:
  /** Construct a rectangular obstacle with:
   *
   * @param tl as top left corner coordinates.
   * @param br as bottom right corner coordinates.
   */
  RectangleObs(Point _tl, Point _br): tl(_tl), br(_br){
  }
  bool intersect(Point &p1, Point &p2) {
    /*
    // add obs
    // Get topLeft and bottomRight points from the given points.
    Vector2f tmp;
    if (firstPoint.x() > secondPoint.x() && firstPoint.y() > secondPoint.y()) {
    tmp = firstPoint;
    firstPoint = secondPoint;
    secondPoint = tmp;
    } else if (firstPoint.x() < secondPoint.x() && firstPoint.y() > secondPoint.y()) {
    int height = firstPoint.y() - secondPoint.y();
    firstPoint.y() -= height;
    secondPoint.y() += height;
    } else if (firstPoint.x() > secondPoint.x() && firstPoint.y() < secondPoint.y()) {
    int length = firstPoint.x() - secondPoint.x();
    firstPoint.x() -= length;
    secondPoint.x() += length;
    }
    firstPoint.x() -= BOT_CLEARANCE;
    firstPoint.y() -= BOT_CLEARANCE;
    secondPoint.x() += BOT_CLEARANCE;
    secondPoint.y() += BOT_CLEARANCE;
    obstacles.push_back(make_pair(firstPoint, secondPoint));
    */
    /*
      QLineF lineSegment(p1.x(), p1.y(), p2.x(), p2.y());
      QPointF *intersectPt = new QPointF;
      for(int i = 0; i < (int)obstacles.size(); i++) {
      float length = obstacles[i].second.x() - obstacles[i].first.x();
      float breadth = obstacles[i].second.y() - obstacles[i].first.y();
      QLineF lseg1(obstacles[i].first.x(), obstacles[i].first.y(), obstacles[i].first.x() + length, obstacles[i].first.y());
      QLineF lseg2(obstacles[i].first.x(), obstacles[i].first.y(), obstacles[i].first.x(), obstacles[i].first.y() + breadth);
      QLineF lseg3(obstacles[i].second.x(), obstacles[i].second.y(), obstacles[i].second.x(), obstacles[i].second.y() - breadth);
      QLineF lseg4(obstacles[i].second.x(), obstacles[i].second.y(), obstacles[i].second.x() - length, obstacles[i].second.y());
      QLineF::IntersectType x1 = lineSegment.intersect(lseg1, intersectPt);
      QLineF::IntersectType x2 = lineSegment.intersect(lseg2, intersectPt);
      QLineF::IntersectType x3 = lineSegment.intersect(lseg3, intersectPt);
      QLineF::IntersectType x4 = lineSegment.intersect(lseg4, intersectPt);
      // check for bounded intersection. IntersectType for bounded intersection is 1.
      if (x1 == 1 || x2 == 1 || x3 == 1 || x4 == 1)
      return true;
      }
      return false;
    */
    return false;
  }
private:
  Point tl;
  Point br;
};

//TODO after fixing the issue with virtual function, update this to template Obstacle instead
typedef std::vector<CircleObs*> Obstacles;
