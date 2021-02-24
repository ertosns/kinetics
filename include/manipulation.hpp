#include <string>
#include <exception>
#include <Eigen/Dense>
#include <igl/linprog.h>

using namespace std;
using namespace Eigen;

class NotFormClosure : public exception {
public:
    string what() {
        return "the Wrench Matrix isn't full Rank, make sure cols>=rows!";
    }
};

/** verify if the given vector are of closed form.
 *
 @param f: list of linear coefficient.
 @param A: matrix of inequality constraints.
 @param b: right hand side of the A's constraints vector.
 @param Aeq: matrix of equality constraints.
 @param beq: right hand side of Aeq's constraints vector.
 @return K: linear list minimizing those constraints.
*/
bool form_closure(const Eigen::VectorXd f,
                  const Eigen::MatrixXd A, const Eigen::VectorXd b,
                  const Eigen::MatrixXd Aeq, const Eigen::VectorXd beq,
                  Eigen::VectorXd K) {
    FullPivLU<Eigen::MatrixXd> lu_decomp(A);
    // check the full rank using ""rank-revealing decomposition"
    if (lu_decomp.rank()<A.rows()) {
        std::cerr << " not for closure not full rank" << std::endl;
        throw NotFormClosure();
    }
    bool res;
    try {
        res = igl::linprog(f, A, b, Aeq, beq, K);
    } catch(exception e) {
        std::cout << "Form Closure failed with Error: " << e.what() << std::endl;
        res=false;
    }
    return res;
}
