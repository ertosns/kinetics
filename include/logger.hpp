#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

//TODO choose wither to log or not.
class Logger {
public:  
  Logger(std::string file_name=std::string("kineticslib.csv")) {
    //TODO set file_name to timestamp in seconds.
    //
    //open file
    buff.open(file_name, std::ios::out|std::ios::app);
  }
  
  /** write out the matrix in csv such that each col is spread out in a single line, and the first line is for the matrix name.
   * 
   * @param name is the matrix name
   * @param mat is the matrix to log out.
   */
  void write(std::string tag, const Eigen::MatrixXd &mat) {
    buff << tag << std::endl;
    
    int nrow=mat.rows();
    int ncol=mat.cols();
    if (nrow==0)
      return;
    
    for (int r=0; r<nrow; r++) {
      for (int c=0; c<ncol; c++) {
        if (c==ncol-1)
          buff << mat(r,c) << std::endl;
        else
          buff << mat(r,c) << std::string(",");
      }
    }
  }
  
  /** write the vector in csv, spead out in a row, preceeded by the tag name
   *
   * @param tag tag name
   * @param vec vector to be logged
   */
  void write(std::string tag, const Eigen::VectorXd &vec) {
    buff << tag << std::endl;
    
    int size=vec.size();
    if(size==0)
      return;
    
    for (int i =0; i < size; i++) {
      if(i==size-1)
        buff << vec(i) << std::endl;
      else
        buff << vec(i) << std::string(",");
    }
  }
  
  /** write integer to the logging file
   *
   * @param tag tag name
   * @param i integer to be logged
   */
  void write(std::string tag, const int i) {
    buff << tag << std::endl <<
      i << std::endl;
  }
  
  /** write double to the logging file
   *
   * @param tag tag name
   * @param double to be logged
   */
  void write(std::string tag, const double d) {
    buff << tag << std::endl <<
      d << std::endl;
  }
  
  void close() {
    buff.close();
  }
  
private:
  std::fstream buff;
  //std::stringstream buf;
};