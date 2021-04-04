#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

//TODO choose wither to log or not.
class Logger {
public:
    Logger(std::string file_name=std::string("/tmp/kinetics.log"), bool _auto_flush=true) :
        fname(file_name), auto_flush(_auto_flush) {
        //TODO set file_name to timestamp in seconds.
        //
        //open file
        buff.open(file_name, std::ios::out|std::ios::trunc);
    }
    Logger(const Logger &copy) {
        //TODO
        //copy.close();
        buff.open(copy.fname, std::ios::out|std::ios::trunc);
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
    if (auto_flush)
        flush();
  }

  /** write a csv line
   *
   * @param vals std::vector of double values to write in csv
   */
  template<typename T>
  void csv_line(std::vector<T> vals) {
    //add csv line
    for (int i =0; i < vals.size(); i++) {
      buff << vals[i];
      if (i!=vals.size()-1)
        buff << ",";
    }
    // endline
    buff << std::endl;
    if (auto_flush)
        flush();
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
    if (auto_flush)
        flush();
  }

  /** write the vector in csv, spead out in a row, preceeded by the tag name
   *
   * @param vec vector to be logged
   */
  void write(const Eigen::VectorXd &vec) {
    int size=vec.size();
    if(size==0)
      return;

    for (int i =0; i < size; i++) {
      if(i==size-1)
        buff << vec(i) << std::endl;
      else
        buff << vec(i) << std::string(",");
    }
    if (auto_flush)
        flush();
  }


  /** write integer to the logging file
   *
   * @param tag tag name
   * @param i integer to be logged
   */
  void write(std::string tag, const int i) {
    buff << tag << std::endl <<
      i << std::endl;
    if (auto_flush)
        flush();
  }

  void write(const int i, bool end=false) {
    if (end)
      buff << i <<  std::endl;
    else
      buff << i << ",";
    if (auto_flush)
        flush();
  }

  void write(const double i, bool end=false) {
    if (end)
      buff << i << std::endl;
    else
      buff << i << ",";
    if (auto_flush)
        flush();
  }

  /** write double to the logging file
   *
   * @param tag tag name
   * @param double to be logged
   */
  void write(std::string tag, const double d) {
    buff << tag << std::endl <<
      d << std::endl;
    if (auto_flush)
        flush();
  }

    void flush() {
        buff.flush();
    }

    void close() {
        buff.flush();
        buff.close();
    }

protected:
    std::string fname;
    std::fstream buff;
    bool auto_flush;
    //std::stringstream buf;
};
