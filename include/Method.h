#ifndef METHOD_H
#define METHOD_H

#include "Scene.h"

#include <opencv2/opencv.hpp>
#include <tinyxml.h>

class Method {
public:
  Method(){};

  bool est_veiling_light;
  Scene scene;

  bool prior_data;
  std::string input_filename;

  TiXmlDocument out_doc;
  bool save_data;
  bool file_initialized;

  virtual cv::Mat color_correct(cv::Mat& img) = 0;
  virtual void end_file(std::string output_filename) = 0;

  virtual void load_data(std::string input_filename) = 0;

private:
};

#endif  // METHOD_H
