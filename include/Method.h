#ifndef METHOD_H
#define METHOD_H

#include "Scene.h"

#include <opencv2/opencv.hpp>
#include <tinyxml.h>

class Method {
public:
  Method(){};

  bool prior;
  bool est_veiling_light;
  Scene scene;

  TiXmlDocument out_doc;
  bool save_data;

  virtual cv::Mat color_correct(cv::Mat& img) = 0;

private:
};

#endif  // METHOD_H
