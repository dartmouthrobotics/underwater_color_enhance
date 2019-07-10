#ifndef METHOD_H
#define METHOD_H

#include "Scene.h"

#include <opencv2/opencv.hpp>

class Method {
public:
  Method(){};

  bool prior;
  bool est_veiling_light;
  Scene scene;

  virtual cv::Mat color_correct(cv::Mat& img) = 0;

private:
};

#endif  // METHOD_H
