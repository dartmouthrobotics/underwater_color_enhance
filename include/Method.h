#pragma once
#include "Scene.h"

#include <opencv2/opencv.hpp>

using namespace cv;

class Method {
public:
  Method(){};

  bool prior;
  Scene scene;

  virtual Mat color_correct(Mat& img) = 0;

private:
};
