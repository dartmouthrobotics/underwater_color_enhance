#pragma once
#include "Scene.h"
#include "Method.h"

#include <opencv2/opencv.hpp>

using namespace cv;

class ColorCorrect {
public:
  ColorCorrect(){};
  ColorCorrect(Scene& new_scene, int new_method, bool is_adaptive, bool optimize);

  Mat enhance(Mat& img);

private:
  Scene underwater_scene;
  Method *method;
  bool is_adaptive;
  bool optimize;
};
