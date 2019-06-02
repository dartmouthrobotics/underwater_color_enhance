#include "Scene.h"
#include "ColorCorrect.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <ctime>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
  if(argc == 1)
  {
    cout << "Usage: " << argv[0] << " <path_to_image>" << endl;
    exit(0);
  }

  // original image to be color corrected
  Mat img = imread(argv[1]);

  // TO DO: have this as input from a YAML file or from command line input
  float distance = 0.33;
  float depth = 6.06;
  int method = 0;

  // WHAT??
  bool is_adaptive = false;

  // NOTE: if optimized then we need to provide the current attenuation values
  bool optimize = false;

  bool showImg = true;

  Scene underwater_scene;
  underwater_scene.distance = distance;
  underwater_scene.depth = depth;

  std::clock_t begin = clock();

  ColorCorrect correction_method(underwater_scene, method, is_adaptive, optimize);

  Mat corrected_frame = correction_method.enhance(img);

  std::clock_t end = clock();

  std::cout << "Elapsed time: " << double(end - begin) / CLOCKS_PER_SEC << std::endl;

  if(showImg)
  {
    imshow("Original", img);
    imshow("Corrected", corrected_frame);
    waitKey(0);
  }

  return 0;
}
