#ifndef SCENE_H
#define SCENE_H

#include <vector>

class Scene {
public:
  float distance;
  float depth;
  std::vector<int> background_sample;

private:
};

#endif  // SCENE_H
