#ifndef VIEWMORPHER_H
#define VIEWMORPHER_H

#include "helpers.h"

class ViewMorpher {
public:
  ViewMorpher();
  ~ViewMorpher();

  std::vector<camera_frame>& getViews();
  camera_frame& getView(int view_id);
  int getNumberOfViews();

  void getMorphedView(camera_frame& cam_frame);

private:
  std::vector<camera_frame> views;
};

#endif
