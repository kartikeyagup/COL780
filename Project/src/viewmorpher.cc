#include "viewmorpher.h"

ViewMorpher::ViewMorpher() {
  // default constructor
}

ViewMorpher::~ViewMorpher() {
  // default destructor
}

std::vector<camera_frame>& ViewMorpher::getViews() {
  return views;
}

camera_frame& ViewMorpher::getView(int view_id) {
  return views[view_id];
}

void ViewMorpher::getMorphedView(camera_frame& cam_frame) {
  views[0].frame.copyTo(cam_frame.frame); 
}
