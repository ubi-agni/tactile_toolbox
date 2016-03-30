#include <pluginlib/class_list_macros.h>
#include "tactile_state_display.h"
#include "tactile_contact_display.h"

PLUGINLIB_EXPORT_CLASS(rviz::tactile::TactileStateDisplay, rviz::Display);
PLUGINLIB_EXPORT_CLASS(rviz::tactile::TactileContactDisplay, rviz::Display);
