#include "glove_display_group.h"
#include <rviz/properties/int_property.h>

namespace rviz {
namespace tactile {

GloveDisplayGroup::GloveDisplayGroup() : DisplayGroup()
{
	num_property_ = new IntProperty("children", 0, "", this, SLOT(onNumChanged()));
	num_property_->setMin(0);
}

GloveDisplayGroup::~GloveDisplayGroup() {}

void GloveDisplayGroup::onNumChanged()
{
	int target_num = num_property_->getInt();
	// remove extra displays
	for (int i = numDisplays(); i > target_num; --i)
		delete takeDisplay(getDisplayAt(i - 1));
	for (int i = numDisplays(); i < target_num; ++i) {
		auto *display = createDisplay("rviz_tactile_plugins/Tactile Contact Display");
		display->setName(QString("Tactile Contact Display %1").arg(i + 1));
		display->initialize(context_);
		display->setEnabled(true);
		addDisplay(display);
	}
}

}  // end namespace tactile
}  // end namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::tactile::GloveDisplayGroup, rviz::Display);
