#pragma once

#include <rviz/display_group.h>

namespace rviz {
class IntProperty;

namespace tactile {

class GloveDisplayGroup : public rviz::DisplayGroup
{
	Q_OBJECT

public:
	GloveDisplayGroup();
	~GloveDisplayGroup() override;

public slots:
	void onNumChanged();

private:
	rviz::IntProperty* num_property_;
};

}  // namespace tactile
}  // namespace rviz
