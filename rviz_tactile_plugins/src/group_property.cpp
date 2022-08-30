/*
 * Copyright (C) 2016, Bielefeld University, CITEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <rviz_tactile_plugins/group_property.h>

namespace rviz {
namespace tactile {

GroupProperty::GroupProperty(const QString &name, bool default_value, const QString &description, Property *parent,
                             const char *changed_slot, QObject *receiver)
  : rviz::BoolProperty(name, default_value, description, parent, changed_slot, receiver)
{}

void GroupProperty::setBoolRecursively(bool new_value)
{
	blockSignals(true);
	setValue(new_value);
	blockSignals(false);
	for (int i = 0, end = numChildren(); i < end; ++i) {
		GroupProperty *child = dynamic_cast<GroupProperty *>(childAtUnchecked(i));
		if (child)
			child->setBoolRecursively(new_value);
	}
}

void GroupProperty::removeEmptyChildren()
{
	for (int i = numChildren() - 1; i >= 0; --i) {
		GroupProperty *child = dynamic_cast<GroupProperty *>(childAtUnchecked(i));
		if (!child)
			continue;
		child->removeEmptyChildren();
		if (child->numChildren() == 0)
			delete child;
	}
}

}  // namespace tactile
}  // namespace rviz
