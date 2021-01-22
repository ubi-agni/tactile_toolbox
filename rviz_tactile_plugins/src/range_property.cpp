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
#include "range_property.h"
#include <rviz/properties/float_property.h>
#include <QLineEdit>

namespace rviz {
namespace tactile {

RangeFloatProperty::RangeFloatProperty(const QString &name, float fallback_value, const QString &description,
                                       Property *parent, const char *changed_slot, QObject *receiver)
  : rviz::FloatProperty(name, fallback_value, description, parent, changed_slot, receiver)
  , value_(fallback_value)
  , fallback_value_(fallback_value)
  , manually_edited_(false)
{
	setShouldBeSaved(false);
	Property::setValue("");
	setValue(fallback_value);
}

bool RangeFloatProperty::setValue(const QVariant &new_value)
{
	if (new_value == Property::getValue())
		return true;

	bool ok = false;
	bool manually_changed = manually_edited_;
	manually_edited_ = false;  // reset flag in any case

	float value = new_value.toFloat(&ok);
	if (ok) {
		if (manually_changed)
			setShouldBeSaved(true);
		value_ = value;
	} else {  // after invalid input: auto-compute the value
		setShouldBeSaved(false);  // don't save it anymore
		value_ = fallback_value_;
	}
	bool ret = Property::setValue(value_ == fallback_value_ ? QVariant("") : QVariant(value_));
	if (manually_changed)  // value manually changed?
		Q_EMIT edited();
	return ret;
}

QVariant RangeFloatProperty::getViewData(int column, int role) const
{
	if (role == Qt::FontRole && column == 1 && !manuallyEdited()) {
		QFont font;
		font.setItalic(true);
		return font;
	}
	return FloatProperty::getViewData(column, role);
}

void RangeFloatProperty::load(const Config &config)
{
	// value was only saved when manually changed before
	manually_edited_ = config.isValid();
	FloatProperty::load(config);
}

QWidget *RangeFloatProperty::createEditor(QWidget *parent, const QStyleOptionViewItem &option)
{
	// use own editor to allow for empty input
	QLineEdit *editor = new QLineEdit(parent);
	editor->setFrame(false);
	editor->setValidator(new EmptyOrDoubleValidator(editor));

	// allow to distinguish manual changes from programmatical ones in setValue()
	setManuallyEdited();
	// ensure that manually_edited_ is reset when editor gets destroyed
	connect(editor, &QLineEdit::destroyed, [this]() { manually_edited_ = false; });
	return editor;
}

RangeProperty::RangeProperty(const QString &name, const QString &description, Property *parent,
                             const char *changed_slot, QObject *receiver)
  : Property(name, "", description, parent, changed_slot, receiver), ignore_children_updates_(false)
{
	min_property_ = new RangeFloatProperty("minimum", FLT_MAX, description, this);
	max_property_ = new RangeFloatProperty("maximum", -FLT_MAX, description, this);

	// update our display from children
	updateFromChildren();
	connect(min_property_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
	connect(max_property_, SIGNAL(changed()), this, SLOT(updateFromChildren()));
	// forward edited() signal
	connect(min_property_, SIGNAL(edited()), this, SIGNAL(edited()));
	connect(max_property_, SIGNAL(edited()), this, SIGNAL(edited()));
}

void RangeProperty::reset()
{
	if (!min_property_->manuallyEdited())
		min_property_->setValue("");
	if (!max_property_->manuallyEdited())
		max_property_->setValue("");
}

void RangeProperty::update(const ::tactile::Range &range)
{
	if (range.min() != min_property_->getFloat())
		min_property_->setValue(range.min());
	if (range.max() != max_property_->getFloat())
		max_property_->setValue(range.max());
}

bool RangeProperty::updateFromChildren()
{
	if (ignore_children_updates_)
		return false;
	return Property::setValue(QString("%1; %2").arg(min_property_->getFloat()).arg(max_property_->getFloat()));
}

bool RangeProperty::setValue(const QVariant &new_value)
{
	if (new_value == Property::getValue())
		return true;
	QStringList values = new_value.toString().split(";", QString::KeepEmptyParts);
	if (values.size() != 2) {
		values.clear();
		values << ""
		       << "";
	}
	ignore_children_updates_ = true;
	min_property_->setManuallyEdited();
	min_property_->setValue(values[0]);
	max_property_->setManuallyEdited();
	max_property_->setValue(values[1]);
	ignore_children_updates_ = false;
	return updateFromChildren();
}

void RangeProperty::save(Config config) const
{
	// don't save our own value (as Property does), but only children
	for (int i = 0, end = numChildren(); i < end; ++i) {
		Property *prop = childAtUnchecked(i);
		if (prop && prop->shouldBeSaved())
			prop->save(config.mapMakeChild(prop->getName()));
	}
}

void RangeProperty::load(const Config &config)
{
	Property::load(config);
}

}  // namespace tactile
}  // namespace rviz
