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

RangeFloatProperty::RangeFloatProperty(const QString &name, float fallback_value,
                                       const QString &description, Property *parent,
                                       const char *changed_slot, QObject *receiver)
  : rviz::FloatProperty(name, fallback_value, description, parent, changed_slot, receiver)
  , value_(fallback_value)
  , fallback_value_(fallback_value)
{
  setManuallyEdited(false);
  Property::setValue("");
  setValue(fallback_value);
}

void RangeFloatProperty::setManuallyEdited(bool manual)
{
  manually_edited_ = manual;
  setShouldBeSaved(manually_edited_);
}

bool RangeFloatProperty::setValue(const QVariant &new_value)
{
  bool ok = false;
  float value = new_value.toFloat(&ok);
  if (ok) {
    value_ = value;
    bool ret = Property::setValue(value_ == fallback_value_ ? QVariant("") : QVariant(value_));
    if (manuallyEdited()) Q_EMIT edited();
    return ret;
  } else {
    setManuallyEdited(false);
    value_ = fallback_value_;
    return Property::setValue("");
  }
}

void RangeFloatProperty::save(Config &config) const
{
  if (manuallyEdited()) FloatProperty::save(config);
}
void RangeFloatProperty::load(const Config &config)
{
  setManuallyEdited(config.isValid());
  FloatProperty::load(config);
}

QWidget *RangeFloatProperty::createEditor(QWidget *parent, const QStyleOptionViewItem &option)
{
  // use own editor to allow for empty input
  QLineEdit *editor = new QLineEdit(parent);
  editor->setFrame(false);
  editor->setValidator(new EmptyOrDoubleValidator(editor));

  // allow to distinguish manual changes from programmatical ones:
  connect(editor, SIGNAL(editingFinished()), this, SLOT(setManuallyEdited()));
  return editor;
}


RangeProperty::RangeProperty(const QString &name, const QString &description,
                             Property *parent, const char *changed_slot, QObject *receiver)
  : Property (name, "", description, parent, changed_slot, receiver)
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
  min_property_->setFloat(range.min());
  max_property_->setFloat(range.max());
}

void RangeProperty::updateFromChildren()
{
  setValue(QString("%1; %2").arg(min_property_->getValue().toString(),
                                 max_property_->getValue().toString()));
}

void RangeProperty::save(Config config) const
{
  // don't save our own value (as Property does), but only children
  for(int i=0, end=numChildren(); i < end; ++i)
  {
    Property* prop = childAtUnchecked(i);
    if (prop && prop->shouldBeSaved())
      prop->save(config.mapMakeChild(prop->getName()));
  }
}

void RangeProperty::load(const Config &config)
{
  Property::load(config);
}

}
}
