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
#pragma once

#include <rviz/properties/float_property.h>
#include <tactile_filters/Range.h>
#include <QDoubleValidator>

namespace rviz {
namespace tactile {

// double validator that allows for empty string too
class EmptyOrDoubleValidator : public QDoubleValidator {
  Q_OBJECT
public:
  EmptyOrDoubleValidator(QObject *parent) : QDoubleValidator(parent) {}
  QValidator::State validate(QString &input, int &pos) const {
    if (input.isEmpty()) return QValidator::Acceptable;
    return QDoubleValidator::validate(input, pos);
  }
};

class RangeFloatProperty : public rviz::FloatProperty {
  Q_OBJECT
public:
   RangeFloatProperty (const QString& name = QString(), float fallback_value = 0,
                       const QString& description = QString(), Property* parent = 0,
                       const char *changed_slot = 0, QObject* receiver = 0);
   bool  setValue(const QVariant& new_value);
   float getFloat() const {return value_;}
   bool  manuallyEdited() const {return manually_edited_;}

   void save(rviz::Config &config) const;
   void load(const rviz::Config &config);

protected:
   QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

protected Q_SLOTS:
   void setManuallyEdited(bool manual=true);

Q_SIGNALS:
   void edited();

protected:
   float value_;
   float fallback_value_;
   bool  manually_edited_;
};


class RangeProperty : public rviz::Property
{
  Q_OBJECT
public:
  RangeProperty(const QString& name = QString(),
                const QString& description = QString(),
                Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0);

  void  save(Config config) const;
  void  reset();
  void  update(const ::tactile::Range &range);
  float min() const {return min_property_->getFloat();}
  float max() const {return max_property_->getFloat();}

  void load(const Config &config);
protected Q_SLOTS:
  void updateFromChildren();

Q_SIGNALS:
   void edited();

protected:
  RangeFloatProperty *min_property_;
  RangeFloatProperty *max_property_;
};

}
}
