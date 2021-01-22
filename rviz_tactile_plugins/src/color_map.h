/* ============================================================
 *
 * Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
 *
 * This file may be licensed under the terms of the
 * GNU Lesser General Public License Version 3 (the "LGPL"),
 * or (at your option) any later version.
 *
 * Software distributed under the License is distributed
 * on an ``AS IS'' basis, WITHOUT WARRANTY OF ANY KIND, either
 * express or implied. See the LGPL for the specific language
 * governing rights and limitations.
 *
 * You should have received a copy of the LGPL along with this
 * program. If not, go to http://www.gnu.org/licenses/lgpl.html
 * or write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The development of this software was supported by:
 *   CITEC, "Cognitive Interaction Technology" Excellence Cluster
 *     Bielefeld University
 *
 * ============================================================ */

#pragma once

#include <QColor>
#include <QList>
#include <QStringList>

namespace rviz {
namespace tactile {

class ColorMap
{
public:
	ColorMap(float fMin = 0, float fMax = 1);

	void init(float fMin = 0, float fMax = 1);
	void append(const QColor &c);
	void append(const QList<QColor> &cols);
	void append(const QStringList &names);

	/// map a value linearly onto the color map (assuming an input range from min..max)
	QColor map(float value) const;

private:
	QList<QColor> colors;
	float fMin, fMax;
};

}  // namespace tactile
}  // namespace rviz
