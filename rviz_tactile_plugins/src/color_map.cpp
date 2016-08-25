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

#include "color_map.h"
#include <assert.h>
#include <cmath>

namespace rviz {
namespace tactile {

ColorMap::ColorMap(float fMin, float fMax)
{
	init(fMin, fMax);
}

void ColorMap::init(float fMin, float fMax)
{
	this->colors.clear();
	this->fMin = fMin;
	this->fMax = fMax;
}

void ColorMap::append(const QColor &c)
{
	colors.append(c);
}

void ColorMap::append(const QList<QColor> &cols)
{
	colors += cols;
}

void ColorMap::append(const QStringList &names)
{
	for (QStringList::const_iterator it=names.begin(), end=names.end(); it!=end; ++it)
		colors.append(QColor(*it));
}

QColor ColorMap::map(float value) const
{
	static const QColor errColor("cyan");
	assert(colors.size() > 1);
	if (std::isnan(value)) return errColor;

	float ratio = (value-fMin) / (fMax-fMin) * (colors.size()-1);
	if (ratio < 0) return colors[0];
	int   idx = ratio;
	if (idx >= colors.size()-1) return colors.last();

	float b = ratio - idx; // in [0..1)
	float a = 1.0 - b;
	const QColor& lo = colors[idx];
	const QColor& hi = colors[idx+1];
	return QColor(a*lo.red()+b*hi.red(), a*lo.green()+b*hi.green(), a*lo.blue()+b*hi.blue(), a*lo.alpha()+b*hi.alpha());
}

}
}
