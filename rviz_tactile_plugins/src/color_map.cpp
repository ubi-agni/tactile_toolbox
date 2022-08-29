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

#include "rviz_tactile_plugins/color_map.h"
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
	for (const auto &name : names)
		colors.append(QColor(name));
}

QColor interpolate(const QColor &lo, const QColor &hi, float b)
{
	float a = 1.0 - b;
	return QColor(a * lo.red() + b * hi.red(), a * lo.green() + b * hi.green(), a * lo.blue() + b * hi.blue(),
	              a * lo.alpha() + b * hi.alpha());
}

QColor ColorMap::map(float value) const
{
	static const QColor ERR_COLOR("magenta");
	if (!std::isfinite(value))
		return ERR_COLOR;

	const int N = colors.size() - 1;  // NOLINT(readability-identifier-naming)
	assert(N > 0);

	float ratio = (value - fMin) / (fMax - fMin) * N;
	if (ratio < 0)  // indicate undershooting with smooth transition to errColor
		return interpolate(colors[0], ERR_COLOR, 0.5f * std::min<float>(-ratio, 1.0f));

	int idx = ratio;
	if (idx >= N)  // indicate overshooting with smooth transition to errColor
		return interpolate(colors.last(), ERR_COLOR, 0.5f * std::min<float>(ratio - N, 1.0f));

	return interpolate(colors[idx], colors[idx + 1], ratio - idx);
}

}  // namespace tactile
}  // namespace rviz
