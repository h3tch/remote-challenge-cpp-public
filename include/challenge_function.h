#pragma once

#include <vector>

#include "point.h"

namespace chal {

	auto x_monotone_from_polygon(const std::vector<Point>& points)
		-> std::vector<Point>;

} //namespace chal