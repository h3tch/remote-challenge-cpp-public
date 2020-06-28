
#include <algorithm>
#include <vector>
#include <tuple>
#include <deque>

#include "challenge_function.h"

namespace chal {

	const static size_t invalid_index = size_t(-1);

	auto split_into_lower_and_upper_boundary(const std::vector<Point>& points)
		-> std::pair<std::vector<Point>, std::vector<Point>>
	{
		auto lowest_left = points.begin();
		auto highest_right = points.begin();
		for (auto point = ++points.begin(); point < points.end(); ++point) {
			if (point->x < lowest_left->x or (point->x == lowest_left->x and point->y < lowest_left->y))
				lowest_left = point;
			if (point->x > highest_right->x or (point->x == highest_right->x and point->y > highest_right->y))
				highest_right = point;
		}
		
		std::vector<Point> lower_points;
		std::vector<Point> upper_points;

		const auto distance = std::distance(lowest_left, highest_right);
		const auto abs_distance = static_cast<size_t>(std::abs(distance));

		if (distance < 0) {
			lower_points.reserve(points.size() - abs_distance);
			lower_points.insert(lower_points.end(), lowest_left, points.end());
			lower_points.insert(lower_points.end(), points.begin(), highest_right);
		} else {
			lower_points.reserve(abs_distance);
			lower_points.insert(lower_points.end(), lowest_left, highest_right);
		}

		if (distance > 0) {
			upper_points.reserve(abs_distance);
			upper_points.insert(upper_points.end(), highest_right, points.end());
			upper_points.insert(upper_points.end(), points.begin(), lowest_left);
		} else {
			upper_points.reserve(points.size() - abs_distance);
			upper_points.insert(upper_points.end(), highest_right, lowest_left);
		}

		return {lower_points, upper_points};
	}

	auto is_boundary_turning_left(const Point& point, std::deque<Point>& boundary)
		-> bool
	{
		Point& middle_point = boundary.back();
		Point& first_point = boundary.at(boundary.size()-2);

		auto ax = middle_point.x - first_point.x;
		auto ay = middle_point.y - first_point.y;
		auto bx = point.x - first_point.x;
		auto by = point.y - first_point.y;
		
		auto determinant = ax * by - ay * bx;

		return determinant > 0;
	}

	auto find_next_edge_intersecting_x(const std::vector<Point>& points, const size_t start, const double x)
		-> size_t
	{
		for (size_t i = start; i < points.size() - 1; ++i) {
			const auto& first = points[i];
			const auto& second = points[i + 1];
			if (first.x <= x and second.x > x)
				return i;
		}
		return invalid_index;
	}

	auto edge_point_at_x(const Point& first, const Point& second, const double x)
		-> Point
	{
		const auto vx = second.x - first.x;
		if (vx == 0.0)
			return first;
		const auto vy = second.y - first.y;
		const auto t = (x - first.x) / vx;
		return {first.x + vx * t, first.y + vy * t};
	}

	auto remove_edges_to_the_right(std::deque<Point>& stack, const double x)
		-> Point
	{
		auto second = stack.back();
		stack.pop_back();
		while (stack.size() > 1) {
			const auto first = stack.back();
			if (first.x <= x and second.x > x)
				break;
			stack.pop_back();
			second = first;
		}
		return second;
	}

	auto skip_boundary_points_to_the_left(const std::vector<Point>& points,
                                          const size_t i,
                                          const Point& end_point,
                                          std::deque<Point>& result)
		-> size_t
	{
		const auto first_edge_point_index = find_next_edge_intersecting_x(points, i, result.back().x);

		if (first_edge_point_index == invalid_index) {
			const auto new_point = edge_point_at_x(points[points.size() - 1], end_point, result.back().x);
			// make sure we do not add the endpoint
			if (new_point.x != result.back().x or new_point.y != result.back().y)
				result.push_back(new_point);
			return points.size();
		}

		const auto& first = points[first_edge_point_index];
		const auto& second = points[first_edge_point_index + 1];
		const auto new_point = edge_point_at_x(first, second, result.back().x);
		result.push_back(new_point);
		return first_edge_point_index;
	}

	void remove_boundary_points_to_the_right(const Point& point, std::deque<Point>& result)
	{
		const auto second_edge_point = remove_edges_to_the_right(result, point.x);
		if (point.x != result.back().x or point.y != result.back().y) {
			const auto new_point = edge_point_at_x(result.back(), second_edge_point, point.x);
			result.push_back(new_point);
		}
	}

	auto make_x_monotone(const std::vector<Point>& points, const Point& end_point)
		-> std::deque<Point>
	{
		std::deque<Point> result;
		result.push_back(points[0]);

		for (size_t i = 1; i < points.size(); ++i) {
			const auto& point = points[i];

			if (point.x >= result.back().x) {
				result.push_back(point);
			} else if (is_boundary_turning_left(point, result)) {
				i = skip_boundary_points_to_the_left(points, i, end_point, result);
			} else {
				remove_boundary_points_to_the_right(point, result);
				result.push_back(point);
			}
		}

		return result;
	}

	auto x_monotone_from_polygon(const std::vector<Point>& points)
		-> std::vector<Point>
	{
		// no polygon
		if (points.size() < 2)
			return points;

		auto [lower_points, upper_points] = split_into_lower_and_upper_boundary(points);

		auto lower = make_x_monotone(lower_points, upper_points[0]);

		std::for_each(upper_points.begin(), upper_points.end(), [](auto& p){ p.x *= -1; p.y *= -1; });

		auto upper = make_x_monotone(upper_points, {-lower_points[0].x, -lower_points[0].y});

		std::vector<Point> result;
		result.reserve(lower.size() + upper.size());
		result.insert(result.end(), lower.begin(), lower.end());
		for (auto i = upper.begin(); i < upper.end(); ++i)
			result.push_back({-i->x, -i->y});

		return result;
	}

} //namespace chal