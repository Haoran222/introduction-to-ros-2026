#pragma once

// Catmull-Rom spline sampling for a sparse 2D waypoint route.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace planning {

struct Point2D {
  double x{0.0};
  double y{0.0};
};

struct SplineSample {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double curvature{0.0};
  double arc_length{0.0};  // cumulative distance from the first sample
};

namespace detail {

inline double Distance(const Point2D &a, const Point2D &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

// Uniform Catmull-Rom basis and its first two derivatives.
inline void EvalSegment(
    const Point2D &p0,
    const Point2D &p1,
    const Point2D &p2,
    const Point2D &p3,
    double t,
    Point2D *pos,
    Point2D *vel,
    Point2D *acc) {
  const double t2 = t * t;

  const double ax = 2.0 * p0.x - 5.0 * p1.x + 4.0 * p2.x - p3.x;
  const double ay = 2.0 * p0.y - 5.0 * p1.y + 4.0 * p2.y - p3.y;
  const double bx = 3.0 * p1.x - 3.0 * p2.x + p3.x - p0.x;
  const double by = 3.0 * p1.y - 3.0 * p2.y + p3.y - p0.y;

  if (pos != nullptr) {
    pos->x = 0.5 * (2.0 * p1.x + (p2.x - p0.x) * t + ax * t2 + bx * t2 * t);
    pos->y = 0.5 * (2.0 * p1.y + (p2.y - p0.y) * t + ay * t2 + by * t2 * t);
  }

  if (vel != nullptr) {
    vel->x = 0.5 * ((p2.x - p0.x) + 2.0 * ax * t + 3.0 * bx * t2);
    vel->y = 0.5 * ((p2.y - p0.y) + 2.0 * ay * t + 3.0 * by * t2);
  }

  if (acc != nullptr) {
    acc->x = 0.5 * (2.0 * ax + 6.0 * bx * t);
    acc->y = 0.5 * (2.0 * ay + 6.0 * by * t);
  }
}

}  // namespace detail

// Remove consecutive duplicate points.
inline std::vector<Point2D> DedupPoints(
    const std::vector<Point2D> &points,
    double epsilon = 1e-3) {
  std::vector<Point2D> out;
  out.reserve(points.size());

  for (const auto &p : points) {
    if (out.empty() || detail::Distance(out.back(), p) > epsilon) {
      out.push_back(p);
    }
  }
  return out;
}

// `points` must already be deduplicated and have at least 2 entries.
inline std::vector<SplineSample> SampleCatmullRom(
    const std::vector<Point2D> &points,
    double sample_spacing) {
  std::vector<SplineSample> samples;
  if (points.size() < 2 || sample_spacing <= 0.0) {
    return samples;
  }

  if (points.size() == 2) {
    // Not enough points to form a spline segment; fall back to a straight line.
    const double yaw = std::atan2(points[1].y - points[0].y, points[1].x - points[0].x);
    const double length = detail::Distance(points[0], points[1]);
    const int n = std::max(1, static_cast<int>(std::ceil(length / sample_spacing)));

    for (int i = 0; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      SplineSample s;
      s.x = points[0].x + t * (points[1].x - points[0].x);
      s.y = points[0].y + t * (points[1].y - points[0].y);
      s.yaw = yaw;
      s.curvature = 0.0;
      s.arc_length = t * length;
      samples.push_back(s);
    }

    return samples;
  }

  // Extrapolate endpoint control points.
  std::vector<Point2D> ctrl;
  ctrl.reserve(points.size() + 2);
  const Point2D before{
      2.0 * points[0].x - points[1].x,
      2.0 * points[0].y - points[1].y,
  };
  ctrl.push_back(before);
  ctrl.insert(ctrl.end(), points.begin(), points.end());

  const size_t last = points.size() - 1;
  const Point2D after{
      2.0 * points[last].x - points[last - 1].x,
      2.0 * points[last].y - points[last - 1].y,
  };
  ctrl.push_back(after);

  double cumulative_length = 0.0;
  Point2D prev_pos = points[0];
  bool have_prev = false;

  const size_t num_segments = points.size() - 1;
  for (size_t seg = 0; seg < num_segments; ++seg) {
    const Point2D &p0 = ctrl[seg];
    const Point2D &p1 = ctrl[seg + 1];
    const Point2D &p2 = ctrl[seg + 2];
    const Point2D &p3 = ctrl[seg + 3];

    const double seg_length_estimate = detail::Distance(p1, p2);
    const int n = std::max(
        1,
        static_cast<int>(std::ceil(seg_length_estimate / sample_spacing)));

    // Skip t=0 after the first segment: it duplicates the previous segment's t=1.
    const int start_i = (seg == 0) ? 0 : 1;

    for (int i = start_i; i <= n; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(n);
      Point2D pos, vel, acc;
      detail::EvalSegment(p0, p1, p2, p3, t, &pos, &vel, &acc);

      if (have_prev) {
        cumulative_length += detail::Distance(prev_pos, pos);
      }

      prev_pos = pos;
      have_prev = true;

      const double speed_sq = vel.x * vel.x + vel.y * vel.y;
      double curvature = 0.0;
      if (speed_sq > 1e-9) {
        curvature = (vel.x * acc.y - vel.y * acc.x) / std::pow(speed_sq, 1.5);
      }

      SplineSample s;
      s.x = pos.x;
      s.y = pos.y;
      s.yaw = std::atan2(vel.y, vel.x);
      s.curvature = curvature;
      s.arc_length = cumulative_length;
      samples.push_back(s);
    }
  }

  return samples;
}

}  // namespace planning
