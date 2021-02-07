#pragma once

#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>
#include <map>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCPoint;
template <typename LTMCImpl>
class LTMCPose;
/// \brief An instance of the "region" concept which represents a closed polygon on some LTMCMap
template <typename LTMCImpl>
class LTMCRegion : public LTMCInstance<LTMCImpl>
{
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  using PointImpl = LTMCPoint<LTMCImpl>;
  using PoseImpl = LTMCPose<LTMCImpl>;

public:
  /// Shorthand for specifying 2D coordinates
  using Point2D = std::pair<double, double>;
  /// The map that owns this region
  MapImpl parent_map;
  /// The points that define the closed boundary of the region
  std::vector<Point2D> points;
  LTMCRegion(uint entity_id, std::string name, std::vector<Point2D> points, MapImpl parent_map,
             LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), points(std::move(points)), InstanceImpl(entity_id, name, ltmc)
  {
  }

  std::string getName() const
  {
    return this->name;
  }

  /**
   * @brief Gets the points that belong to the same map and are inside or on the bounds of the region
   * @return a list of contained points
   */
  std::vector<PointImpl> getContainedPoints()
  {
    return this->ltmc.get().getContainedPoints(*this);
  }

  /**
   * @brief Gets the poses that belong to the same map and are inside or on the bounds of the region
   * @return a list of contained poses
   */
  std::vector<PoseImpl> getContainedPoses()
  {
    return this->ltmc.get().getContainedPoses(*this);
  }

  /**
   * @brief Checks whether a point is in or on the bounds of the region
   * @return whether the point is contained in the region
   */
  bool isPointContained(double x, double y)
  {
    return this->ltmc.get().isPointContained(*this, x, y);
  }

  /**
   * @brief Checks whether a point is in or on the bounds of the region
   * @return whether the point is contained in the region
   */
  bool isPointContained(const Point2D& point)
  {
    return this->ltmc.get().isPointContained(*this, point.first, point.second);
  }

  /**
   * @brief Checks whether a point is in or on the bounds of the region
   * @return whether the point is contained in the region
   */
  bool isPointContained(const PointImpl& point)
  {
    return this->ltmc.get().isPointContained(*this, point.x, point.y);
  }

  /**
   * @brief Checks whether a pose is in or on the bounds of the region
   * @return whether the pose is contained in the region
   */
  bool isPoseContained(const PoseImpl& pose)
  {
    return this->ltmc.get().isPointContained(*this, pose.x, pose.y);
  }

  bool operator==(const LTMCRegion& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name && this->parent_map == other.parent_map &&
           this->points == other.points;
  }

  bool operator!=(const LTMCRegion& other) const
  {
    return this->entity_id != other.entity_id || this->name != other.name || this->parent_map != other.parent_map ||
           this->points != other.points;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCRegion<LTMCImpl>& r)
{
  strm << "Region(" << r.entity_id << " \"" << r.getName() << "\" " << r.parent_map << " (";
  for (const auto& p : r.points)
  {
    strm << p.first << "," << p.second << " ";
  }
  return strm << "))";
}

}  // namespace knowledge_rep
