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
  using Point2D = std::pair<double, double>;
  MapImpl parent_map;
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

  std::vector<PointImpl> getContainedPoints()
  {
    assert(false);
  }

  std::vector<PoseImpl> getContainedPoses()
  {
    assert(false);
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
