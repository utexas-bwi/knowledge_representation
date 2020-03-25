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
    this->addAttribute("name", name);
  }

  std::string getName()
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
};
}  // namespace knowledge_rep
