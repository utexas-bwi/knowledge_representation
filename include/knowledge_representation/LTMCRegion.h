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

template <typename LTMCImpl>
class LTMCRegion : public LTMCInstance<LTMCImpl>
{
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  using PointImpl = LTMCPoint<LTMCImpl>;
  using PoseImpl = LTMCPose<LTMCImpl>;

public:
  MapImpl parent_map;
  std::vector<std::pair<double, double>> points;
  LTMCRegion(uint entity_id, std::string name, MapImpl parent_map, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), InstanceImpl(entity_id, name, ltmc)
  {
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
