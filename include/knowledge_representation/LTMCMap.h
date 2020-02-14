#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCMap : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using PointImpl = LTMCPoint<LTMCImpl>;
  using PoseImpl = LTMCPose<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

public:
  LTMCMap(uint entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : InstanceImpl(entity_id, name, ltmc)
  {
  }
  PointImpl addPoint(const std::string& name, float x, float y)
  {
    return this->ltmc.get().addPoint(*this, name, x, y);
  }

  PoseImpl addPose(const std::string& name, float x, float y, float theta)
  {
    return this->ltmc.get().addPose(*this, name, x, y, theta);
  }

  RegionImpl addRegion(const std::string& name, const std::vector<std::pair<float, float>>& points)
  {
    return this->ltmc.get().addRegion(*this, name, points);
  }
};
}  // namespace knowledge_rep
