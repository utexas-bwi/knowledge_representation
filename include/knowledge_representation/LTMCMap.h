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
  std::string getName()
  {
    return this->name;
  }
  PointImpl addPoint(const std::string& name, double x, double y)
  {
    return this->ltmc.get().addPoint(*this, name, x, y);
  }

  PoseImpl addPose(const std::string& name, double x, double y, double theta)
  {
    return this->ltmc.get().addPose(*this, name, x, y, theta);
  }

  RegionImpl addRegion(const std::string& name, const std::vector<std::pair<double, double>>& points)
  {
    return this->ltmc.get().addRegion(*this, name, points);
  }
  boost::optional<PointImpl> getPoint(const std::string& name)
  {
    return this->ltmc.get().getPoint(*this, name);
  }
  boost::optional<PoseImpl> getPose(const std::string& name)
  {
    return this->ltmc.get().getPose(*this, name);
  }
  boost::optional<RegionImpl> getRegion(const std::string& name)
  {
    return this->ltmc.get().getRegion(*this, name);
  }
  std::vector<PointImpl> getAllPoints()
  {
    return this->ltmc.get().getAllPoints(*this);
  }
  std::vector<PoseImpl> getAllPoses()
  {
    return this->ltmc.get().getAllPoses(*this);
  }
  std::vector<RegionImpl> getAllRegions()
  {
    return this->ltmc.get().getAllRegions(*this);
  }
};
}  // namespace knowledge_rep
