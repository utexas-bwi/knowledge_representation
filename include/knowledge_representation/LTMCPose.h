#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCPose : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  MapImpl parent_map;

public:
  LTMCPose(uint entity_id, std::string name, MapImpl parent_map, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), InstanceImpl(entity_id, name, ltmc)
  {
  }
};
}  // namespace knowledge_rep
