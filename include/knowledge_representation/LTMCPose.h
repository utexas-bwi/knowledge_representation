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
class LTMCPose : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  std::string name;
  InstanceImpl parent_map;

public:
  LTMCPose(uint entity_id, std::string name, uint parent_map_entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
      : name(std::move(name)), parent_map(parent_map_entity_id, ltmc), InstanceImpl(entity_id, ltmc)
  {
  }

};
}  // namespace knowledge_rep