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
class LTMCRegion;

template <typename LTMCImpl>
class LTMCPoint : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

public:
  MapImpl parent_map;
  double x;
  double y;

  LTMCPoint(uint entity_id, std::string name, double x, double y, MapImpl parent_map,
            LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), x(x), y(y), InstanceImpl(entity_id, name, ltmc)
  {
  }

  std::vector<RegionImpl> getContainingRegions()
  {
    assert(false);
  }

  bool operator==(const LTMCPoint& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name && this->x == other.x && this->y == other.y;
  }

  bool operator!=(const LTMCPoint& other) const
  {
    return this->entity_id != other.entity_id || this->name == other.name || this->x != other.x || this->y != other.y;
  }
  LTMCPoint& operator=(const LTMCPoint& that)
  {
    this->entity_id = that.entity_id;
    this->ltmc = that.ltmc;
  }
};
}  // namespace knowledge_rep
