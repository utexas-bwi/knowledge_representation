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
/// \brief An instance of the Door concept, which stores a line with respect to some LTMCMap
template <typename LTMCImpl>
class LTMCDoor : public LTMCInstance<LTMCImpl>
{
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using MapImpl = LTMCMap<LTMCImpl>;

public:
  /// The map that owns this door
  MapImpl parent_map;

  /// The x component of the first point of the door line
  double x_0;
  /// The y component of the first point of the door line
  double y_0;
  /// The x component of the second point of the door line
  double x_1;
  /// The y component of the second point of the door line
  double y_1;

  LTMCDoor(uint entity_id, std::string name, double x_0, double y_0, double x_1, double y_1, MapImpl parent_map,
           LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : parent_map(parent_map), x_0(x_0), y_0(y_0), x_1(x_1), y_1(y_1), InstanceImpl(entity_id, name, ltmc)
  {
  }

  std::string getName() const
  {
    return this->name;
  }

  bool operator==(const LTMCDoor& other) const
  {
    return this->entity_id == other.entity_id && this->name == other.name && this->x_0 == other.x_0 &&
           this->y_0 == other.y_0 && this->x_1 == other.x_1 && this->y_1 == other.y_1;
  }

  bool operator!=(const LTMCDoor& other) const
  {
    return this->entity_id != other.entity_id || this->name == other.name || this->x_0 != other.x_0 ||
           this->y_0 != other.y_0 || this->x_1 != other.x_1 || this->y_1 != other.y_1;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCDoor<LTMCImpl>& p)
{
  return strm << "Door(" << p.entity_id << " \"" << p.getName() << "\" " << p.parent_map << " (" << p.x_0 << ", "
              << p.y_0 << ") (" << p.x_1 << ", " << p.y_1 << "))";
}

}  // namespace knowledge_rep
