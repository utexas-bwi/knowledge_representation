#pragma once

#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCInstance.h>
#include <utility>
#include <vector>
#include <string>
#include <algorithm>

namespace knowledge_rep
{
/**
 * @brief An instance of the Map concept, which represents a single 2D frame of reference
 *
 * Unlike instances of other concepts, maps **must** be uniquely named
 * @tparam LTMCImpl
 */
template <typename LTMCImpl>
class LTMCMap : public LTMCInstance<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  using PointImpl = LTMCPoint<LTMCImpl>;
  using PoseImpl = LTMCPose<LTMCImpl>;
  using RegionImpl = LTMCRegion<LTMCImpl>;

  uint map_id;

public:
  LTMCMap(uint entity_id, uint map_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : map_id(map_id), InstanceImpl(entity_id, name, ltmc)
  {
  }

  /**
   * @brief Renames a map
   *
   * Due to special uniqueness constraints on map names, you need to use
   * this method instead of manually adjusting the name attribute.
   * @param new_name
   * @return true if the rename succeeded
   */
  bool rename(const std::string& new_name)
  {
    bool rename_succeeded = this->ltmc.get().renameMap(*this, new_name);
    if (rename_succeeded)
    {
      this->name = new_name;
    }
    return rename_succeeded;
  }

  std::string getName() const
  {
    return this->name;
  }

  /**
   * @brief Get's the map's id (**not** its entity ID)
   *
   * Map's are indexed by their own space of IDs. This prevents edge cases where a plain
   * entity ID could be coerced to be an instance of a map, without respecting unique constraints for maps.
   * @return
   */
  uint getId()
  {
    return this->map_id;
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
  bool operator==(const LTMCMap& other) const
  {
    return this->entity_id == other.entity_id && this->map_id == other.map_id;
  }
  bool operator!=(const LTMCMap& other) const
  {
    return this->entity_id != other.entity_id || this->map_id != other.map_id;
  }
};

template <typename LTMCImpl>
std::ostream& operator<<(std::ostream& strm, const LTMCMap<LTMCImpl>& m)
{
  return strm << "Map(" << m.entity_id << " \"" << m.getName() << "\")";
}
}  // namespace knowledge_rep
