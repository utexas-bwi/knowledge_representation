#pragma once

#include <iostream>
#include <string>
#include <map>
#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <typeindex>
#include <vector>
#include "EntityAttribute.h"

namespace knowledge_rep
{
template <typename EntLTMCImpl>
class LTMCEntity;

template <typename ConLTMCImpl>
class LTMCConcept;

template <typename InsLTMCImpl>
class LTMCInstance;

template <typename MapLTMCImpl>
class LTMCMap;

template <typename PointLTMCImpl>
class LTMCPoint;

template <typename PoseLTMCImpl>
class LTMCPose;

template <typename RegionLTMCImpl>
class LTMCRegion;

class EntityAttribute;

enum AttributeValueType;

/// This is an instance of the Curiously Recurring Template Pattern (CRTP), which
/// we're leveraging to enable multiple backend implementations while maintaining
/// a static interface with no runtime dispatch. You can read more about this pattern
/// here: https://www.fluentcpp.com/2017/05/12/curiously-recurring-template-pattern/
template <typename Impl>
class LongTermMemoryConduitInterface
{
  using LTMC = LongTermMemoryConduitInterface;

public:
  using EntityImpl = LTMCEntity<Impl>;
  using InstanceImpl = LTMCInstance<Impl>;
  using ConceptImpl = LTMCConcept<Impl>;
  using MapImpl = LTMCMap<Impl>;
  using PointImpl = LTMCPoint<Impl>;
  using PoseImpl = LTMCPose<Impl>;
  using RegionImpl = LTMCRegion<Impl>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;
  friend MapImpl;
  friend PointImpl;
  friend PoseImpl;
  friend RegionImpl;
  friend Impl;

  LongTermMemoryConduitInterface(LongTermMemoryConduitInterface&& that) noexcept = default;

  LongTermMemoryConduitInterface& operator=(LongTermMemoryConduitInterface&& that) noexcept = default;

  bool addAttribute(const std::string& name, const AttributeValueType type)
  {
    return static_cast<Impl*>(this)->addAttribute(name, type);
  };

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const uint other_entity_id)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, other_entity_id);
  };

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const bool bool_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, bool_val);
  }

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name,
                                                          const std::string& string_val)
  {
    return static_cast<Impl*>(this)->getEntitiesWithAttributeOfValue(attribute_name, string_val);
  }

  bool entityExists(uint id) const
  {
    return static_cast<const Impl*>(this)->entityExists(id);
  };

  bool deleteAttribute(const std::string& name)
  {
    return static_cast<Impl*>(this)->deleteAttribute(name);
  }

  bool attributeExists(const std::string& name) const
  {
    return static_cast<const Impl*>(this)->attributeExists(name);
  };

  uint deleteAllEntities()
  {
    return static_cast<Impl*>(this)->deleteAllEntities();
  }

  uint deleteAllAttributes()
  {
    return static_cast<Impl*>(this)->deleteAllAttributes();
  }

  std::vector<EntityImpl> getAllEntities()
  {
    return static_cast<Impl*>(this)->getAllEntities();
  }

  std::vector<ConceptImpl> getAllConcepts()
  {
    return static_cast<Impl*>(this)->getAllConcepts();
  };

  std::vector<InstanceImpl> getAllInstances()
  {
    return static_cast<Impl*>(this)->getAllInstances();
  }

  std::vector<std::pair<std::string, int>> getAllAttributes() const
  {
    return static_cast<const Impl*>(this)->getAllAttributes();
  }

  /// RAW QUERIES

  bool selectQueryInt(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryInt(sql_query, result);
  }

  bool selectQueryFloat(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryFloat(sql_query, result);
  }

  bool selectQueryString(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryString(sql_query, result);
  }

  bool selectQueryBool(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return static_cast<const Impl*>(this)->selectQueryBool(sql_query, result);
  }

  /// MAP
  MapImpl getMap(const std::string& name)
  {
    return static_cast<Impl*>(this)->getMap(name);
  }

  /// CONVENIENCE
  ConceptImpl getConcept(const std::string& name)
  {
    return static_cast<Impl*>(this)->getConcept(name);
  };

  InstanceImpl getInstanceNamed(const std::string& name)
  {
    return static_cast<Impl*>(this)->getInstanceNamed(name);
  };

  InstanceImpl getRobot()
  {
    return static_cast<Impl*>(this)->getRobot();
  };

  EntityImpl addEntity()
  {
    return static_cast<Impl*>(this)->addEntity();
  };

  bool addEntity(uint id)
  {
    return static_cast<Impl*>(this)->addEntity(id);
  };

  boost::optional<EntityImpl> getEntity(uint entity_id)
  {
    return static_cast<Impl*>(this)->getEntity(entity_id);
  };

protected:
  /// ENTITY BACKERS
  // These provide implementation for entity level operations. We want these to be centralized
  // with the rest of the database access code for ease of reimplementation in another backend,
  // but the users should see a nice API through the Entity class.
  bool deleteEntity(EntityImpl& entity)
  {
    return static_cast<Impl*>(this)->deleteEntity(entity);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const float float_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, float_val);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const bool bool_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, bool_val);
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const uint other_entity_id)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, static_cast<const uint>(other_entity_id));
  }

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const std::string& string_val)
  {
    return static_cast<Impl*>(this)->addAttribute(entity, attribute_name, string_val);
  }

  int removeAttribute(EntityImpl& entity, const std::string& attribute_name)
  {
    return static_cast<Impl*>(this)->removeAttribute(entity, attribute_name);
  }

  int removeAttributeOfValue(EntityImpl& entity, const std::string& attribute_name, const EntityImpl& other_entity)
  {
    return static_cast<Impl*>(this)->removeAttributeOfValue(entity, attribute_name, other_entity);
  }

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity) const
  {
    return static_cast<const Impl*>(this)->getAttributes(entity);
  }

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity, const std::string& attribute_name) const
  {
    return static_cast<const Impl*>(this)->getAttributes(entity, attribute_name);
  }

  bool isValid(const EntityImpl& entity) const
  {
    return static_cast<const Impl*>(this)->isValid(entity);
  }

  /// INSTANCE BACKERS

  std::vector<ConceptImpl> getConcepts(const InstanceImpl& instance)
  {
    return static_cast<Impl*>(this)->getConcepts(instance);
  }

  /// MAP BACKERS
  PointImpl addPoint(MapImpl& map, const std::string& name, double x, double y)
  {
    return static_cast<Impl*>(this)->addPoint(map, name, x, y);
  }

  PoseImpl addPose(MapImpl& map, const std::string& name, double x, double y, double theta)
  {
    return static_cast<Impl*>(this)->addPose(map, name, x, y, theta);
  }

  RegionImpl addRegion(MapImpl& map, const std::string& name, const std::vector<std::pair<double, double>>& points)
  {
    return static_cast<Impl*>(this)->addRegion(map, name, points);
  }

  boost::optional<PointImpl> getPoint(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getPoint(map, name);
  }

  boost::optional<PoseImpl> getPose(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getPose(map, name);
  }

  boost::optional<RegionImpl> getRegion(MapImpl& map, const std::string& name)
  {
    return static_cast<Impl*>(this)->getRegion(map, name);
  }
  std::vector<PointImpl> getAllPoints(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllPoints(map);
  }

  std::vector<PoseImpl> getAllPoses(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllPoses(map);
  }

  std::vector<RegionImpl> getAllRegions(MapImpl& map)
  {
    return static_cast<Impl*>(this)->getAllRegions(map);
  }

  /// GEOMETRY BACKERS

private:
  // We make the constructor private to make sure people can't build this interface type directly
  LongTermMemoryConduitInterface() = default;
};

}  // namespace knowledge_rep
