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

// Requirements
// * Be able to staticly select which database backend
// * No external dependency on which backend is available
// Options
// Non-requirements:
// * ABI compatibility. We will continue to assume source builds
//
// Solution: Static polymorphism w/ CRTP switched via a compile flag (?)
// * If we had used a pure virtual interface, we'd have to go through a pointer to the objects (LTMC, Entity, etc)
//   and we'd still have a vtable lookup
// The outside still sees
namespace knowledge_rep
{
template <typename ConLTMCImpl>
class LTMCConcept;

template <typename InsLTMCImpl>
class LTMCInstance;

template <typename EntLTMCImpl>
class LTMCEntity;

class EntityAttribute;

enum AttributeValueType;

template <typename Impl>
class LongTermMemoryConduitInterface
{
  using LTMC = LongTermMemoryConduitInterface;

public:
  using EntityImpl = LTMCEntity<Impl>;
  using InstanceImpl = LTMCInstance<Impl>;
  using ConceptImpl = LTMCConcept<Impl>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;
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

  std::vector<EntityAttribute> getAllEntityAttributes() const
  {
    return static_cast<const Impl*>(this)->getAllEntityAttributes();
  }

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

  //// CONVENIENCE
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

  bool addEntity(int id)
  {
    return static_cast<Impl*>(this)->addEntity(id);
  };

  boost::optional<EntityImpl> getEntity(int entity_id)
  {
    return static_cast<Impl*>(this)->getEntity(entity_id);
  };

protected:
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

  std::vector<ConceptImpl> getConcepts(const InstanceImpl& instance)
  {
    return static_cast<Impl*>(this)->getConcepts(instance);
  }

private:
  // We make the constructor private to make sure people can't build this interface type directly
  LongTermMemoryConduitInterface() = default;
};

}  // namespace knowledge_rep
