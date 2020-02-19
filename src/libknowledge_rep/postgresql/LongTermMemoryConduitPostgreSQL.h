#pragma once
#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <pqxx/pqxx>
#include <string>
#include <vector>
#include <utility>

namespace knowledge_rep
{
static const char* table_names[] = { "entity_attributes_id", "entity_attributes_str",
                                     "entity_attributes_bool",     // NOLINT
                                     "entity_attributes_float" };  // NOLINT

class LongTermMemoryConduitPostgreSQL : public LongTermMemoryConduitInterface<LongTermMemoryConduitPostgreSQL>
{
  using EntityImpl = LTMCEntity<LongTermMemoryConduitPostgreSQL>;
  using InstanceImpl = LTMCInstance<LongTermMemoryConduitPostgreSQL>;
  using ConceptImpl = LTMCConcept<LongTermMemoryConduitPostgreSQL>;

  friend EntityImpl;
  friend InstanceImpl;
  friend ConceptImpl;

  friend class LongTermMemoryConduitInterface;

public:
  std::unique_ptr<pqxx::connection> conn;

  explicit LongTermMemoryConduitPostgreSQL(const std::string& db_name);

  // Move constructor
  LongTermMemoryConduitPostgreSQL(LongTermMemoryConduitPostgreSQL&& that) = default;

  ~LongTermMemoryConduitPostgreSQL();

  // Move assignment
  LongTermMemoryConduitPostgreSQL& operator=(LongTermMemoryConduitPostgreSQL&& that) noexcept = default;

  bool addNewAttribute(const std::string& name, const AttributeValueType type);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name,
                                                          const uint other_entity_id);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name, const bool bool_val);

  std::vector<EntityImpl> getEntitiesWithAttributeOfValue(const std::string& attribute_name,
                                                          const std::string& string_val);

  bool entityExists(uint id) const;

  bool deleteAttribute(std::string& name);

  bool attributeExists(const std::string& name) const;

  uint deleteAllEntities();

  uint deleteAllAttributes();

  std::vector<EntityImpl> getAllEntities();

  std::vector<ConceptImpl> getAllConcepts();

  std::vector<InstanceImpl> getAllInstances();

  std::vector<std::pair<std::string, AttributeValueType>> getAllAttributes() const;

  std::vector<EntityAttribute> getAllEntityAttributes();

  template <typename T>
  bool selectQuery(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    try
    {
      pqxx::work txn{ *conn };
      auto query_result = txn.exec(sql_query);
      for (const auto& row : query_result)
      {
        result.emplace_back(row["entity_id"].as<uint>(), row["attribute_name"].as<std::string>(),
                            row["attribute_name"].as<T>());
      }
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      return false;
    }
    return true;
  }

  bool selectQueryInt(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<int>(sql_query, result);
  }

  bool selectQueryFloat(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<float>(sql_query, result);
  }

  bool selectQueryString(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<std::string>(sql_query, result);
  }

  bool selectQueryBool(const std::string& sql_query, std::vector<EntityAttribute>& result) const
  {
    return selectQuery<bool>(sql_query, result);
  }

  //// CONVENIENCE
  LTMCConcept<LongTermMemoryConduitPostgreSQL> getConcept(const std::string& name);

  InstanceImpl getInstanceNamed(const std::string& name);

  InstanceImpl getRobot();

  EntityImpl addEntity();

  bool addEntity(int id);

  boost::optional<EntityImpl> getEntity(uint entity_id);

protected:
  bool deleteEntity(EntityImpl& entity);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const float float_val);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const bool bool_val);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const uint other_entity_id);

  bool addAttribute(EntityImpl& entity, const std::string& attribute_name, const std::string& string_val);

  int removeAttribute(EntityImpl& entity, const std::string& attribute_name);

  int removeAttributeOfValue(EntityImpl& entity, const std::string& attribute_name, const EntityImpl& other_entity);

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity) const;

  std::vector<EntityAttribute> getAttributes(const EntityImpl& entity, const std::string& attribute_name) const;

  bool isValid(const EntityImpl& entity) const;

  std::vector<ConceptImpl> getConcepts(const InstanceImpl& instance);
};

typedef LTMCEntity<LongTermMemoryConduitPostgreSQL> Entity;
typedef LTMCConcept<LongTermMemoryConduitPostgreSQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitPostgreSQL> Instance;
typedef LongTermMemoryConduitPostgreSQL LongTermMemoryConduit;
}  // namespace knowledge_rep
