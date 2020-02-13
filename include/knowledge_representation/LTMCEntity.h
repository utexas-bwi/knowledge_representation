#pragma once
#include <string>
#include <vector>
#include <knowledge_representation/LongTermMemoryConduitInterface.h>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCEntity
{
public:
  using EntityId = uint;
  EntityId entity_id;

  LTMCEntity(EntityId entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc) : entity_id(entity_id), ltmc(ltmc)
  {
  }

  bool addAttribute(const std::string& attribute_name, const std::string& string_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, string_val);
  };

  bool addAttribute(const std::string& attribute_name, const char* string_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, std::string(string_val));
  };

  bool addAttribute(const std::string& attribute_name, float float_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, float_val);
  };

  bool addAttribute(const std::string& attribute_name, bool bool_val)
  {
    return ltmc.get().addAttribute(*this, attribute_name, bool_val);
  };

  bool addAttribute(const std::string& attribute_name, EntityId other_entity_id)
  {
    return ltmc.get().addAttribute(*this, attribute_name, other_entity_id);
  };

  bool addAttribute(const std::string& attribute_name, const LTMCEntity& other_entity)
  {
    return ltmc.get().addAttribute(*this, attribute_name, other_entity.entity_id);
  };

  int removeAttribute(const std::string& attribute_name)
  {
    return ltmc.get().removeAttribute(*this, attribute_name);
  };

  int removeAttributeOfValue(const std::string& attribute_name, const LTMCEntity& other_entity)
  {
    return ltmc.get().removeAttributeOfValue(*this, attribute_name, other_entity);
  };

  std::vector<EntityAttribute> getAttributes() const
  {
    return ltmc.get().getAttributes(*this);
  };

  std::vector<EntityAttribute> getAttributes(const std::string& attribute_name) const
  {
    return ltmc.get().getAttributes(*this, attribute_name);
  };

  boost::optional<std::string> getName()
  {
    // There should only be one
    auto name_attrs = getAttributes("name");
    if (!name_attrs.empty())
    {
      return boost::get<std::string>(name_attrs[0].value);
    }
    else
    {
      return {};
    }
  };

  bool deleteEntity()
  {
    return ltmc.get().deleteEntity(*this);
  };

  bool isValid() const
  {
    return ltmc.get().isValid(*this);
  };

  bool operator==(const LTMCEntity& other) const
  {
    return this->entity_id == other.entity_id;
  }

  LTMCEntity& operator=(const LTMCEntity& that)
  {
    this->entity_id = that.entity_id;
    this->ltmc = that.ltmc;
  }

  std::vector<EntityAttribute> operator[](const std::string& attr_name) const
  {
    return getAttributes(attr_name);
  };

protected:
  std::reference_wrapper<LongTermMemoryConduitInterface<LTMCImpl>> ltmc;
};

}  // namespace knowledge_rep
