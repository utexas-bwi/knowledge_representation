#pragma once
#include <utility>
#include <vector>
#include <string>

#include <knowledge_representation/LTMCEntity.h>

namespace knowledge_rep
{
template <typename LTMCImpl>
class LTMCInstance : public LTMCEntity<LTMCImpl>
{
  template <typename ConLTMCImpl>
  class Concept;
  std::string name;

public:
  LTMCInstance(uint entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : name(std::move(name)), LTMCEntity<LTMCImpl>(entity_id, ltmc)
  {
  }

  LTMCInstance(uint entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc) : LTMCEntity<LTMCImpl>(entity_id, ltmc)
  {
  }

  boost::optional<std::string> getName()
  {
    if (!name.empty())
    {
      return name;
    }
    // There should only be one
    auto name_attrs = this->ltmc.get().getAttributes(*this, "name");
    if (!name_attrs.empty())
    {
      name = boost::get<std::string>(name_attrs[0].value);
      return name;
    }
    return {};
  };

  bool makeInstanceOf(const LTMCConcept<LTMCImpl>& concept)
  {
    return this->addAttribute("instance_of", concept.entity_id);
  }

  std::vector<LTMCConcept<LTMCImpl>> getConcepts() const
  {
    return this->ltmc.get().getConcepts(*this);
  }
  /**
   * @brief whether instance descends from the concept
   * @return
   */
  bool hasConcept(const LTMCConcept<LTMCImpl>& concept)
  {
    auto concepts = this->getConcepts();
    return std::find(concepts.begin(), concepts.end(), concept) != concepts.end();
  }
};

}  // namespace knowledge_rep
