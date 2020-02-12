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
class LTMCConcept : public LTMCEntity<LTMCImpl>
{
  using EntityImpl = LTMCEntity<LTMCImpl>;
  using InstanceImpl = LTMCInstance<LTMCImpl>;
  std::string name;

public:
  LTMCConcept(uint entity_id, std::string name, LongTermMemoryConduitInterface<LTMCImpl>& ltmc)
    : name(std::move(name)), EntityImpl(entity_id, ltmc)
  {
  }

  LTMCConcept(uint entity_id, LongTermMemoryConduitInterface<LTMCImpl>& ltmc) : EntityImpl(entity_id, ltmc)
  {
  }

  std::string getName()
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
    // Shouldn't be an unnamed concept
    assert(false);
  }

  std::vector<InstanceImpl> getInstances() const
  {
    auto entities = this->ltmc.get().getEntitiesWithAttributeOfValue(
        "instance_of", this->entity_id);
    std::vector<InstanceImpl> instances;
    transform(entities.begin(), entities.end(), std::back_inserter(instances),
              [this](const EntityImpl& entity)
              {
      return InstanceImpl(entity.entity_id, this->ltmc.get());
    });
    return instances;
  }

  /**
   * @brief Recursively remove instances of the concept
   * An instance of a given concept "C" is transitively an instance of any concept that that defines an "is_a" relation
   * with "C". For example, if the concept of apple "is_a" concept of fruit, then removing instances of fruit will
   * remove all instances of apple.
   * @return the number of instances removed
   */
  int removeInstances()
  {
    std::vector<EntityImpl> child_concepts =
        this->ltmc.get().getEntitiesWithAttributeOfValue("is_a",
                                                         this->entity_id);
    std::vector<EntityImpl> instances_of_concept =
        this->ltmc.get().getEntitiesWithAttributeOfValue("instance_of",
                                                         this->entity_id);

    int total_removed = 0;
    for (const auto& child : child_concepts)
    {
      total_removed +=
          LTMCConcept(child.entity_id, this->ltmc).removeInstances();
    }
    for (auto& instance : instances_of_concept)
    {
      instance.deleteEntity();
    }

    total_removed += instances_of_concept.size();
    return total_removed;
  }

  InstanceImpl createInstance()
  {
    auto instance = this->ltmc.get().addEntity();
    instance.addAttribute("instance_of", this->entity_id);
    return { instance.entity_id, this->ltmc.get() };
  }

  /**
   * @brief Create an instance of the concept and give it a particular name
   * @param name the name to give the instance
   * @return the created instance, or empty if the instance cannot be created (as when the name is taken)
   */
  boost::optional<InstanceImpl> createInstance(const std::string& name)
  {
    auto instance = InstanceImpl(this->ltmc.get().addEntity().entity_id, this->ltmc);
    instance.addAttribute("instance_of", this->entity_id);
    bool added_name = instance.addAttribute("name", name);
    if (!added_name)
    {
      return {};
    }
    return instance;
  }

  /**
   * @brief Get all concepts that are direct children of this concept
   *
   * @return all direct childern
   */
  std::vector<LTMCConcept> getChildren() const
  {
    // Is a should only apply to concepts
    auto entities = this->ltmc.get().getEntitiesWithAttributeOfValue(
        "is_a", this->entity_id);
    std::vector<LTMCConcept> as_concept{};
    std::transform(entities.begin(), entities.end(), std::back_inserter(as_concept),
                   [this](const EntityImpl& entity) { return LTMCConcept<LTMCImpl>(entity.entity_id, this->ltmc); });
    return as_concept;
  }

  /**
   * @brief Removes all references to a concept
   * References include all entity-attributes that refer to this concept. But note that this method doesn't
   * delete the entities that are the subjects of the references. This means that all instances of this
   * concept will continue to exist, but they will no longer be identified as instances of this concept.
   * @param concept_name
   * @return
   */
  bool removeReferences()
  {
    // Rely on the schema to clear out the childern via cascading delete
    this->deleteEntity();
    {
      // Recreate it with the same ID
      auto new_entity = this->ltmc.get().getEntity(this->entity_id);
      if (!new_entity)
      {
        return false;
      }
    }
    // The current entity should still be valid (because we recreated it), so make it a concept again
    this->addAttribute("is_concept", true);
    return true;
  }
};
}  // namespace knowledge_rep
