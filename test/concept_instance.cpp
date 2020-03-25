#include <knowledge_representation/LongTermMemoryConduit.h>
#include <string>
#include <vector>
#include <knowledge_representation/convenience.h>

#include <gtest/gtest.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCMap.h>
#include <knowledge_representation/LTMCPoint.h>
#include <knowledge_representation/LTMCPose.h>
// #include <knowledge_representation/LTMCRegion.h>

using knowledge_rep::AttributeValueType;
using knowledge_rep::Concept;
using knowledge_rep::Entity;
using knowledge_rep::EntityAttribute;
using knowledge_rep::Instance;
using knowledge_rep::Map;
using knowledge_rep::Point;
using knowledge_rep::Pose;
using knowledge_rep::Region;
using std::cout;
using std::endl;
using std::string;
using std::vector;

class ConceptInstanceTest : public ::testing::Test
{
protected:
  ConceptInstanceTest()
    : ltmc(knowledge_rep::getDefaultLTMC())
    , entity(ltmc.addEntity())
    , concept(ltmc.getConcept("test concept"))
    , parent_concept(ltmc.getConcept("parent concept"))
    , instance(ltmc.addEntity().entity_id, ltmc)
  {
  }

  ~ConceptInstanceTest()
  {
    entity.deleteEntity();
    parent_concept.deleteEntity();
    concept.deleteEntity();
    instance.deleteEntity();
    ltmc.deleteAllEntities();
    ltmc.deleteAllAttributes();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
  knowledge_rep::Entity entity;
  knowledge_rep::Concept concept;
  knowledge_rep::Concept parent_concept;
  knowledge_rep::Instance instance;
};

TEST_F(ConceptInstanceTest, RemoveInstancesRecursiveWorks)
{
  instance.makeInstanceOf(concept);
  Concept parent_concept = ltmc.getConcept("parent concept");
  concept.addAttribute("is_a", parent_concept);
  EXPECT_EQ(0, parent_concept.removeInstances());
  EXPECT_EQ(1, parent_concept.removeInstancesRecursive());
  EXPECT_FALSE(instance.isValid());
  EXPECT_EQ(0, concept.getInstances().size());
}

TEST_F(ConceptInstanceTest, GetConceptsWorks)
{
  instance.makeInstanceOf(concept);
  concept.addAttribute("is_a", parent_concept);
  EXPECT_EQ(1, instance.getConcepts().size());
}

TEST_F(ConceptInstanceTest, GetConceptsRecursiveWorks)
{
  instance.makeInstanceOf(concept);
  concept.addAttribute("is_a", parent_concept);
  EXPECT_EQ(2, instance.getConceptsRecursive().size());
}

TEST_F(ConceptInstanceTest, CreateInstanceWorks)
{
  auto instance = concept.createInstance();
  EXPECT_TRUE(instance.isValid());
  auto named_instance = concept.createInstance("named");
  ASSERT_TRUE(named_instance.is_initialized());
  EXPECT_TRUE(named_instance->isValid());
}

TEST_F(ConceptInstanceTest, MakeInstanceOfWorks)
{
  EXPECT_EQ(0, instance.getConcepts().size());
  EXPECT_TRUE(instance.makeInstanceOf(concept));
  EXPECT_EQ(1, instance.getConcepts().size());
}
