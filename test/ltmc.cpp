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

class LTMCTest : public ::testing::Test
{
protected:
  LTMCTest() : ltmc(knowledge_rep::getDefaultLTMC())
  {
  }

  void SetUp() override
  {
    ltmc.deleteAllAttributes();
    ltmc.deleteAllEntities();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
};

TEST_F(LTMCTest, InitialConfigurationIsValid)
{
  // Robot should exist
  EXPECT_TRUE(ltmc.entityExists(1));
  Instance robot = { ltmc.getEntity(1).get().entity_id, ltmc };
  auto concepts = robot.getConcepts();
  EXPECT_EQ(concepts.size(), 1);
  EXPECT_EQ(concepts[0].getName(), "robot");
  EXPECT_EQ(ltmc.getAllEntities().size(), 2);
  EXPECT_EQ(ltmc.getAllConcepts().size(), 1);
  EXPECT_EQ(ltmc.getAllInstances().size(), 1);
  EXPECT_EQ(ltmc.getAllAttributes().size(), 15);
}

TEST_F(LTMCTest, GetConceptWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  EXPECT_EQ(soda.entity_id, ltmc.getConcept("soda").entity_id);
}

TEST_F(LTMCTest, GetMapWorks)
{
  // Get concept returns the one true concept id
  auto fresh_map = ltmc.getMap("test map");
  auto second_map = ltmc.getMap("second test map");
  auto retrieved_map = ltmc.getMap("test map");
  EXPECT_EQ(fresh_map, retrieved_map);
}

TEST_F(LTMCTest, SQLQueryStrWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_str", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryIdWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_id", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryBoolWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_bool", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, SQLQueryFloatWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryFloat("SELECT * FROM entity_attributes_float", query_result);
  EXPECT_EQ(query_result.size(), 0);
}

TEST_F(LTMCTest, ObjectAndConceptNameSpacesAreSeparate)
{
  Concept pitcher_con = ltmc.getConcept("soylent pitcher");
  knowledge_rep::Entity pitcher = ltmc.getInstanceNamed("soylent pitcher");
  EXPECT_NE(ltmc.getInstanceNamed("soylent pitcher").entity_id, pitcher_con.entity_id);
  // Both should still be valid
  EXPECT_TRUE(pitcher_con.isValid());
  EXPECT_TRUE(pitcher.isValid());
  // Named entity returns the only entity by that name
  EXPECT_EQ(pitcher, ltmc.getInstanceNamed("soylent pitcher"));
}

TEST_F(LTMCTest, CanOnlyAddAttributeOnce)
{
  knowledge_rep::Entity drinkable = ltmc.getConcept("drinkable");
  knowledge_rep::Entity can = ltmc.addEntity();

  // Adding a second time should fail
  EXPECT_TRUE(can.addAttribute("is_a", drinkable));
  EXPECT_FALSE(can.addAttribute("is_a", drinkable));
}

TEST_F(LTMCTest, OnlyValidAttributeNamesAllowed)
{
  knowledge_rep::Entity can = ltmc.addEntity();
  EXPECT_FALSE(can.addAttribute("not a real attribute", true));
}

TEST_F(LTMCTest, GetAllEntitiesWorks)
{
  int start_num = ltmc.getAllEntities().size();
  ltmc.addEntity();
  EXPECT_EQ(ltmc.getAllEntities().size(), start_num + 1);
}

TEST_F(LTMCTest, GetAllConceptsWorks)
{
  int start_num = ltmc.getAllConcepts().size();
  ltmc.getConcept("neverseenbeforecon");
  EXPECT_EQ(ltmc.getAllConcepts().size(), start_num + 1);
}

TEST_F(LTMCTest, GetAllInstancesWorks)
{
  int start_num = ltmc.getAllInstances().size();
  ltmc.getInstanceNamed("neverseenbeforeinst");
  EXPECT_EQ(ltmc.getAllInstances().size(), start_num + 1);
}

TEST_F(LTMCTest, InstanceConceptSumToTotalEntities)
{
  knowledge_rep::Entity can = ltmc.addEntity();
  EXPECT_EQ(ltmc.getAllConcepts().size() + ltmc.getAllInstances().size(), ltmc.getAllEntities().size());
}

TEST_F(LTMCTest, GetAllAttributesWorks)
{
  int start_num = ltmc.getAllAttributes().size();
  ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str);
  EXPECT_EQ(ltmc.getAllAttributes().size(), start_num + 1);
}

TEST_F(LTMCTest, AddNewAttributeWorks)
{
  EXPECT_TRUE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str));
  // Second add should fail
  EXPECT_FALSE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Str));
  // Even if it's on another type
  EXPECT_FALSE(ltmc.addNewAttribute("neverseenbeforeattr", AttributeValueType::Bool));
}

TEST_F(LTMCTest, RecursiveRemoveWorks)
{
  Concept parent = ltmc.getConcept("parent concept");
  Concept child = ltmc.getConcept("child concept");
  child.addAttribute("is_a", parent);
  auto instance = child.createInstance();
  ASSERT_EQ(1, parent.removeInstancesRecursive());
  EXPECT_FALSE(instance.isValid());
}

// Run all the tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
