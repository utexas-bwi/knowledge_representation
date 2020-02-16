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

class EntityTest : public ::testing::Test
{
protected:
  EntityTest()
    : ltmc(knowledge_rep::getDefaultLTMC())
    , entity(ltmc.addEntity())
    , concept(ltmc.getConcept("test concept"))
    , parent_concept(ltmc.getConcept("parent concept"))
    , instance(ltmc.addEntity().entity_id, ltmc)
  {
  }

  ~EntityTest()
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

class MapTest : public ::testing::Test
{
protected:
  MapTest() : ltmc(knowledge_rep::getDefaultLTMC()), map(ltmc.getMap("test map"))
  {
  }

  void TearDown() override
  {
    map.deleteEntity();
    ltmc.deleteAllEntities();
    ltmc.deleteAllAttributes();
  }

  knowledge_rep::LongTermMemoryConduit ltmc;
  knowledge_rep::Map map;
};

// Declare a test
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
  EXPECT_EQ(ltmc.getAllAttributes().size(), 17);
}

TEST_F(LTMCTest, GetConceptWorks)
{
  // Get concept returns the one true concept id
  Concept soda = ltmc.getConcept("soda");
  EXPECT_EQ(soda.entity_id, ltmc.getConcept("soda").entity_id);
}

TEST_F(LTMCTest, SQLQueryStrWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_str", query_result);
  EXPECT_EQ(query_result.size(), 1);
}

TEST_F(LTMCTest, SQLQueryIdWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_id", query_result);
  EXPECT_EQ(query_result.size(), 1);
}

TEST_F(LTMCTest, SQLQueryBoolWorks)
{
  vector<EntityAttribute> query_result;
  ltmc.selectQueryString("SELECT * FROM entity_attributes_bool", query_result);
  EXPECT_EQ(query_result.size(), 1);
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

TEST_F(EntityTest, EntityEqualityWorks)
{
  auto new_entity = ltmc.addEntity();
  ASSERT_NE(entity, new_entity);

  auto reconstructed = Entity(new_entity.entity_id, ltmc);
  EXPECT_EQ(new_entity, reconstructed);
}

TEST_F(EntityTest, DeleteEntityWorks)
{
  concept.deleteEntity();
  EXPECT_FALSE(concept.isValid());
  EXPECT_EQ(0, concept.getAttributes().size());
}

TEST_F(EntityTest, RemoveInstancesWorks)
{
  instance.makeInstanceOf(concept);
  concept.removeInstances();
  EXPECT_FALSE(instance.isValid());
  EXPECT_EQ(0, concept.getInstances().size());
}

TEST_F(EntityTest, GetConceptsWorks)
{
  instance.makeInstanceOf(concept);
  concept.addAttribute("is_a", parent_concept);
  EXPECT_EQ(2, instance.getConcepts().size());
}

TEST_F(LTMCTest, RecursiveRemoveWorks)
{
  Concept parent = ltmc.getConcept("parent concept");
  Concept child = ltmc.getConcept("child concept");
  child.addAttribute("is_a", parent);
  knowledge_rep::Entity object = ltmc.addEntity();
  object.addAttribute("instance_of", child);
  parent.removeInstances();
  EXPECT_FALSE(object.isValid());
}

TEST_F(EntityTest, AddEntityWorks)
{
  ASSERT_TRUE(entity.isValid());
}

TEST_F(EntityTest, CreateInstanceWorks)
{
  auto instance = concept.createInstance();
  EXPECT_TRUE(instance.isValid());
  auto named_instance = concept.createInstance("named");
  ASSERT_TRUE(named_instance.is_initialized());
  EXPECT_TRUE(named_instance->isValid());
}

TEST_F(EntityTest, StringAttributeWorks)
{
  entity.addAttribute("is_concept", "test");
  auto attrs = entity.getAttributes("is_concept");
  ASSERT_EQ(typeid(string), attrs.at(0).value.type());
  EXPECT_EQ("test", boost::get<string>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_concept"));
}

TEST_F(EntityTest, IntAttributeWorks)
{
  entity.addAttribute("is_concept", 1u);
  auto attrs = entity.getAttributes("is_concept");
  ASSERT_EQ(typeid(int), attrs.at(0).value.type());
  EXPECT_EQ(1, boost::get<int>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_concept"));
}

TEST_F(EntityTest, FloatAttributeWorks)
{
  entity.addAttribute("is_concept", 1.f);
  auto attrs = entity.getAttributes("is_concept");
  ASSERT_EQ(typeid(float), attrs.at(0).value.type());
  EXPECT_EQ(1, boost::get<float>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_concept"));
}

TEST_F(EntityTest, BoolAttributeWorks)
{
  entity.addAttribute("is_concept", true);
  auto attrs = entity.getAttributes("is_concept");
  ASSERT_EQ(typeid(bool), attrs.at(0).value.type());
  EXPECT_TRUE(boost::get<bool>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_concept"));

  entity.addAttribute("is_concept", false);
  attrs = entity.getAttributes("is_concept");
  ASSERT_EQ(typeid(bool), attrs.at(0).value.type());
  EXPECT_FALSE(boost::get<bool>(attrs.at(0).value));
  EXPECT_EQ(1, entity.removeAttribute("is_concept"));
}

TEST_F(EntityTest, CantRemoveEntityAttributeTwice)
{
  entity.addAttribute("is_concept", true);
  ASSERT_TRUE(entity.removeAttribute("is_concept"));
  ASSERT_FALSE(entity.removeAttribute("is_concept"));
}

TEST_F(MapTest, GetMap)
{
  ASSERT_EQ(ltmc.getMap("test map"), map);
  ASSERT_TRUE(map.hasConcept(ltmc.getConcept("map")));
}

TEST_F(MapTest, AddPointWorks)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  ASSERT_TRUE(point.hasConcept(ltmc.getConcept("point")));
}

TEST_F(MapTest, DoubleAddPointFails)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  // TODO(nickswalker): Making this throw depends on sorting out whether name attribute is allowed to have duplicates
  // EXPECT_ANY_THROW(map.addPoint("test point", 2.0, 3.0));
}

TEST_F(MapTest, PointEqualityWorks)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  auto same_point = Point(point.entity_id, "test point", 1.0, 2.0, map, ltmc);
  ASSERT_EQ(same_point, point);
}

TEST_F(MapTest, GetPointWorks)
{
  auto added = map.addPoint("test point", 1.0, 2.0);
  auto retrieved = map.getPoint("test point");
  ASSERT_EQ(retrieved, added);
}

TEST_F(MapTest, AddPoseWorks)
{
  auto pose = map.addPose("test point", 1.0, 2.0, 3.14);
  ASSERT_TRUE(pose.hasConcept(ltmc.getConcept("pose")));
}

TEST_F(MapTest, GetPoseWorks)
{
  auto added = map.addPose("test pose", 1.0, 2.0, 3.14);
  auto retrieved = map.getPose("test pose");
  ASSERT_EQ(retrieved, added);
}

/*TEST_F(MapTest, AddRegionWorks)
{
  auto region = map.addRegion("test region", {{1.0, 2.0}, {3.0, 4.0}});
  ASSERT_TRUE(region.hasConcept(ltmc.getConcept("region")));
}

TEST_F(MapTest, GetRegionWorks)
{
  auto added = map.addRegion("test region", {{1.0, 2.0}, {3.0, 4.0}});
  auto retrieved = map.getRegion("test region");
  ASSERT_EQ(retrieved, added);
}*/

// Run all the tests
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
