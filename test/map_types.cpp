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

TEST_F(MapTest, GetMap)
{
  ASSERT_EQ(ltmc.getMap("test map"), map);
  ASSERT_TRUE(map.hasConcept(ltmc.getConcept("map")));
  ASSERT_NE(ltmc.getMap("another map"), map);
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
