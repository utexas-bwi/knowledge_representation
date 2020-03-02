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
  EXPECT_EQ(map, ltmc.getMap("test map"));
  EXPECT_TRUE(map.hasConcept(ltmc.getConcept("map")));
  EXPECT_NE(map, ltmc.getMap("another map"));
}

TEST_F(MapTest, AddPointWorks)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  EXPECT_EQ(1.0, point.x);
  EXPECT_EQ(2.0, point.y);
  EXPECT_EQ("test point", point.getName().get());
  EXPECT_TRUE(point.hasConcept(ltmc.getConcept("point")));
  auto second_point = map.addPoint("another point", 1.0, 2.0);
  EXPECT_EQ("another point", second_point.getName().get());
}

TEST_F(MapTest, GetAllPointsWorks)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  EXPECT_EQ(1, map.getAllPoints().size());
  point.deleteEntity();
  EXPECT_EQ(0, map.getAllPoints().size());
}

TEST_F(MapTest, DoubleAddPointFails)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  EXPECT_ANY_THROW(map.addPoint("test point", 2.0, 3.0));
}

TEST_F(MapTest, PointEqualityWorks)
{
  auto point = map.addPoint("test point", 1.0, 2.0);
  auto same_point = Point(point.entity_id, "test point", 1.0, 2.0, map, ltmc);
  EXPECT_EQ(point, same_point);
}

TEST_F(MapTest, GetPointWorks)
{
  auto added = map.addPoint("test point", 1.0, 2.0);
  auto retrieved = map.getPoint("test point");
  EXPECT_EQ(added, retrieved);
}

TEST_F(MapTest, AddPoseWorks)
{
  auto pose = map.addPose("test point", 1.0, 2.0, 3.14);
  EXPECT_TRUE(pose.hasConcept(ltmc.getConcept("pose")));
}

TEST_F(MapTest, GetPoseWorks)
{
  auto added = map.addPose("test pose", 1.0, 2.0, 3.14);
  auto retrieved = map.getPose("test pose");
  EXPECT_EQ(added, retrieved);
}

TEST_F(MapTest, GetAllPosesWorks)
{
  auto pose = map.addPose("test pose", 1.0, 2.0, 3.14);
  EXPECT_EQ(1, map.getAllPoses().size());
  pose.deleteEntity();
  EXPECT_EQ(0, map.getAllPoses().size());
}

/*TEST_F(MapTest, AddRegionWorks)
{
  auto region = map.addRegion("test region", {{1.0, 2.0}, {3.0, 4.0}});
  EXPECT_TRUE(region.hasConcept(ltmc.getConcept("region")));
}

TEST_F(MapTest, GetRegionWorks)
{
  auto added = map.addRegion("test region", {{1.0, 2.0}, {3.0, 4.0}});
  auto retrieved = map.getRegion("test region");
  EXPECT_EQ(retrieved, added);
}*/
