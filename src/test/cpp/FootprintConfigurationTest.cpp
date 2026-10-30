#include "base/FootprintConfiguration.h"
#include <gtest/gtest.h>

TEST(FootprintConfiguration, RequiresExplicitGeometryLimitsAndValidObstacles) {
    const auto valid=R"({"version":1,"footprint":{"length_m":0.8,"width_m":0.7,"margin_m":0.05},
      "field":{"min_x":0,"min_y":0,"max_x":16,"max_y":8},
      "obstacles":[{"min_x":3,"min_y":2,"max_x":4,"max_y":3}],
      "planner":{"resolution_m":0.2,"expansion_budget":30000},
      "limits":{"max_speed_mps":1,"max_omega_radps":1,"max_acceleration_mps2":2,"max_alpha_radps2":3}})";
    const auto result=base::ParseFootprintConfiguration(valid);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->drive.footprint.length,.9,1e-9);
    EXPECT_NEAR(result->drive.footprint.width,.8,1e-9);
    EXPECT_FALSE(result->drive.follow.terrain);
    ASSERT_EQ(result->obstacles.size(),1u);
    EXPECT_FALSE(base::SweptFootprintClear({2,2.5,0},{5,2.5,0},result->drive.footprint,result->drive.field,result->obstacles));
    EXPECT_FALSE(base::ParseFootprintConfiguration("{}"));
    EXPECT_FALSE(base::ParseFootprintConfiguration("not json"));
    auto invalid=std::string(valid);
    const std::string needle="\"margin_m\":0.05";
    invalid.replace(invalid.find(needle),needle.size(),"\"margin_m\":-0.1");
    EXPECT_FALSE(base::ParseFootprintConfiguration(invalid));
    invalid=valid;
    const std::string obstacle="{\"min_x\":3,\"min_y\":2,\"max_x\":4,\"max_y\":3}";
    invalid.replace(invalid.find(obstacle),obstacle.size(),"");
    EXPECT_FALSE(base::ParseFootprintConfiguration(invalid));
}

TEST(FootprintConfiguration, OptionalTerrainRequiresValidProfileAndSuppliesDirectionalGrade) {
    const std::string json=R"({"version":1,"footprint":{"length_m":0.8,"width_m":0.7,"margin_m":0.05},
      "field":{"min_x":0,"min_y":0,"max_x":16,"max_y":8},
      "obstacles":[{"min_x":8,"min_y":6,"max_x":9,"max_y":7}],
      "planner":{"resolution_m":0.2,"expansion_budget":30000},
      "limits":{"max_speed_mps":1,"max_omega_radps":1,"max_acceleration_mps2":2,"max_alpha_radps2":3},
      "terrain":{"lateral_acceleration_mps2":2,"steering_rate_radps":1,
        "stall_timeout_s":3,"stall_progress_m":0.03,"stall_request_mps":0.01,
        "regions":[{"bounds":{"min_x":1,"min_y":1,"max_x":2,"max_y":2},"grade_x":0.1,"grade_y":0,"rolling_resistance":0.02}]}})";
    const auto result=base::ParseFootprintConfiguration(json);
    ASSERT_TRUE(result);
    ASSERT_TRUE(result->drive.follow.terrain);
    ASSERT_TRUE(result->drive.follow.terrainBoundaries);
    EXPECT_DOUBLE_EQ(result->drive.follow.translationStallTimeout,3);
    EXPECT_EQ(result->drive.follow.terrainBoundaries({0,1.5,0},{3,1.5,0}).size(),2u);
    EXPECT_NEAR(result->drive.follow.terrain({1,1.5,0},{2,1.5,0}).grade,.1,1e-9);
    EXPECT_NEAR(result->drive.follow.terrain({2,1.5,0},{1,1.5,0}).grade,-.1,1e-9);
    auto invalid=json;
    const std::string needle="\"rolling_resistance\":0.02";
    invalid.replace(invalid.find(needle),needle.size(),"\"rolling_resistance\":-0.02");
    EXPECT_FALSE(base::ParseFootprintConfiguration(invalid));
    invalid=json;
    const std::string timeout="\"stall_timeout_s\":3,";
    invalid.erase(invalid.find(timeout),timeout.size());
    EXPECT_FALSE(base::ParseFootprintConfiguration(invalid));
}
