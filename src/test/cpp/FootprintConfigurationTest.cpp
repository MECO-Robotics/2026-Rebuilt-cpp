#include "season/FootprintConfiguration.h"
#include <gtest/gtest.h>

TEST(FootprintConfiguration, RequiresExplicitGeometryLimitsAndValidObstacles) {
    const auto valid=R"({"version":1,"footprint":{"length_m":0.8,"width_m":0.7,"margin_m":0.05},
      "field":{"min_x":0,"min_y":0,"max_x":16,"max_y":8},
      "obstacles":[{"min_x":3,"min_y":2,"max_x":4,"max_y":3}],
      "planner":{"resolution_m":0.2,"expansion_budget":30000},
      "limits":{"max_speed_mps":1,"max_omega_radps":1,"max_acceleration_mps2":2,"max_alpha_radps2":3}})";
    const auto result=season::ParseFootprintConfiguration(valid);
    ASSERT_TRUE(result);
    EXPECT_NEAR(result->drive.footprint.length,.9,1e-9);
    EXPECT_NEAR(result->drive.footprint.width,.8,1e-9);
    ASSERT_EQ(result->obstacles.size(),1u);
    EXPECT_FALSE(base::SweptFootprintClear({2,2.5,0},{5,2.5,0},result->drive.footprint,result->drive.field,result->obstacles));
    EXPECT_FALSE(season::ParseFootprintConfiguration("{}"));
    EXPECT_FALSE(season::ParseFootprintConfiguration("not json"));
    auto invalid=std::string(valid);
    const std::string needle="\"margin_m\":0.05";
    invalid.replace(invalid.find(needle),needle.size(),"\"margin_m\":-0.1");
    EXPECT_FALSE(season::ParseFootprintConfiguration(invalid));
    invalid=valid;
    const std::string obstacle="{\"min_x\":3,\"min_y\":2,\"max_x\":4,\"max_y\":3}";
    invalid.replace(invalid.find(obstacle),obstacle.size(),"");
    EXPECT_FALSE(season::ParseFootprintConfiguration(invalid));
}
