#include "season/OffensiveAutoDrive.h"
#include <gtest/gtest.h>
using namespace units::literals;

TEST(OffensiveAutoDrive, HubShotRequiresKnownActiveHubAndSettledArrival) {
  const frc::Pose2d target{3_m,1_m,frc::Rotation2d{0_rad}};
  EXPECT_FALSE(season::OffensiveHubShotAllowed(std::nullopt,target,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(false,target,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,std::nullopt,target,{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,target,{4_m,1_m,frc::Rotation2d{0_rad}},{}));
  EXPECT_FALSE(season::OffensiveHubShotAllowed(true,target,target,{.5_mps,0_mps,0_rad_per_s}));
  EXPECT_TRUE(season::OffensiveHubShotAllowed(true,target,target,{}));
}

TEST(OffensiveAutoDrive, PickupEstimateIsAnExplicitAmbiguousObservation) {
  season::OffensiveFuelEstimate estimate;
  const base::ObjectCluster mouth{{.4_m,0_m},1,1.,0_s};
  const base::ObjectCluster distant{{2_m,0_m},1,1.,0_s};
  estimate.Observe({mouth,distant},{});
  estimate.Observe({mouth,distant},{});
  EXPECT_EQ(estimate.Count(),0);
  // Disappearance near the intake also counts occlusion: this cannot prove pickup.
  estimate.Observe({distant},{});
  EXPECT_EQ(estimate.Count(),1);
  estimate.CompleteFeed(false);
  EXPECT_EQ(estimate.Count(),1);
  estimate.CompleteFeed(true);
  EXPECT_EQ(estimate.Count(),0);
  estimate.StartCollection();
  estimate.Observe({},{});
  EXPECT_EQ(estimate.Count(),0);
}

TEST(OffensiveAutoDrive, TargetLockKeepsNearestObservationWhenOrderChanges) {
  const frc::Translation2d locked{4_m,2_m};
  const base::ObjectCluster neighbor{{4.25_m,2_m},1,1.,0_s};
  const base::ObjectCluster same{{4.02_m,2_m},1,1.,0_s};
  const auto matched=season::MatchOffensiveTarget(locked,{neighbor,same});
  ASSERT_TRUE(matched);
  EXPECT_NEAR(matched->X().value(),4.02,1e-9);
  EXPECT_FALSE(season::MatchOffensiveTarget(locked,{{{5_m,2_m},1,1.,0_s}}));
}

TEST(OffensiveAutoDrive, ArbitrationUsesHubZoneAndEstimatedLoad) {
  using D=season::OffensiveDecision;
  const season::FieldGeometry field{16.541,8.069,{},{}};
  const auto blue=frc::DriverStation::Alliance::kBlue;
  const frc::Pose2d neutral{6_m,4_m,frc::Rotation2d{0_rad}};
  const frc::Pose2d home{2_m,4_m,frc::Rotation2d{0_rad}};
  const base::ObjectCluster friendly{{2_m,4_m},1,1.,0_s};
  const base::ObjectCluster nearby{{7_m,4_m},1,1.,0_s};
  EXPECT_EQ(season::DecideOffense(std::nullopt,blue,neutral,field,{},3,0),D::Wait);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},3,10),D::Collect);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},10,10),D::Score);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{},3,0),D::Score);
  EXPECT_EQ(season::DecideOffense(true,blue,home,field,{friendly},3,10),D::Collect);
  EXPECT_EQ(season::DecideOffense(true,blue,home,field,{},3,10),D::Score);
  EXPECT_EQ(season::DecideOffense(false,blue,neutral,field,{friendly},3,10),D::Ferry);
  EXPECT_EQ(season::DecideOffense(false,blue,home,field,{friendly},3,10,false,false,false,std::nullopt,true),D::Collect);
  EXPECT_EQ(season::DecideOffense(false,blue,neutral,field,{nearby},3,10),D::Collect);
  EXPECT_EQ(season::DecideOffense(false,blue,neutral,field,{nearby},10,10),D::Ferry);
  EXPECT_EQ(season::DecideOffense(false,blue,neutral,field,{nearby},1,10,false,true),D::Ferry);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},1,0,false,false,true,13.),D::Score);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},1,0,false,false,true,14.),D::Collect);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},1,0,false,false,false,std::nullopt,true),D::Score);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},0,0,false,false,false,std::nullopt,true),D::Score);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},0,0,false,false,false,std::nullopt,true,false),D::Collect);
  EXPECT_EQ(season::DecideOffense(true,blue,neutral,field,{nearby},0,0,false,false,true,13.),D::Score);
}

TEST(OffensiveAutoDrive, CommitmentSurvivesFailedAttemptButRespondsToHubChange) {
  using C=season::OffensiveCommitment;
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Score,true,false),C::Score);
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Score,std::nullopt,false),C::Score);
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Score,false,false),C::None);
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Ferry,false,false),C::Ferry);
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Ferry,true,false),C::None);
  EXPECT_EQ(season::ReconcileOffensiveCommitment(C::Ferry,false,true),C::None);
}

TEST(OffensiveAutoDrive, ReturnsBeforeActiveWindowClosesBasedOnRoute) {
  const season::FieldGeometry field{16.541,8.069,{},{}};
  const auto blue=frc::DriverStation::Alliance::kBlue;
  const frc::Pose2d neutral{8_m, 6_m, frc::Rotation2d{0_rad}};
  EXPECT_FALSE(season::OffensiveScoringWindowClosing(neutral,blue,field,std::nullopt));
  EXPECT_FALSE(season::OffensiveScoringWindowClosing(neutral,blue,field,25.));
  EXPECT_TRUE(season::OffensiveScoringWindowClosing(neutral,blue,field,12.));
  EXPECT_FALSE(season::OffensiveScoringWindowClosing(neutral,blue,field,-1.));
}
