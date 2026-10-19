#include "season/Shooting.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace season;
static void Near(double a,double b) { EXPECT_NEAR(a,b,1e-9); }
TEST(SeasonAlgorithms, MatchesJavaShootingAndReadinessCases) {
  for(std::size_t i=0;i<kDistanceInches.size();++i) {
    double d=kDistanceInches[i]*.0254;
    auto s=CalculateShot({d+.19,0,0},Point{0,0});
    Near(s.distanceMeters,d);Near(s.hoodRotations,kHoodTable[i]);Near(s.flywheelRps,kFlywheelTable[i]);Near(s.headingRadians,0);ASSERT_TRUE(s.calibrated);
  }
  auto midpoint=CalculateShot({(46.003+123.24)/2*.0254+.19,0,0},Point{0,0});
  Near(midpoint.hoodRotations,.0095);Near(midpoint.flywheelRps,33.95);
  ASSERT_TRUE(!CalculateShot({40*.0254+.19,0,0},Point{0,0}).calibrated);
  ASSERT_TRUE(!CalculateShot({250*.0254+.19,0,0},Point{0,0}).calibrated);
  auto rotated=CalculateShot({2,2,kPi/2},Point{2,0});Near(rotated.distanceMeters,1.81);Near(rotated.headingRadians,kPi/2);
  Near(CalculateShot({0,0,0},Point{-3,0}).headingRadians,0);
  Near(std::abs(CalculateShot({0,0,0},Point{3,0}).headingRadians),kPi);
  Near(CalculateShot({0,0,0},Point{3,0},true).headingRadians,0);
  // Synthetic geometry tests target selection independently of layout loading.
  FieldGeometry f{16.54,8.07,{4.6,7.8},{11.94,.27}};
  for(auto a:{Alliance::Blue,Alliance::Red}) {
    bool blue=a==Alliance::Blue;
    ASSERT_TRUE(SeesAllianceSideTag(a,{blue?17:1}));
    ASSERT_TRUE(SeesAllianceSideTag(a,{blue?32:16}));
    ASSERT_TRUE(!SeesAllianceSideTag(a,{0,33,blue?1:17}));
    ASSERT_TRUE(SelectTarget(a,{blue?1.0:15.0,2,0},{},f).mode==ShotMode::Hub);
    ASSERT_TRUE(SelectTarget(a,{8,2,0},{blue?17:1},f).mode==ShotMode::Hub);
    auto low=SelectTarget(a,{8,2,0},{},f),high=SelectTarget(a,{8,7,0},{},f);
    ASSERT_TRUE(low.mode==ShotMode::Ferry&&InAllianceZone(a,low.position,f));
    Near(low.position.y+high.position.y,f.width); ASSERT_TRUE(low.position.y<high.position.y);
  }
  ASSERT_TRUE(std::isfinite(FerryFlywheelRps(5,.049)));
  ASSERT_TRUE(std::isinf(FerryFlywheelRps(-1,.049)));
  auto unsafe=CalculateFerry({1,2,0},FerryTarget(Alliance::Blue,{8,2,0},f),f);ASSERT_TRUE(!unsafe.safe);
  auto safe=CalculateFerry({8,1,0},FerryTarget(Alliance::Blue,{8,1,0},f),f);ASSERT_TRUE(safe.safe);ASSERT_TRUE(safe.flywheelRps<=55);
  auto s=CalculateShot({3,0,0},Point{0,0});
  auto readiness=[&](bool pose,double vx,double omega,double heading,double hood,double rps){return EvaluateReadiness(pose,s,vx,0,omega,heading,hood,rps);};
  ASSERT_TRUE(readiness(false,0,0,s.headingRadians,s.hoodRotations,s.flywheelRps)==Readiness::PoseNotReady);
  ASSERT_TRUE(readiness(true,.26,0,s.headingRadians,s.hoodRotations,s.flywheelRps)==Readiness::MovingTooFast);
  ASSERT_TRUE(readiness(true,0,0,s.headingRadians+.1,s.hoodRotations,s.flywheelRps)==Readiness::Aiming);
  ASSERT_TRUE(readiness(true,0,0,s.headingRadians,s.hoodRotations+.003,s.flywheelRps)==Readiness::HoodMoving);
  ASSERT_TRUE(readiness(true,0,0,s.headingRadians,s.hoodRotations,s.flywheelRps+1)==Readiness::FlywheelSpinningUp);
  ASSERT_TRUE(readiness(true,0,0,s.headingRadians,s.hoodRotations,s.flywheelRps)==Readiness::Ready);
  double nan=std::numeric_limits<double>::quiet_NaN();
  Near(ClampCalibrationHood(nan),0);Near(ClampCalibrationHood(1),.049);Near(ClampCalibrationFlywheel(-1),0);Near(ClampCalibrationFlywheel(100),60);
  VolleyFeedGate gate;gate.Reset(0);
  ASSERT_TRUE(!gate.Calculate(true,0,37,0));ASSERT_TRUE(!gate.Calculate(true,30,37,.2));
  ASSERT_TRUE(!gate.Calculate(true,37,37,.25));ASSERT_TRUE(gate.Calculate(true,37,37,.36));ASSERT_TRUE(gate.Calculate(true,30,37,.37));
  ASSERT_TRUE(!gate.Calculate(false,30,37,.38));ASSERT_TRUE(!gate.Calculate(true,30,37,.39));ASSERT_TRUE(gate.Calculate(true,37,37,.6));
  gate.Reset(.6);ASSERT_TRUE(!gate.Calculate(true,37,37,.6));ASSERT_TRUE(!gate.Calculate(true,nan,37,.8));ASSERT_TRUE(!gate.Calculate(true,0,0,1));
  ShotIntakeSettings settings{-1,0};Near(settings.retractSpeedMetersPerSecond,.01);Near(settings.holdPositionMeters,.13);
  ShotIntakeSettings invalid{nan,nan};Near(invalid.retractSpeedMetersPerSecond,.1);Near(invalid.holdPositionMeters,.13);
}

#include "season/SeasonCommands.h"
#include "season/HardwareAdapter.h"
#include <frc/simulation/DriverStationSim.h>
#include <frc/DriverStation.h>
#include <hal/HAL.h>
namespace {
struct TestRig {
  std::vector<base::InjectedMotorIO*> observed;
  std::unique_ptr<base::MotorIO> IO(){auto io=std::make_unique<base::InjectedMotorIO>();observed.push_back(io.get());return io;}
  base::PositionJoint rack{"Rack",IO(),GainsFor(hardware::kRackGains)};
  base::Flywheel intake{"Intake",IO(),GainsFor(hardware::kIntakeGains)};
  base::Flywheel conveyor{"Conveyor",IO(),GainsFor(hardware::kConveyorGains)};
  base::Flywheel bottom{"Bottom",IO(),GainsFor(hardware::kIndexerGains)};
  base::Flywheel top{"Top",IO(),GainsFor(hardware::kIndexerGains)};
  base::PositionJoint hood{"Hood",IO(),GainsFor(hardware::kHoodGains)};
  base::Flywheel shooter{"Shooter",IO(),GainsFor(hardware::kShooterGains)};
  Mechanisms Get(){return {rack,intake,conveyor,bottom,top,hood,shooter};}
};
void EnableRobot(){HAL_Initialize(500,0);frc::sim::DriverStationSim::SetDsAttached(true);frc::sim::DriverStationSim::SetEnabled(true);frc::sim::DriverStationSim::NotifyNewData();frc::DriverStation::RefreshData();}
}
TEST(SeasonCommands, AcquisitionEndStopsEveryOwnedMechanism) {
  EnableRobot();TestRig rig;
  auto command=Acquire(rig.Get());command.get()->Initialize();command.get()->Execute();
  rig.intake.Periodic();rig.conveyor.Periodic();rig.bottom.Periodic();rig.top.Periodic();
  EXPECT_DOUBLE_EQ(rig.observed[1]->requestedVoltage,-10);
  EXPECT_DOUBLE_EQ(rig.observed[2]->requestedVoltage,5);
  EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,5);
  EXPECT_DOUBLE_EQ(rig.observed[4]->requestedVoltage,-5);
  EXPECT_FALSE(command.get()->IsFinished());
  command.get()->End(true);
  rig.intake.Periodic();rig.conveyor.Periodic();rig.bottom.Periodic();rig.top.Periodic();
  for(auto* io:rig.observed) EXPECT_DOUBLE_EQ(io->requestedVoltage,0);
  // Restarting the cancelled action is valid and remains held until interruption.
  command.get()->Initialize();command.get()->Execute();EXPECT_FALSE(command.get()->IsFinished());command.get()->End(true);
}
TEST(SeasonCommands, DriverPresetLatchesUntilExplicitIdle) {
  EnableRobot();TestRig rig;
  auto preset=ShooterPreset(rig.Get(),.02,40);preset.get()->Initialize();preset.get()->End(false);
  EXPECT_TRUE(rig.shooter.VelocityControlActive());EXPECT_DOUBLE_EQ(rig.shooter.VelocitySetpoint(),40);EXPECT_DOUBLE_EQ(rig.hood.DesiredPosition(),.02);
  auto idle=ShooterIdle(rig.Get());idle.get()->Initialize();idle.get()->End(false);
  EXPECT_FALSE(rig.shooter.VelocityControlActive());EXPECT_DOUBLE_EQ(rig.hood.DesiredPosition(),0);
}
#include <frc/simulation/SimHooks.h>
TEST(SeasonCommands, ManualVolleyWaitsLatchesSpeedSagAndResetsOnRelease) {
  EnableRobot();frc::sim::PauseTiming();TestRig rig;double rumble=0;
  rig.shooter.SetVelocity(37);rig.observed[6]->inputs.velocity=0;rig.shooter.Periodic();
  auto command=AgitateAndFeed(rig.Get(),[&rumble](double value){rumble=value;});
  command.get()->Initialize();command.get()->Execute();rig.bottom.Periodic();
  EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,0);
  rig.observed[6]->inputs.velocity=37;rig.shooter.Periodic();
  frc::sim::StepTiming(units::second_t{.2});command.get()->Execute();rig.bottom.Periodic();
  EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,-10);EXPECT_DOUBLE_EQ(rumble,.35);
  rig.observed[6]->inputs.velocity=20;rig.shooter.Periodic();command.get()->Execute();rig.bottom.Periodic();
  EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,-10);
  command.get()->End(true);rig.bottom.Periodic();EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,0);EXPECT_DOUBLE_EQ(rumble,0);
  command.get()->Initialize();command.get()->Execute();rig.bottom.Periodic();EXPECT_DOUBLE_EQ(rig.observed[3]->requestedVoltage,0);
  command.get()->End(true);frc::sim::ResumeTiming();
}
#include <networktables/NetworkTableInstance.h>
TEST(SeasonCommands, DriverHubCalibrationUpdatesLiveAndSnapshotsSelectedBranch) {
  EnableRobot();TestRig rig;
  auto table=nt::NetworkTableInstance::GetDefault().GetTable("TunableNumbers/ShooterCalibration");
  table->PutBoolean("Enabled",true);table->PutNumber("HoodRotations",.2);table->PutNumber("FlywheelRPS",90);
  auto calibration=DriverHubPreset(rig.Get());calibration.get()->Initialize();calibration.get()->Execute();calibration.get()->Execute();
  EXPECT_FALSE(calibration.get()->IsFinished());
  EXPECT_DOUBLE_EQ(rig.hood.DesiredPosition(),.049);EXPECT_DOUBLE_EQ(rig.shooter.VelocitySetpoint(),60);
  table->PutBoolean("Enabled",false);table->PutNumber("HoodRotations",.025);table->PutNumber("FlywheelRPS",38);
  calibration.get()->Execute();
  EXPECT_DOUBLE_EQ(rig.hood.DesiredPosition(),.025);EXPECT_DOUBLE_EQ(rig.shooter.VelocitySetpoint(),38);
  calibration.get()->End(true);rig.shooter.Periodic();rig.hood.Periodic();
  EXPECT_FALSE(rig.shooter.VelocityControlActive());EXPECT_DOUBLE_EQ(rig.observed[6]->requestedVoltage,0);EXPECT_DOUBLE_EQ(rig.observed[5]->requestedVoltage,0);
  auto normal=DriverHubPreset(rig.Get());normal.get()->Initialize();normal.get()->Execute();normal.get()->Execute();
  EXPECT_DOUBLE_EQ(rig.hood.DesiredPosition(),0);EXPECT_DOUBLE_EQ(rig.shooter.VelocitySetpoint(),30);
  table->PutBoolean("Enabled",true);table->PutNumber("FlywheelRPS",50);normal.get()->Execute();
  EXPECT_DOUBLE_EQ(rig.shooter.VelocitySetpoint(),30);normal.get()->End(true);table->PutBoolean("Enabled",false);
}
TEST(SeasonHardware, RackMotorRotationConvertsToJavaMetres) {
  // Physical 9:1 reduction plus drum circumference equals Java's
  // 112.787 motor rotations per metre, with no duplicate circumference.
  constexpr double physicalRatio=9;
  constexpr double metresPerOutputRotation=2*kPi*hardware::kRackDrumRadius;
  EXPECT_NEAR(1/hardware::kRackGearRatio,metresPerOutputRotation/physicalRatio,1e-12);
  auto gains=GainsFor(hardware::kHoodGains);
  EXPECT_DOUBLE_EQ(gains.gravity,.1);EXPECT_DOUBLE_EQ(gains.v,0);EXPECT_DOUBLE_EQ(gains.a,0);
}
#include <choreo/Choreo.h>
#include <choreo/trajectory/SwerveSample.h>
TEST(SeasonChoreo, DeployedTrajectoryLoadsAndAllianceFlipRoundTrips) {
  auto trajectory=choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("LeftBlueBump");
  ASSERT_TRUE(trajectory.has_value());
  auto initial=trajectory->GetInitialSample();ASSERT_TRUE(initial.has_value());
  EXPECT_NEAR(initial->x.value(),4.12235,1e-4);EXPECT_NEAR(initial->y.value(),7.45418,1e-4);
  EXPECT_NEAR(trajectory->GetTotalTime().value(),2.96515,1e-4);
  auto flipped=initial->Flipped<2026>();auto roundTrip=flipped.Flipped<2026>();
  EXPECT_NEAR(roundTrip.x.value(),initial->x.value(),1e-9);EXPECT_NEAR(roundTrip.y.value(),initial->y.value(),1e-9);
  EXPECT_GT(std::abs(flipped.x.value()-initial->x.value()),1);
}
