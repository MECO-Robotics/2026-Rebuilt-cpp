#include "base/DeterministicSimulation.h"
#include "base/ModelFeedforward.h"
#include <gtest/gtest.h>
#include <vector>

TEST(DeterministicSimulation, MultirateReplayRetainsSensorLatencyAndOrdering) {
  const std::array<base::MotorReplaySample,3> log{{{0,{1}},{4000,{2}},{8000,{3}}}};
  auto run=[&] {
    base::ReplayMotorIO replay(log);
    base::LatencyChannel<base::MotorInputs,4> sensor;
    base::DeterministicSimulation simulation;
    std::vector<std::int64_t> commands;
    std::vector<double> positions;
    for(int i=0;i<10;++i) simulation.Step(
      [&](auto time){ commands.push_back(time); },
      [&](auto time){
        EXPECT_EQ(commands.size(),static_cast<std::size_t>(time/20000+1));
        if(auto sample=sensor.Receive(time)) positions.push_back(sample->position);
        replay.AdvanceTo(time);
        EXPECT_TRUE(sensor.Push(time+8000,replay.Read()));
        const auto position=replay.Read().position;
        replay.SetVoltage(6);
        EXPECT_DOUBLE_EQ(replay.Read().position,position);
      }, [](auto){});
    EXPECT_EQ(commands,(std::vector<std::int64_t>{0,20000}));
    EXPECT_THROW(replay.AdvanceTo(0),std::invalid_argument);
    return positions;
  };
  const auto positions=run();
  EXPECT_EQ(positions,run());
  ASSERT_EQ(positions.size(),8u);
  EXPECT_DOUBLE_EQ(positions[0],1);
  EXPECT_DOUBLE_EQ(positions[1],2);
  EXPECT_DOUBLE_EQ(positions[2],3);
  base::LatencyChannel<int,1> bounded;
  EXPECT_TRUE(bounded.Push(4,1));
  EXPECT_FALSE(bounded.Push(5,2));
  EXPECT_FALSE(bounded.Receive(3));
  EXPECT_EQ(bounded.Receive(4),1);
}

TEST(ModelFeedforward, MatchesMotorPhysicsAcrossRotationalAndLinearCoordinates) {
  auto motors=frc::DCMotor::NEO(2);
  base::Gains original; original.p=3; original.s=.2;
  base::MechanismModel model{.2,5,1,.85,4};
  auto angular=base::PretuneFeedforward(original,motors,model);
  const double velocity=2, acceleration=3;
  const double omega=velocity*2*std::numbers::pi;
  const double torque=(model.outputInertiaKgMetersSquared*acceleration*2*std::numbers::pi+4)/(5*.85);
  const double expected=motors.Voltage(units::newton_meter_t{torque},units::radians_per_second_t{omega*5}).value();
  EXPECT_NEAR(angular.v*velocity+angular.a*acceleration+angular.gravity,expected,1e-12);
  model.unitsPerOutputRotation=.4;
  auto linear=base::PretuneFeedforward(original,motors,model);
  EXPECT_NEAR(linear.v*(velocity*.4)+linear.a*(acceleration*.4)+linear.gravity,expected,1e-12);
  EXPECT_DOUBLE_EQ(linear.p,original.p);
  EXPECT_DOUBLE_EQ(linear.s,original.s);
  model.efficiency=0;
  EXPECT_THROW(base::PretuneFeedforward(original,motors,model),std::invalid_argument);
}
