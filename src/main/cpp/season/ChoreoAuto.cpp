#include "season/ChoreoAuto.h"
#include <choreo/Choreo.h>
#include <choreo/trajectory/SwerveSample.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/Errors.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <cmath>
#include <memory>
namespace season {
frc2::CommandPtr Auto(base::Drive& drive,std::string name) {
  auto trajectory=choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(name);
  if(!trajectory||!trajectory->GetInitialPose()) {
    return frc2::cmd::RunOnce([&drive,name]{drive.Stop();FRC_ReportError(frc::warn::Warning,"Unable to load Choreo trajectory: {}",name);},{&drive});
  }
  struct State {
    choreo::Trajectory<choreo::SwerveSample> trajectory;
    frc::Timer timer;
    bool flipped=false;
    pathplanner::PPHolonomicDriveController controller{pathplanner::PIDConstants{10,0,0},pathplanner::PIDConstants{7,0,0},units::second_t{.02}};
  };
  auto state=std::make_shared<State>();state->trajectory=std::move(*trajectory);
  return frc2::cmd::RunEnd([&drive,state]{
    auto sample=state->trajectory.SampleAt<2026>(state->timer.Get(),state->flipped);if(!sample){drive.Stop();return;}
    pathplanner::PathPlannerTrajectoryState target;
    target.time=sample->timestamp;target.pose=sample->GetPose();target.fieldSpeeds=sample->GetChassisSpeeds();
    target.linearVelocity=units::meters_per_second_t{std::hypot(sample->vx.value(),sample->vy.value())};
    target.heading=target.linearVelocity.value()>1e-6?frc::Rotation2d{sample->vx.value(),sample->vy.value()}:target.pose.Rotation();
    // Java populates wheel force metadata but sends only calculated chassis speeds.
    drive.RunVelocity(frc::ChassisSpeeds::Discretize(state->controller.calculateRobotRelativeSpeeds(drive.Pose(),target),units::second_t{.02}));
  },[&drive,state]{state->timer.Stop();drive.Stop();},{&drive})
  .BeforeStarting([&drive,state]{state->flipped=frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed;drive.ResetPose(*state->trajectory.GetInitialPose<2026>(state->flipped));state->controller.reset(drive.Pose(),drive.Speeds());state->timer.Restart();})
  .Until([state]{return state->timer.HasElapsed(state->trajectory.GetTotalTime());});
}
}
