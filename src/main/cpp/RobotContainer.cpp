#include "season/SimulationLatency.h"
#include "season/SimulationField.h"
#include "season/SimulationPose.h"
#include "RobotContainer.h"
#include "season/HardwareAdapter.h"
#include "season/ChoreoAuto.h"
#include "season/SimulationTuning.h"
#include "season/SystemCheckSetup.h"
#include <frc/RobotBase.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <networktables/NetworkTableInstance.h>
#include <array>
#include <numbers>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <string_view>
#include <stdexcept>
using namespace units::literals;
namespace {
base::Mode Mode(){return frc::RobotBase::IsReal()?base::Mode::Real:base::Mode::Sim;}
base::DriveConfig DriveConfig(){
 base::DriveConfig c;c.bus="MECO CANIvore";c.pigeon=13;
 c.driveRatio=5.8909090909090915;c.steerRatio=12.1;c.couplingRatio=4.909090909090909;
 c.maxSpeed=5.24;c.maxAngularRate=7;c.driveKP=.15;c.driveKS=.1;c.driveKV=.124;
 const bool simulation=frc::RobotBase::IsSimulation();
 c.steerKP=100;c.steerKD=.5;
 c.steerKS=.2;c.steerKV=1.5;c.driveCurrent=55;c.slipCurrent=50;c.steerCurrent=30;
 // Simulation ready-to-drive maximum: robot, bumpers and battery (67.6 kg).
 // The real configuration remains unchanged. The yaw MOI is the existing
 // PathPlanner estimate; replace it when a measured or CAD value is available.
 c.massKg=simulation?67.6:53;c.moi=6.883;c.cof=simulation?2.1:2.9;c.simulation.wheelInertia=.01;c.simulation.steerInertia=.01;
 if(simulation){
   // Estimated bumper envelope: 0.25 inch floor gap, 5.5 inch height.
   // Move the body origin to bumper centre; preserve the assumed world CoM at .30 m.
   c.simulation.bumperHalfHeight=.1397/2;
   c.simulation.rideHeight=.00635+c.simulation.bumperHalfHeight;
   c.simulation.centerOfMassZ=.30-c.simulation.rideHeight;
 }
 c.autoConfigFromGUI=true;c.autoTranslationKP=15;c.autoRotationKP=10;
 c.modules={{{2,1,9,.3076171875,.2794,.2794,false,true,false},
 {8,3,10,.304931640625,.2794,-.2794,true,true,false},
 {6,5,11,-.083740234375,-.2794,.2794,false,true,false},
 {4,7,12,-.346435546875,-.2794,-.2794,true,true,false}}};
 // Physical mounting directions measured from the current module geometry.
 // These describe the simulated rotor/encoder axes, independently of the
 // Phoenix electrical inversion settings above.
 using Orientation=base::SimulationDeviceOrientation;
 c.simulation.modules={{{Orientation::CounterClockwisePositive,Orientation::ClockwisePositive,Orientation::CounterClockwisePositive},
                        {Orientation::ClockwisePositive,Orientation::ClockwisePositive,Orientation::CounterClockwisePositive},
                        {Orientation::CounterClockwisePositive,Orientation::ClockwisePositive,Orientation::CounterClockwisePositive},
                        {Orientation::ClockwisePositive,Orientation::ClockwisePositive,Orientation::CounterClockwisePositive}}};
 if(simulation){
   // Native Phoenix swerve requests enable FOC. Use the same electrical
   // plant and stator torque limit as the simulation adapter, not supply amps.
   const auto motor=frc::DCMotor::KrakenX60FOC(1);
   const double motorTorqueConstant=motor.Kt.value();
   std::array<season::simulation::ModuleLocation,4> locations{};
   for(std::size_t i=0;i<locations.size();++i)
     locations[i]={c.modules[i].xMeters,c.modules[i].yMeters};
   const auto tuned=season::simulation::DeriveSimulationTuning({
       c.massKg,c.moi,c.cof,c.wheelRadius,c.driveRatio,c.slipCurrent,motorTorqueConstant,locations});
   // The contact-loaded plant uses the configured robot steering gains.
   // The old simulation-only pole estimate assumed an unloaded azimuth motor.
   c.autoTranslationKP=tuned.autoTranslationKP;c.autoRotationKP=tuned.autoRotationKP;
 }
 return c;
}
std::unique_ptr<base::MotorIO> Motor(const season::hardware::FlywheelConfig& c,base::Vendor vendor){return base::MakeMotorIO(Mode(),vendor,season::MotorConfigFor(c),c.inertiaKgMetersSquared);}
season::FieldGeometry Geometry(const frc::AprilTagFieldLayout& layout){
 auto midpoint=[&](int a,int b){auto p=layout.GetTagPose(a).value(),q=layout.GetTagPose(b).value();return season::Point{(p.X().value()+q.X().value())/2,(p.Y().value()+q.Y().value())/2};};
 return {layout.GetFieldLength().value(),layout.GetFieldWidth().value(),midpoint(22,23),midpoint(6,7)};
}
base::VisionConfig VisionConfig(const frc::AprilTagFieldLayout& layout){
 base::VisionConfig c;c.name="limelight";c.fieldLength=layout.GetFieldLength().value();c.fieldWidth=layout.GetFieldWidth().value();
 c.robotToCamera=frc::Transform3d{0_m,0_m,units::meter_t{21.25*.0254},frc::Rotation3d{0_deg,-30_deg,0_deg}};return c;
}
}
season::Mechanisms RobotContainer::Mechanisms(){return {rack_,intake_,conveyor_,bottom_,top_,hood_,shooter_};}
RobotContainer::RobotContainer():drive_(DriveConfig()),
 rack_("IntakeRack",base::MakeMotorIO(Mode(),base::Vendor::SparkMax,{{21},{false},season::hardware::kRackGearRatio*2*std::numbers::pi*season::hardware::kRackDrumRadius,40,""},15*.45359237*.0127*.0127),(frc::RobotBase::IsSimulation()?season::SimulationGainsFor(season::hardware::kRackGains):season::GainsFor(season::hardware::kRackGains)),2*std::numbers::pi*season::hardware::kRackDrumRadius),
 hood_("Hood",base::MakeMotorIO(Mode(),base::Vendor::SparkMax,{{33},{false},105,40,""},.01),(frc::RobotBase::IsSimulation()?season::SimulationGainsFor(season::hardware::kHoodGains):season::GainsFor(season::hardware::kHoodGains)),1,base::Gravity::Cosine),
 intake_("IntakeRoller",Motor(season::hardware::kIntake,base::Vendor::SparkMax),season::GainsFor(season::hardware::kIntakeGains)),
 conveyor_("Conveyor",Motor(season::hardware::kConveyor,base::Vendor::SparkMax),season::GainsFor(season::hardware::kConveyorGains)),
 bottom_("BottomIndexer",Motor(season::hardware::kBottomIndexer,base::Vendor::TalonFX),season::GainsFor(season::hardware::kIndexerGains)),
 top_("TopIndexer",Motor(season::hardware::kTopIndexer,base::Vendor::SparkMax),season::GainsFor(season::hardware::kIndexerGains)),
 shooter_("ShooterFlywheel",Motor(season::hardware::kShooter,base::Vendor::TalonFX),season::GainsFor(season::hardware::kShooterGains)),
 layout_(frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField)),field_(Geometry(layout_)),
 vision_(VisionConfig(layout_),[this](const auto& obs){drive_.AddVision(obs.pose,obs.timestamp,obs.stdDevs);lastAcceptedVision_=frc::Timer::GetFPGATimestamp().value();},[this]{return drive_.Pose().Rotation();}),
 checks_([this]{drive_.Stop();rack_.Stop();hood_.Stop();intake_.Stop();conveyor_.Stop();bottom_.Stop();top_.Stop();shooter_.Stop();},frc::RobotBase::IsReal()?"/home/lvuser/systemcheck-latest.json":"systemcheck-latest.json") {
 drive_.ConfigureAuto();
#ifndef __FRC_ROBORIO__
 if(frc::RobotBase::IsSimulation()){
   frc::SmartDashboard::PutNumber("SimulationTuning/SteerKP",drive_.Config().steerKP);
   frc::SmartDashboard::PutNumber("SimulationTuning/SteerKD",drive_.Config().steerKD);
   frc::SmartDashboard::PutNumber("SimulationTuning/AutoTranslationKP",drive_.Config().autoTranslationKP);
   frc::SmartDashboard::PutNumber("SimulationTuning/AutoRotationKP",drive_.Config().autoRotationKP);
   simulationPosePublisher_ = nt::NetworkTableInstance::GetDefault()
       .GetStructTopic<frc::Pose3d>("/Simulation/RobotTruth").Publish();
   simulationBodyPosePublisher_ = nt::NetworkTableInstance::GetDefault()
       .GetStructTopic<frc::Pose3d>("/Simulation/PhysicsBodyTruth").Publish();
   const auto backend=base::sim::BackendFromEnvironment();
   const auto& physics=drive_.Config().simulation;
   // Hard-mounted wheels have a fixed geometric body-to-floor offset.
   simulationGroundedBodyHeight_=units::meter_t{physics.rideHeight};
   frc::SmartDashboard::PutNumber("FieldSimulation/GroundedBodyHeight",physics.rideHeight);
   frc::SmartDashboard::PutNumber("FieldSimulation/BumperCornerRadius",base::sim::kBumperCornerRadius);
   frc::SmartDashboard::PutNumber("FieldSimulation/BumperBottomHeight",physics.rideHeight-physics.bumperHalfHeight);
   frc::SmartDashboard::PutNumber("FieldSimulation/BumperTopHeight",physics.rideHeight+physics.bumperHalfHeight);
   auto simulation=meco::sim::CreateDriveSimulationAdapter(drive_.Config(),backend);
   const char* fieldMode=std::getenv("FRC_SIM_FIELD_COLLIDERS");
   simulationFieldCollidersEnabled_=!fieldMode || std::string_view{fieldMode}!="0";
   if(simulationFieldCollidersEnabled_){
     auto& arena=simulation->CreateArena(base::sim::ArenaScene::Empty);
     season::simulation::InstallField(arena);
   }
   frc::SmartDashboard::PutBoolean("FieldSimulation/CollidersEnabled",simulationFieldCollidersEnabled_);
   frc::SmartDashboard::PutNumber("FieldSimulation/ColliderCount",simulationFieldCollidersEnabled_?season::simulation::FieldColliders().size():0);
   simulationSession_=simulation.get(); // Drive owns the session for this container lifetime.
   drive_.SetSimulationAdapter(std::move(simulation));
   drive_.SetPoseResetCallback([this](const frc::Pose2d& pose){
     if(simulationResetFeedbackLoops_>0)simulationResetPose_=pose;
   });
 }
#endif
 drive_.SetDefaultCommand(drive_.Joystick([this]{return season::simulation::ObserveJoystick(-controller_.GetLeftY());},[this]{return -controller_.GetLeftX();},[this]{return -controller_.GetRightX();}));
#ifndef __FRC_ROBORIO__
 // Full scene recovery (including an overturned body), separate from Start's
 // driver heading reset. This binding never exists on the physical robot.
 controller_.Back().OnTrue(frc2::cmd::RunOnce([this]{ResetSimulation();}));
#endif
 auto m=Mechanisms();
 if(frc::RobotBase::IsSimulation())simulation_=std::make_unique<season::FuelSimulation>(m,drive_);
 // Presets latch between button presses. No idle default overwrites their setpoints.
 (controller_.Start() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(frc2::cmd::RunOnce([this]{drive_.ResetPose(frc::Pose2d{drive_.Pose().Translation(),frc::Rotation2d{}});},{&drive_}));
 (controller_.RightTrigger() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Acquire(m));
 (controller_.LeftTrigger() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Eject(m).AlongWith(frc2::cmd::RunEnd([this]{rack_.SetPosition(.30);},[this]{rack_.Stop();},{&rack_})));
 (controller_.RightBumper() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::AgitateAndFeed(m,[this](double intensity){controller_.GetHID().SetRumble(frc::GenericHID::RumbleType::kBothRumble,intensity);}));
 (controller_.POVUp() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Deploy(m));(copilot_.POVUp() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Deploy(m));
 (controller_.POVDown() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Stow(m));(copilot_.POVDown() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(season::Stow(m));
 (copilot_.POVLeft() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(frc2::cmd::RunOnce([this]{rack_.SetPosition(.30);},{&rack_}));
 (copilot_.POVRight() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(frc2::cmd::RunOnce([this]{rack_.SetPosition(0);},{&rack_}));
 (controller_.A() && []{return frc::DriverStation::IsTeleopEnabled();}).WhileTrue(drive_.Aim([this]{return -controller_.GetLeftY();},[this]{return -controller_.GetLeftX();},[this]{auto p=drive_.Pose();auto s=season::CalculateShot({p.X().value(),p.Y().value(),p.Rotation().Radians().value()},SelectTarget(),field_,frc::RobotBase::IsSimulation());return frc::Rotation2d{units::radian_t{s.headingRadians}};}));
 (controller_.X() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::DriverHubPreset(m));(copilot_.X() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::DriverHubPreset(m));
 (controller_.Y() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::ShooterPreset(m,.049,55));(copilot_.Y() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::ShooterPreset(m,.049,55));
 (copilot_.A() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::ShooterPreset(m,.049,35));
 (controller_.B() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::ShooterIdle(m));(copilot_.B() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(season::ShooterIdle(m));
 pathplanner::NamedCommands::registerCommand("DeployIntake",frc2::cmd::RunOnce([this]{rack_.SetPosition(.3);},{&rack_}));
 pathplanner::NamedCommands::registerCommand("StowIntake",frc2::cmd::RunOnce([this]{rack_.SetPosition(0);},{&rack_}));
 pathplanner::NamedCommands::registerCommand("FeedRollers",HubShot(false));
 pathplanner::NamedCommands::registerCommand("CoordinatedHubShot",HubShot(false));
 pathplanner::NamedCommands::registerCommand("IdleRollers",frc2::cmd::RunOnce([this]{conveyor_.Stop();bottom_.Stop();top_.Stop();},{&conveyor_,&bottom_,&top_}));
 pathplanner::NamedCommands::registerCommand("Agitate",frc2::cmd::RunEnd([this]{bottom_.SetVoltage(-10);top_.SetVoltage(10);},[this]{bottom_.Stop();top_.Stop();},{&bottom_,&top_}));
 pathplanner::NamedCommands::registerCommand("SpinIntake",frc2::cmd::RunEnd([this]{intake_.SetVoltage(-10);conveyor_.SetVoltage(5);},[this]{intake_.Stop();conveyor_.Stop();},{&intake_,&conveyor_}));
 auto calculatedSpinUp=[this]{return frc2::cmd::RunEnd([this]{
   auto p=drive_.Pose();auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed)==frc::DriverStation::Alliance::kBlue?season::Alliance::Blue:season::Alliance::Red;
   auto shot=season::CalculateShot({p.X().value(),p.Y().value(),p.Rotation().Radians().value()},season::HubPosition(alliance),frc::RobotBase::IsSimulation());
   hood_.SetPosition(shot.hoodRotations);shooter_.SetVelocity(shot.flywheelRps);
 },[this]{hood_.Stop();shooter_.Stop();},{&hood_,&shooter_});};
 pathplanner::NamedCommands::registerCommand("AutoSpinUp",calculatedSpinUp().WithTimeout(2_s));
 pathplanner::NamedCommands::registerCommand("Fender",season::ShooterPreset(m,0,30));
 pathplanner::NamedCommands::registerCommand("AutoAim",frc2::cmd::RunEnd([this]{
   auto p=drive_.Pose();auto target=SelectTarget();auto shot=season::CalculateShot({p.X().value(),p.Y().value(),p.Rotation().Radians().value()},target,field_,frc::RobotBase::IsSimulation());
   double error=std::remainder(shot.headingRadians-p.Rotation().Radians().value(),2*std::numbers::pi);
   drive_.RunVelocity(frc::ChassisSpeeds{0_mps,0_mps,units::radians_per_second_t{std::clamp(7*error,-7.,7.)}});
 },[this]{drive_.Stop();},{&drive_}).AlongWith(calculatedSpinUp()));
 autoChooser_=pathplanner::AutoBuilder::buildAutoChooser();
 seasonAutos_.emplace_back(HubShot(true,true));
 seasonAutos_.emplace_back(season::Auto(drive_,"LeftBlueBump").AndThen(HubShot()));
 seasonAutos_.emplace_back(frc2::cmd::None());
 autoChooser_.SetDefaultOption("Fender I HARDLY KNOW HER -JAVI",seasonAutos_[0].get());
 autoChooser_.AddOption("Choreo LeftBlueBump + Shoot",seasonAutos_[1].get());
 autoChooser_.AddOption("You better hit the A stop before this -Manny (none)",seasonAutos_[2].get());
 frc::SmartDashboard::PutData("Auto Choices",&autoChooser_);
 checks_.ConfigureRequirements({&drive_,&rack_,&hood_,&shooter_,&intake_,&conveyor_,&bottom_,&top_});
 season::ConfigureSystemChecks(checks_,drive_,rack_,hood_,shooter_,{{"intake",&intake_},{"conveyor",&conveyor_},{"bottom-indexer",&bottom_},{"top-indexer",&top_}},[this]{return vision_.Connected();});
 if(frc::RobotBase::IsSimulation())ResetSimulation();
}
season::ShotTarget RobotContainer::SelectTarget(){
 auto alliance=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed)==frc::DriverStation::Alliance::kBlue?season::Alliance::Blue:season::Alliance::Red;
 auto calibration=nt::NetworkTableInstance::GetDefault().GetEntry("/TunableNumbers/ShooterCalibration/Enabled").GetBoolean(false);
 if(calibration)return {season::ShotMode::Hub,season::HubPosition(alliance)};
 auto p=drive_.Pose();std::vector<int> ids;
 auto table=nt::NetworkTableInstance::GetDefault().GetTable("limelight");
 auto raw=table->GetNumberArray("rawfiducials",{});
 if(vision_.Connected())for(size_t i=0;i+6<raw.size();i+=7)ids.push_back(static_cast<int>(raw[i]));
 return season::SelectTarget(alliance,{p.X().value(),p.Y().value(),p.Rotation().Radians().value()},ids,field_);
}
frc2::CommandPtr RobotContainer::HubShot(bool ownDrive,bool requireVision){season::CoordinatedShotOptions o;o.ownDrive=ownDrive;o.autonomous=true;o.simulation=frc::RobotBase::IsSimulation();o.poseReady=[this,requireVision]{return (!requireVision&&frc::DriverStation::IsAutonomousEnabled())||frc::Timer::GetFPGATimestamp().value()-lastAcceptedVision_<.5;};return season::CoordinatedShot(Mechanisms(),drive_,field_,[]{auto a=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed)==frc::DriverStation::Alliance::kBlue?season::Alliance::Blue:season::Alliance::Red;return season::ShotTarget{season::ShotMode::Hub,season::HubPosition(a)};},o);}
frc2::Command* RobotContainer::GetAutonomousCommand(){return autoChooser_.GetSelected();}
void RobotContainer::CancelChecks(){checks_.Cancel("Robot mode changed");}

void RobotContainer::SimulationPeriodic(){
 if(simulation_)simulation_->Periodic();
#ifndef __FRC_ROBORIO__
 if(auto truth=drive_.SimulationTruth()){
   // Scene reset changes encoder and gyro raw inputs asynchronously. Reconcile
   // once after transport/odometry has consumed them. Preserve any autonomous
   // pose seed applied since reset and motion in that frame. No loop waits.
   if(simulationResetFeedbackLoops_>0 && --simulationResetFeedbackLoops_==0)
     drive_.ResetPose(simulationResetPose_.TransformBy(frc::Transform2d{
         simulationResetPhysicalOrigin_,frc::Pose2d{units::meter_t{truth->x},units::meter_t{truth->y},
                                  frc::Rotation2d{units::radian_t{truth->yaw}}}}));
   const auto q=simulationSession_->PhysicsTruth().body.pose.rotation;
   const frc::Pose3d bodyPose{
       units::meter_t{truth->x}, units::meter_t{truth->y}, units::meter_t{truth->z},
       frc::Rotation3d{frc::Quaternion{q.w,q.x,q.y,q.z}}};
   simulationBodyPosePublisher_.Set(bodyPose);
   simulationPosePublisher_.Set(season::simulation::FloorReferencedRobotPose(bodyPose,simulationGroundedBodyHeight_));
 }
#endif
}
void RobotContainer::ResetSimulation(){
#ifndef __FRC_ROBORIO__
 const bool red=frc::DriverStation::GetAlliance()==frc::DriverStation::Alliance::kRed;
 simulationResetPhysicalOrigin_=simulationFieldCollidersEnabled_?season::simulation::DefaultSpawn(red):frc::Pose2d{};
 if(simulationFieldCollidersEnabled_ && !season::simulation::IsClearStartingPose(simulationResetPhysicalOrigin_))
   throw std::logic_error("Simulation spawn intersects field geometry");
 simulationResetFeedbackLoops_=5;
 drive_.ResetSimulation(simulationResetPhysicalOrigin_);
#endif
 if(simulation_)simulation_->Reset();
}
