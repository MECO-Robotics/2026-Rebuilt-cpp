#include "RobotContainer.h"
#include "season/HardwareAdapter.h"
#include "season/ChoreoAuto.h"
#include "season/SystemCheckSetup.h"
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <networktables/NetworkTableInstance.h>
#include <numbers>
#include <algorithm>
#include <cmath>
using namespace units::literals;
namespace {
base::Mode Mode(){return frc::RobotBase::IsReal()?base::Mode::Real:base::Mode::Sim;}
base::DriveConfig DriveConfig(){
 base::DriveConfig c;c.bus="MECO CANIvore";c.pigeon=13;
 c.driveRatio=5.8909090909090915;c.steerRatio=12.1;c.couplingRatio=4.909090909090909;
 c.maxSpeed=5.24;c.maxAngularRate=7;c.driveKP=.15;c.driveKS=.1;c.driveKV=.124;
 c.steerKP=100;c.steerKD=.5;c.steerKS=.2;c.steerKV=1.5;c.driveCurrent=55;c.steerCurrent=30;
 c.massKg=53;c.speedAt12Volts=5.5;c.driveInertia=.01;c.steerInertia=.01;
 c.fusedEncoder=true;c.autoConfigFromGUI=true;c.autoTranslationKP=15;c.autoRotationKP=10;
 c.modules={{{2,1,9,.3076171875,.2794,.2794,false,true,false},
 {8,3,10,.304931640625,.2794,-.2794,true,true,false},
 {6,5,11,-.083740234375,-.2794,.2794,false,true,false},
 {4,7,12,-.346435546875,-.2794,-.2794,true,true,false}}};return c;
}
std::unique_ptr<base::MotorIO> Motor(const season::hardware::FlywheelConfig& c,base::Vendor vendor){return base::MakeMotorIO(Mode(),vendor,season::MotorConfigFor(c),c.inertiaKgMetersSquared);}
season::FieldGeometry Geometry(const frc::AprilTagFieldLayout& layout){
 auto midpoint=[&](int a,int b){auto p=layout.GetTagPose(a).value(),q=layout.GetTagPose(b).value();return season::Point{(p.X().value()+q.X().value())/2,(p.Y().value()+q.Y().value())/2};};
 return {layout.GetFieldLength().value(),layout.GetFieldWidth().value(),midpoint(22,23),midpoint(6,7)};
}
base::QuestNavConfig QuestConfig(const frc::AprilTagFieldLayout& layout){
 base::QuestNavConfig c; c.fieldLength=layout.GetFieldLength().value(); c.fieldWidth=layout.GetFieldWidth().value();
 // Java VisionConstants: Quest 3S faces -X and is mounted upside down.
 c.robotToHeadset=frc::Transform3d{units::meter_t{-13.5*.0254},0_m,units::meter_t{8.5*.0254},frc::Rotation3d{180_deg,0_deg,180_deg}};
 return c;
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
 quest_(QuestConfig(layout_)),
 vision_(VisionConfig(layout_),[this](const auto& obs){
   if(!absoluteVision_||obs.timestamp>absoluteVision_->timestamp)absoluteVision_=obs;
   if(pendingAbsoluteCount_==pendingAbsolute_.size()){
     pendingAbsoluteStart_=(pendingAbsoluteStart_+1)%pendingAbsolute_.size();--pendingAbsoluteCount_;++droppedAbsolute_;
   }
   pendingAbsolute_[(pendingAbsoluteStart_+pendingAbsoluteCount_++)%pendingAbsolute_.size()]=obs;
 },[this]{return drive_.Pose().Rotation();}),
 checks_([this]{drive_.Stop();rack_.Stop();hood_.Stop();intake_.Stop();conveyor_.Stop();bottom_.Stop();top_.Stop();shooter_.Stop();},frc::RobotBase::IsReal()?"/home/lvuser/systemcheck-latest.json":"systemcheck-latest.json") {
 drive_.ConfigureAuto();
 drive_.SetDefaultCommand(drive_.Joystick([this]{return -controller_.GetLeftY();},[this]{return -controller_.GetLeftX();},[this]{return -controller_.GetRightX();}));
 auto m=Mechanisms();
 if(frc::RobotBase::IsSimulation())simulation_=std::make_unique<season::FuelSimulation>(m,drive_);
 // Presets latch between button presses. No idle default overwrites their setpoints.
 (controller_.Start() && []{return frc::DriverStation::IsTeleopEnabled();}).OnTrue(frc2::cmd::RunOnce([this]{ResetLocalization(frc::Pose2d{drive_.Pose().Translation(),frc::Rotation2d{}});},{&drive_}));
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
 season::ConfigureSystemChecks(checks_,drive_,rack_,hood_,shooter_,{{"intake",&intake_},{"conveyor",&conveyor_},{"bottom-indexer",&bottom_},{"top-indexer",&top_}},[this]{return vision_.Connected()&&quest_.Connected(frc::Timer::GetFPGATimestamp());});
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
frc2::CommandPtr RobotContainer::HubShot(bool ownDrive,bool requireVision){season::CoordinatedShotOptions o;o.ownDrive=ownDrive;o.autonomous=true;o.simulation=frc::RobotBase::IsSimulation();o.poseReady=[this,requireVision]{return (!requireVision&&frc::DriverStation::IsAutonomousEnabled())||quest_.Ready(frc::Timer::GetFPGATimestamp())||frc::Timer::GetFPGATimestamp().value()-lastAcceptedVision_<.5;};return season::CoordinatedShot(Mechanisms(),drive_,field_,[]{auto a=frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kRed)==frc::DriverStation::Alliance::kBlue?season::Alliance::Blue:season::Alliance::Red;return season::ShotTarget{season::ShotMode::Hub,season::HubPosition(a)};},o);}
frc2::Command* RobotContainer::GetAutonomousCommand(){return autoChooser_.GetSelected();}
void RobotContainer::CancelChecks(){checks_.Cancel("Robot mode changed");}

void RobotContainer::SimulationPeriodic(){if(simulation_)simulation_->Periodic();}
void RobotContainer::ResetSimulation(){if(simulation_)simulation_->Reset();}

// Called after scheduler subsystem Periodic methods have supplied the current
// absolute-camera observation. All vision handoffs remain on the robot thread;
// Phoenix owns synchronization with its odometry thread.
void RobotContainer::Periodic(){
 const auto now=frc::Timer::GetFPGATimestamp();
 if(pendingAbsoluteCount_&&absoluteVision_&&absoluteVision_->timestamp<=now&&
     now-absoluteVision_->timestamp<.5_s)lastAcceptedVision_=absoluteVision_->timestamp.value();
 bool fusedQuest=false;
 questTransport_.Poll([&](const base::QuestNavFrame& frame){
   std::optional<base::QuestNavObservation> absolute;
   if(absoluteVision_)absolute=base::QuestNavObservation{absoluteVision_->pose,absoluteVision_->timestamp};
   if(auto observation=quest_.Update(frame,now,absolute)){
     // Java camera zero uses the 0.02 m / 0.06 rad baselines directly for Quest.
     drive_.AddVision(observation->pose,observation->timestamp,{.02,.02,.06});
     fusedQuest=true;
   }
 });
 // Absolute vision corrects Quest's field transform when it is tracking. Fuse
 // that source independently only during fallback, avoiding correlated updates
 // from the same absolute measurement and its derived Quest pose in one cycle.
 const bool questReady=quest_.Ready(now);
 if(!fusedQuest&&!questReady){
   for(std::size_t i=0;i<pendingAbsoluteCount_;++i){
     const auto& observation=pendingAbsolute_[(pendingAbsoluteStart_+i)%pendingAbsolute_.size()];
     const auto age=now-observation.timestamp;
     if(age>=0_s&&age<.5_s)drive_.AddVision(observation.pose,observation.timestamp,observation.stdDevs);
   }
 }
 pendingAbsoluteCount_=pendingAbsoluteStart_=0;
 if(now.value()-lastQuestDiagnostic_>=.1){
   lastQuestDiagnostic_=now.value();
   frc::SmartDashboard::PutBoolean("QuestNav/Connected",quest_.Connected(now));
   frc::SmartDashboard::PutBoolean("QuestNav/FieldAligned",quest_.Initialized());
   frc::SmartDashboard::PutBoolean("QuestNav/PoseReady",questReady);
   frc::SmartDashboard::PutBoolean("QuestNav/BypassingToAbsolute",!questReady);
   frc::SmartDashboard::PutBoolean("QuestNav/AbsoluteConnected",vision_.Connected());
   frc::SmartDashboard::PutNumber("QuestNav/RejectedFrames",quest_.RejectedFrames());
   frc::SmartDashboard::PutNumber("QuestNav/DecodeFailures",questTransport_.DecodeFailures());
   frc::SmartDashboard::PutNumber("QuestNav/DroppedAbsoluteFrames",droppedAbsolute_);
   const auto version=questTransport_.Version();
   frc::SmartDashboard::PutString("QuestNav/Version",version);
   frc::SmartDashboard::PutBoolean("QuestNav/VersionMatches",version=="2026-2.2.0");
   const auto& device=questTransport_.Device();
   frc::SmartDashboard::PutNumber("QuestNav/BatteryPercent",device?device->batteryPercent:-1);
   frc::SmartDashboard::PutNumber("QuestNav/TrackingLostCounter",device?device->trackingLostCounter:-1);
   frc::SmartDashboard::PutNumber("QuestNav/LastAbsoluteObservationTimestamp",quest_.LastAbsoluteTimestamp().value());
   const double absoluteAge=absoluteVision_?std::max(0.,now.value()-absoluteVision_->timestamp.value()):-1;
   frc::SmartDashboard::PutNumber("QuestNav/AbsoluteObservationAgeSeconds",absoluteAge);
 }
}
void RobotContainer::ResetLocalization(const frc::Pose2d& pose){
 drive_.ResetPose(pose);
 if(!quest_.ResetPose(pose,frc::Timer::GetFPGATimestamp()))quest_.InvalidateAlignment();
 absoluteVision_.reset();
 pendingAbsoluteCount_=pendingAbsoluteStart_=0;
 lastAcceptedVision_=-1e9;
}
