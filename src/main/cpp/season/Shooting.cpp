#include "season/Shooting.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace season {
namespace {
constexpr double inf = std::numeric_limits<double>::infinity();
Point Sub(Point a, Point b) { return {a.x-b.x,a.y-b.y}; }
double Norm(Point p) { return std::hypot(p.x,p.y); }
Point Exit(Pose p) { return {p.x-.19*std::cos(p.heading),p.y-.19*std::sin(p.heading)}; }
double Wrap(double angle) { return std::atan2(std::sin(angle),std::cos(angle)); }
double Pitch(double hood) { return kPi/2 - (16*kPi/180 + hood*2*kPi); }
double Interpolate(double distance, const std::array<double,8>& values) {
  if (distance <= kDistanceInches.front()*.0254) return values.front();
  for (std::size_t i=1;i<values.size();++i) {
    if (distance <= kDistanceInches[i]*.0254) {
      double a=kDistanceInches[i-1]*.0254,b=kDistanceInches[i]*.0254;
      return values[i-1]+(values[i]-values[i-1])*(distance-a)/(b-a);
    }
  }
  return values.back();
}
Point Reference(Alliance a,const FieldGeometry& f) { return a==Alliance::Blue?f.blueTrenchReference:f.redTrenchReference; }
double SegmentDistance(Point point,Point start,Point end) {
  Point segment=Sub(end,start); double squared=segment.x*segment.x+segment.y*segment.y;
  if(squared<=1e-9) return Norm(Sub(point,start));
  Point v=Sub(point,start);
  double t=std::clamp((v.x*segment.x+v.y*segment.y)/squared,0.0,1.0);
  return Norm(Sub(point,{start.x+t*segment.x,start.y+t*segment.y}));
}
double HeightAtX(Point start,Point target,double x,double hood,double rps) {
  double dx=target.x-start.x;
  if(std::abs(dx)<1e-9) return -inf;
  double fraction=(x-start.x)/dx;
  if(fraction<0||fraction>1) return -inf;
  double pitch=Pitch(hood)-kPi/180, speed=rps*kProjectileSpeedPerRps*.99;
  double horizontal=speed*std::cos(pitch);
  if(horizontal<=0) return -inf;
  double time=Norm(Sub(target,start))*fraction/horizontal;
  return kReleaseHeight+speed*std::sin(pitch)*time-.5*9.80665*time*time;
}
}
Point HubPosition(Alliance a) { return {(a==Alliance::Blue?182.105:469.115)*.0254,158.845*.0254}; }
double AllianceBoundary(Alliance a,const FieldGeometry& f) { return a==Alliance::Blue?158.6*.0254:f.length-158.6*.0254; }
bool InAllianceZone(Alliance a,Point p,const FieldGeometry& f) { return a==Alliance::Blue?p.x<=AllianceBoundary(a,f):p.x>=AllianceBoundary(a,f); }
bool SeesAllianceSideTag(Alliance a,const std::vector<int>& ids) {
  int first=a==Alliance::Blue?17:1,last=a==Alliance::Blue?32:16;
  return std::any_of(ids.begin(),ids.end(),[=](int id){return id>=first&&id<=last;});
}
ShotTarget FerryTarget(Alliance a,Pose robot,const FieldGeometry& f) {
  double direction=a==Alliance::Blue?-1:1;
  Point reference{AllianceBoundary(a,f)+direction*.90,Reference(a,f).y+direction*.40};
  double low=std::min(reference.y,f.width-reference.y),high=std::max(reference.y,f.width-reference.y);
  return {ShotMode::Ferry,{reference.x,robot.y<=f.width/2?low:high}};
}
ShotTarget SelectTarget(Alliance a,Pose robot,const std::vector<int>& ids,const FieldGeometry& f) {
  return InAllianceZone(a,{robot.x,robot.y},f)||SeesAllianceSideTag(a,ids)?ShotTarget{ShotMode::Hub,HubPosition(a)}:FerryTarget(a,robot,f);
}
double FerryFlywheelRps(double distance,double hood) {
  double pitch=Pitch(hood),cosine=std::cos(pitch),height=kReleaseHeight+distance*std::tan(pitch);
  if(!std::isfinite(distance)||distance<0||cosine<=0||height<=0) return inf;
  return std::sqrt(9.80665*distance*distance/(2*cosine*cosine*height))/kProjectileSpeedPerRps;
}
FerryPlan CalculateFerry(Pose robot,ShotTarget target,const FieldGeometry& f,ShotTuning tuning) {
  Point start=Exit(robot); double distance=Norm(Sub(start,target.position));
  double maxHood=std::clamp(tuning.ferryHoodRotations,kHoodMin,kHoodMax), maxRps=std::max(0.0,tuning.ferryFlywheelRps);
  Alliance a=target.position.x<f.length/2?Alliance::Blue:Alliance::Red;
  double direction=a==Alliance::Blue?1:-1, center=Reference(a,f).x;
  double neutral=center+direction*47*.0254/2,wall=center-direction*47*.0254/2;
  double radius=std::hypot(47*.0254/2,47*.0254/2)+5.91*.0254/2+.12+.10;
  bool safe=InAllianceZone(a,target.position,f)&&(a==Alliance::Blue?start.x>neutral:start.x<neutral)&&SegmentDistance(HubPosition(a),start,target.position)>radius;
  if(safe) for(int step=0;step<=50;++step) {
    double hood=maxHood-(maxHood-kHoodMin)*step/50, rps=FerryFlywheelRps(distance,hood);
    double minHeight=40.25*.0254+5.91*.0254/2+.10;
    if(std::isfinite(rps)&&rps<=maxRps&&HeightAtX(start,target.position,neutral,hood,rps)>=minHeight&&HeightAtX(start,target.position,wall,hood,rps)>=minHeight) return {hood,rps,true};
  }
  return {maxHood,std::clamp(FerryFlywheelRps(distance,maxHood),0.0,maxRps),false};
}
ShotSolution CalculateShot(Pose robot,Point target,bool simulation) {
  Point delta=Sub(target,Exit(robot)); double distance=Norm(delta);
  return {distance,Wrap(std::atan2(delta.y,delta.x)-(simulation?0:kPi)),Interpolate(distance,kHoodTable),Interpolate(distance,kFlywheelTable),distance>=kMinDistance&&distance<=kMaxDistance};
}
ShotSolution CalculateShot(Pose robot,ShotTarget target,const FieldGeometry& field,bool simulation,ShotTuning tuning) {
  auto solution=CalculateShot(robot,target.position,simulation);
  if(target.mode==ShotMode::Ferry) { auto plan=CalculateFerry(robot,target,field,tuning); solution.hoodRotations=plan.hoodRotations;solution.flywheelRps=plan.flywheelRps;solution.calibrated=plan.safe; }
  return solution;
}
const char* ReadinessName(Readiness value) {
  static constexpr const char* names[]{"POSE_NOT_READY","OUT_OF_RANGE","UNSAFE_FERRY_PATH","MOVING_TOO_FAST","AIMING","HOOD_MOVING","FLYWHEEL_SPINNING_UP","CALIBRATION_MODE","READY","FORCE_FEED_OVERRIDE"};
  return names[static_cast<unsigned>(value)];
}
Readiness EvaluateReadiness(bool poseReady,ShotSolution s,double vx,double vy,double omega,double heading,double hood,double flywheel) {
  if(!poseReady||!std::isfinite(heading)) return Readiness::PoseNotReady;
  if(!s.calibrated||!std::isfinite(s.distanceMeters)||!std::isfinite(s.headingRadians)||!std::isfinite(s.hoodRotations)||!std::isfinite(s.flywheelRps)) return Readiness::OutOfRange;
  if(!std::isfinite(vx)||!std::isfinite(vy)||!std::isfinite(omega)||std::hypot(vx,vy)>.25||std::abs(omega)>10*kPi/180) return Readiness::MovingTooFast;
  if(std::abs(Wrap(heading-s.headingRadians))>2*kPi/180) return Readiness::Aiming;
  if(!std::isfinite(hood)||std::abs(hood-s.hoodRotations)>.002) return Readiness::HoodMoving;
  if(!std::isfinite(flywheel)||std::abs(flywheel-s.flywheelRps)>.75) return Readiness::FlywheelSpinningUp;
  return Readiness::Ready;
}
double ClampCalibrationHood(double value) { return std::isfinite(value)?std::clamp(value,kHoodMin,kHoodMax):0; }
double ClampCalibrationFlywheel(double value) { return std::isfinite(value)?std::clamp(value,0.0,60.0):0; }
void VolleyFeedGate::Reset(double now) { feeding_=false;lastFalseTime_=now; }
bool VolleyFeedGate::Calculate(bool held,double measured,double target,double now) {
  // A volley may tolerate real speed sag after spin-up, never invalid feedback.
  if(!held||!std::isfinite(measured)||!std::isfinite(target)||target<=0||!std::isfinite(now)) { Reset(now);return false; }
  if(!feeding_) { bool ready=std::abs(measured-target)<=3;
    if(!ready) lastFalseTime_=now;
    feeding_=ready&&now-lastFalseTime_>=kDebounceSeconds;
  }
  return feeding_;
}
ShotIntakeSettings::ShotIntakeSettings(double speed,double position):retractSpeedMetersPerSecond(std::isfinite(speed)?std::clamp(speed,.01,.50):.10),holdPositionMeters(std::isfinite(position)?std::clamp(position,.13,.30):.13) {}
}
