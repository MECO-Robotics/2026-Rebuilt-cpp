#pragma once
namespace season::simulation::motorconfig
{
  inline constexpr double kNominalVoltage = 12.;
  inline constexpr double kNeoStallCurrentAmps = 105, kNeoStallTorqueNm = 2.6, kNeoFreeSpeedRpm = 5676;
  inline constexpr double kKrakenStallCurrentAmps = 366, kKrakenStallTorqueNm = 7.09, kKrakenFreeSpeedRpm = 6000;
  inline constexpr double kMaxUpdateSeconds = 1;
  inline constexpr double kMaxPhysicsStepSeconds = .0002, kLoadDirectionSpeedThreshold = .01;
  inline constexpr double kViscousDrag = .0001;
}
