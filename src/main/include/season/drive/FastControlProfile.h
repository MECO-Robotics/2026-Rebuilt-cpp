#pragma once
#include "season/drive/NtGamepadConfig.h"

#include <frc/Filesystem.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace season::drive::fastcontrol
{
  // Robot-wide timing and input profile, overridden by deploy/fast-control.json.
  struct Profile
  {
    bool enabled = true;
    double periodMs = 2.0;
    bool ntGamepadEnabled = true;
    bool ntFallbackToDriverStation = true;
    std::string driverInput = "nt"; // Buttons and enable authority always remain on Driver Station.
    double ntPeriodMs = ntgamepadconfig::kDefaultPeriodMs;
    double ntStaleMs = ntgamepadconfig::kDefaultStaleMs;
    double sparkControlMs = 10.0;
    double talonFdControlMs = 4.0;
    double talonClassicControlMs = 10.0;
    double detectionPeriodMs = 5.0;
    double questPeriodMs = 5.0;
    double sparkStatusHz = 100.0;
    double talonFdStatusHz = 250.0;
    double talonClassicStatusHz = 100.0;
    double odometryFdHz = 500.0;
    double odometryClassicHz = 100.0;

    static Profile Load()
    {
      Profile p;
      const auto file = wpi::MemoryBuffer::GetFile(frc::filesystem::GetDeployDirectory() + "/fast-control.json");
      if (!file)
        return p;
      try
      {
        const auto j = wpi::json::parse(file.value()->GetCharBuffer());
        static const std::array<const char *, 17> known{
            "enabled", "periodMs", "ntGamepadEnabled", "ntFallbackToDriverStation", "driverInput", "ntPeriodMs", "ntStaleMs",
            "sparkControlMs", "talonFdControlMs", "talonClassicControlMs", "detectionPeriodMs", "questPeriodMs",
            "sparkStatusHz", "talonFdStatusHz", "talonClassicStatusHz", "odometryFdHz", "odometryClassicHz"};
        for (const auto &[key, value] : j.items())
        {
          (void)value;
          if (std::find(known.begin(), known.end(), key) == known.end())
            throw std::runtime_error("unknown configuration key: " + key);
        }
        p.enabled = j.value("enabled", p.enabled);
        p.periodMs = j.value("periodMs", p.periodMs);
        p.ntGamepadEnabled = j.value("ntGamepadEnabled", p.ntGamepadEnabled);
        p.ntFallbackToDriverStation = j.value("ntFallbackToDriverStation", p.ntFallbackToDriverStation);
        p.driverInput = j.value("driverInput", p.driverInput);
        p.ntPeriodMs = j.value("ntPeriodMs", p.ntPeriodMs);
        p.ntStaleMs = j.value("ntStaleMs", p.ntStaleMs);
        p.sparkControlMs = j.value("sparkControlMs", p.sparkControlMs);
        p.talonFdControlMs = j.value("talonFdControlMs", p.talonFdControlMs);
        p.talonClassicControlMs = j.value("talonClassicControlMs", p.talonClassicControlMs);
        p.detectionPeriodMs = j.value("detectionPeriodMs", p.detectionPeriodMs);
        p.questPeriodMs = j.value("questPeriodMs", p.questPeriodMs);
        p.sparkStatusHz = j.value("sparkStatusHz", p.sparkStatusHz);
        p.talonFdStatusHz = j.value("talonFdStatusHz", p.talonFdStatusHz);
        p.talonClassicStatusHz = j.value("talonClassicStatusHz", p.talonClassicStatusHz);
        p.odometryFdHz = j.value("odometryFdHz", p.odometryFdHz);
        p.odometryClassicHz = j.value("odometryClassicHz", p.odometryClassicHz);
      }
      catch (const std::exception &e)
      {
        throw std::runtime_error(std::string("Invalid deploy/fast-control.json: ") + e.what());
      }
      const auto bounded = [](double value, double low, double high, const char *name)
      {
        if (!std::isfinite(value) || value < low || value > high)
          throw std::runtime_error(std::string("fast-control.json value out of range: ") + name);
      };
      bounded(p.periodMs, 1, 20, "periodMs");
      bounded(p.ntPeriodMs, 5, 100, "ntPeriodMs");
      bounded(p.ntStaleMs, 20, 250, "ntStaleMs");
      bounded(p.sparkControlMs, 5, 100, "sparkControlMs");
      bounded(p.talonFdControlMs, 2, 100, "talonFdControlMs");
      bounded(p.talonClassicControlMs, 5, 100, "talonClassicControlMs");
      bounded(p.detectionPeriodMs, 5, 1000, "detectionPeriodMs");
      bounded(p.questPeriodMs, 5, 1000, "questPeriodMs");
      bounded(p.sparkStatusHz, 10, 100, "sparkStatusHz");
      bounded(p.talonFdStatusHz, 10, 500, "talonFdStatusHz");
      bounded(p.talonClassicStatusHz, 10, 100, "talonClassicStatusHz");
      bounded(p.odometryFdHz, 50, 500, "odometryFdHz");
      bounded(p.odometryClassicHz, 50, 100, "odometryClassicHz");
      const auto divisible = [&](double period, const char *name)
      {
        const double ratio = period / p.periodMs;
        if (std::abs(ratio - std::round(ratio)) > 1e-9)
          throw std::runtime_error(std::string("fast-control.json ") + name + " must be a multiple of periodMs");
      };
      divisible(p.sparkControlMs, "sparkControlMs");
      divisible(p.talonFdControlMs, "talonFdControlMs");
      divisible(p.talonClassicControlMs, "talonClassicControlMs");
      if (p.driverInput != "nt" && p.driverInput != "driverstation")
        throw std::runtime_error("fast-control.json driverInput must be 'nt' or 'driverstation'");
      return p;
    }
  };
  inline const Profile &Config()
  {
    static const Profile profile = Profile::Load();
    return profile;
  }
} // namespace season::drive::fastcontrol
