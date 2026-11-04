#pragma once
#include "base/QuestNav.h"
#include "base/Vision.h"
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace season::hardware
{
  using namespace units::literals;
  // Polling interval used to subscribe to QuestNav measurements.
  inline constexpr double kQuestSubscriptionPeriodSeconds = .005;
  // Express the headset mount relative to the robot frame.
  inline base::QuestNavConfig QuestConfig(const frc::AprilTagFieldLayout &layout)
  {
    base::QuestNavConfig c;
    c.subscriptionPeriodSeconds = kQuestSubscriptionPeriodSeconds;
    c.fieldLength = layout.GetFieldLength().value();
    c.fieldWidth = layout.GetFieldWidth().value();
    c.robotToQuest = frc::Transform3d{units::meter_t{-13.5 * .0254}, 0_m, units::meter_t{8.5 * .0254},
                                      frc::Rotation3d{180_deg, 0_deg, 180_deg}};
    return c;
  }
  // Express the Limelight mount relative to the robot frame.
  inline base::VisionConfig VisionConfig(const frc::AprilTagFieldLayout &layout)
  {
    base::VisionConfig c;
    c.name = "limelight";
    c.fieldLength = layout.GetFieldLength().value();
    c.fieldWidth = layout.GetFieldWidth().value();
    c.robotToCamera = frc::Transform3d{0_m, 0_m, units::meter_t{21.25 * .0254}, frc::Rotation3d{0_deg, -30_deg, 0_deg}};
    return c;
  }
}
