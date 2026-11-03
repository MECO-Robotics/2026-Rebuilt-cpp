#pragma once
namespace season::simulation::runtimeconfig
{
  inline constexpr int kResetFeedbackLoops = 5, kQuestAlignmentMinTags = 2;
  inline constexpr unsigned kFuelRandomSeed = 3737;
  inline constexpr double kSimulationPeriodSeconds = .02;
  inline constexpr double kLimelightHorizontalFovDegrees = 62.5, kLimelightVerticalFovDegrees = 48.9;
  inline constexpr double kQuestHorizontalFovDegrees = 90, kQuestVerticalFovDegrees = 90, kVisionRangeMetres = 6;
  inline constexpr double kVisionTagDistanceNoiseMetres = .75;
  inline constexpr double kLimelightStdX = .08, kLimelightStdY = .08, kLimelightStdHeading = .12;
  inline constexpr double kQuestStdX = .1, kQuestStdY = .1, kQuestStdHeading = .05;
  inline constexpr double kRampClearancePaddingMetres = .005, kSpawnLaneWidthInches = 50.34;
  inline constexpr double kSpawnXMetres = 3.65, kFootprintHalfLengthMetres = .43, kFootprintHalfWidthMetres = .43;
  inline constexpr double kFootprintHeightMetres = .45, kSpawnClearanceMetres = .02;
  inline constexpr int kPracticeAutonomousSeconds = 20, kPracticeAssessmentSeconds = 3, kPracticeTeleopSeconds = 140;
  inline constexpr double kBatteryCapacityAh = 18.0, kBatteryEmptyVoltage = 10.5;
  inline constexpr double kBatteryVoltageRange = 2.1, kBatteryInternalResistanceOhms = .020;
}
