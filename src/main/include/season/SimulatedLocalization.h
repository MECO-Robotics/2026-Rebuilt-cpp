#pragma once
#ifndef __FRC_ROBORIO__
#include "base/Vision.h"
#include "meco/sim/IdealVisionObservation.h"
#include "meco/sim/SimulatedVision.h"
#include <frc/geometry/Pose3d.h>

namespace season::simulation {
// Robot-specific ideal sensor model. Geometry/mounts are supplied by the season;
// shared acquisition, filtering and estimator ownership remain in MecoBase.
struct SimulatedLocalization {
    base::VisionInputs limelight, quest;
    std::vector<int> tagIds;
    bool questAligned = false;

    void Reset() {
        limelight = {};
        quest = {};
        tagIds.clear();
        questAligned = false;
    }

    void Update(const frc::Pose3d& truth, units::second_t timestamp,
                const frc::AprilTagFieldLayout& layout,
                const frc::Transform3d& limelightMount, const frc::Transform3d& questMount) {
        limelight = {};
        quest = {};
        limelight.connected = quest.connected = quest.tracking = true;
        // LL3 standard lens; Quest optical frustum is an approximate model.
        tagIds = meco::sim::VisibleTags(truth, limelightMount, layout,
            units::degree_t{62.5}, units::degree_t{48.9}, units::meter_t{6});
        const auto questTags = meco::sim::VisibleTags(truth, questMount, layout,
            units::degree_t{90}, units::degree_t{90}, units::meter_t{6});
        // Model AprilTag-assisted absolute initialization, then continuous VIO.
        questAligned = questAligned || questTags.size() >= 2 || !tagIds.empty();
        quest.aligned = questAligned;
        const auto limelightObservation = meco::sim::MakeIdealVisionObservation(
            truth, timestamp, layout, !tagIds.empty(), units::meter_t{.75}, {.08, .08, .12});
        const auto questObservation = meco::sim::MakeIdealVisionObservation(
            truth, timestamp, layout, questAligned, units::meter_t{.75}, {.1, .1, .05});
        if (limelightObservation)
            limelight.observations.push_back({limelightObservation->pose, limelightObservation->timestamp, limelightObservation->stdDevs});
        if (questObservation)
            quest.observations.push_back({questObservation->pose, questObservation->timestamp, questObservation->stdDevs});
    }
};
}
#endif
