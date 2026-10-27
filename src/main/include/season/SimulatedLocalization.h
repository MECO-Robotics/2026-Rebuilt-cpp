#pragma once
#ifndef __FRC_ROBORIO__
#include "base/Vision.h"
#include <frc/geometry/Pose3d.h>
#include <cmath>
#include <numbers>

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

    static std::vector<int> VisibleTags(const frc::Pose3d& robot,
                                       const frc::Transform3d& mount,
                                       const frc::AprilTagFieldLayout& layout,
                                       double horizontalFov, double verticalFov) {
        const auto camera = robot.TransformBy(mount);
        std::vector<int> ids;
        for (const auto& tag : layout.GetTags()) {
            const frc::Transform3d relative{camera, tag.pose};
            const auto& p = relative.Translation();
            // Reject tag backs as well as targets outside the optical frustum.
            const frc::Transform3d tagToCamera{tag.pose, camera};
            if (p.X().value() <= 0 || p.Norm().value() > 6 ||
                tagToCamera.X().value() <= 0) continue;
            if (std::abs(std::atan2(p.Y().value(), p.X().value())) < horizontalFov / 2 &&
                std::abs(std::atan2(p.Z().value(), std::hypot(p.X().value(), p.Y().value()))) < verticalFov / 2)
                ids.push_back(tag.ID);
        }
        return ids;
    }

    void Update(const frc::Pose3d& truth, units::second_t timestamp,
                const frc::AprilTagFieldLayout& layout,
                const frc::Transform3d& limelightMount, const frc::Transform3d& questMount) {
        limelight = {};
        quest = {};
        limelight.connected = quest.connected = quest.tracking = true;
        constexpr double degrees = std::numbers::pi / 180;
        // LL3 standard lens; Quest optical frustum is an approximate model.
        tagIds = VisibleTags(truth, limelightMount, layout, 62.5 * degrees, 48.9 * degrees);
        const auto questTags = VisibleTags(truth, questMount, layout, 90 * degrees, 90 * degrees);
        // Model AprilTag-assisted absolute initialization, then continuous VIO.
        questAligned = questAligned || questTags.size() >= 2 || !tagIds.empty();
        quest.aligned = questAligned;
        const bool onField = truth.X().value() >= 0 && truth.Y().value() >= 0 &&
            truth.X() <= layout.GetFieldLength() && truth.Y() <= layout.GetFieldWidth() &&
            std::abs(truth.Z().value()) <= .75;
        if (!onField) return;
        if (!tagIds.empty())
            limelight.observations.push_back({truth.ToPose2d(), timestamp, {.08, .08, .12}});
        if (questAligned)
            quest.observations.push_back({truth.ToPose2d(), timestamp, {.1, .1, .05}});
    }
};
}
#endif
