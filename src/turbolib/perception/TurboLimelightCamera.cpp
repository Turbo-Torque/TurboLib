#include "turbolib/perception/TurboLimelightCamera.hpp"

#include "frc/Timer.h"
#include "networktables/NetworkTableInstance.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "turbolib/structure/PoseTimestampPair.hpp"
#include "units/angle.h"
#include "units/time.h"
#include <vector>

using namespace turbolib::perception;

TurboLimelightCamera::TurboLimelightCamera(const std::string& limelightName, const frc::Transform3d& cameraInBotSpace)
    : limelightName(limelightName) {
    auto table = nt::NetworkTableInstance::GetDefault().GetTable(limelightName);
    botposeSubscriber = table->GetDoubleArrayTopic("botpose_wpiblue").Subscribe({});
    targetValidSubscriber = table->GetDoubleTopic("tv").Subscribe(0.0);
    cameraPosePublisher = table->GetDoubleArrayTopic("camerapose_robotspace_set").Publish();

    // ADDED: tells the Limelight where it's physically mounted on the robot (forward/side/up in
    // meters, roll/pitch/yaw in degrees — same axis convention Transform3d already uses, no
    // conversion needed) so botpose_wpiblue computes the ROBOT's pose, not just the camera's.
    // Set once here since the mount is physically fixed; per Limelight's docs this overrides
    // whatever's set in its web UI, which should be zeroed out to avoid ambiguity about which wins.
    const frc::Translation3d& t = cameraInBotSpace.Translation();
    const frc::Rotation3d& r = cameraInBotSpace.Rotation();
    std::vector<double> robotSpacePose = {
        t.X().value(), t.Y().value(), t.Z().value(),
        units::degree_t{r.X()}.value(), units::degree_t{r.Y()}.value(), units::degree_t{r.Z()}.value()
    };
    cameraPosePublisher.Set(robotSpacePose);
}

std::vector<turbolib::structure::PoseTimestampPair> TurboLimelightCamera::FetchPose() {
    std::vector<turbolib::structure::PoseTimestampPair> poses;

    std::vector<double> botpose = botposeSubscriber.Get();
        if (botpose.size() < 6) {
            seesTag = false;
            return poses;
        }


    if (targetValidSubscriber.Get() < 0.5) {
        seesTag = false;
        return poses;
    }

    frc::Pose2d pose{units::meter_t{botpose[0]}, units::meter_t{botpose[1]}, frc::Rotation2d{units::degree_t{botpose[5]}}};

    units::second_t timestamp = frc::Timer::GetFPGATimestamp();
    if (botpose.size() >= 7) {
        timestamp -= units::millisecond_t{botpose[6]};
    }

    poses.emplace_back(pose, timestamp);
    seesTag = true;
    return poses;
}