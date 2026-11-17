package org.firstinspires.ftc.teamcode.auto.paths.red;

import com.pedropathing.api.Paths;

import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.AutoConstants;

public class RedDownAlliance2Path {
    //TODO REVISIT THIS AFTER SCRIMMAGE, WILL FOCUS ON SCRIMMAGE SOLO AUTO FOR NOW
    public Path gardenIntake() {
        return Paths.line(AutoConstants.redDownStartPose, AutoConstants.redGardenPose).constant(AutoConstants.redGardenPose);
    }

    public Path gardenShoot() {
        return Paths.line(AutoConstants.redGardenPose, AutoConstants.redDownStartPose).constant(AutoConstants.redDownStartPose);
    }

    public Path flowerIntake() {
        return Paths.curve(AutoConstants.redDownStartPose, AutoConstants.redAllianceAdjFlowerCtrl, AutoConstants.redAllianceAdjFlower).tangent();
    }

    public Path flowerShoot() {
        return Paths.line(AutoConstants.redAllianceAdjFlower, AutoConstants.redDownStartPose).linear(AutoConstants.redAllianceAdjFlower, AutoConstants.redDownStartPose);
    }
    public Path leave() {
        return Paths.curve(AutoConstants.redDownStartPose, AutoConstants.redDownLeaveCtrl, AutoConstants.redDownLeave).tangent();
    }
}
