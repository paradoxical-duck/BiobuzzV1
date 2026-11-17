package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class AutoConstants {
    //*todo this is a class with constant auto poses and paths for consistency across paths
    public static PoseFactory poseFactory = PoseFactory.degrees();

    //*TODO RED DOWN ALLIANCE POSES
    public static Pose redDownStartPose = poseFactory.of(55, 9, 180);
    public static Pose redGardenPose = poseFactory.of(12, 9, 180);
    public static Pose redAllianceAdjFlower = poseFactory.of(10.7389, 37.7114, 111.3137);
    //*TODO flower pose for flower on side adjacent to red down start pose, when reaching flower from down side (NOT same as solo auto)
    public static Pose redAllianceAdjFlowerCtrl = poseFactory.of(19.7874, 14.5196, 0);
    public static Pose redDownLeave = poseFactory.of(13.3988, 92.707, 97.6053);
    public static Pose redDownLeaveCtrl = poseFactory.of(18.7138, 51.4447, 0);

    //*TODO RED SOLO (2) POSES
    public static Pose redFlowerAndShoot = poseFactory.of(53.85, 128.0821, 90);
    public static Pose redFlowerAndShootCtrl = poseFactory.of(13.9516, 81.4727, 0);
    public static Pose redSolo2Leave = poseFactory.of(13.2197, 116.5202, 90);

    public static Pose flowerAndShoot = redFlowerAndShoot;
    public static Pose flowerAndShootCtrl = redFlowerAndShootCtrl;

    public static Pose blueDownStartPose = poseFactory.of(86.5, 132.5, 0);
    public static Pose blueGardenPose = poseFactory.of(129.5, 132.5, 0);

    //*TODO BLUE SOLO (2) POSES
    public static Pose blueFlowerAndShoot = poseFactory.of(87.65, 13.4179, 270);
    public static Pose blueFlowerAndShootCtrl = poseFactory.of(127.5484, 60.0273, 180);
    public static Pose blueSolo2Leave = poseFactory.of(128.2803, 24.9798, 270);
}