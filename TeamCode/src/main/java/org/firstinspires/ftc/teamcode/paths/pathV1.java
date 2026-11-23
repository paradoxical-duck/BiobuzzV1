package org.firstinspires.ftc.teamcode.paths;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class pathV1 {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    public final Pose start = poseFactory.of(32, 9, 180);
    private final Pose shoot1 = poseFactory.of(13, 9, 180);
    private final Pose shoot2Start = poseFactory.of(13, 9, 180);
    private final Pose shoot2 = poseFactory.of(37.3926, 118.6015, 90);
    private final Pose shoot2Control1 = poseFactory.of(24, 110, 0);
    private final Pose flower1Start = poseFactory.of(37.3926, 118.6015, 90);
    private final Pose flower1 = poseFactory.of(46.9856, 130.1902, 90);
    private final Pose parkStart = poseFactory.of(46.9856, 130.1902, 90);
    private final Pose park = poseFactory.of(10.063, 117.0475, 90);
    private final Pose parkControl1 = poseFactory.of(39.3444, 114.5848, 0);

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path shoot2() {
        return Paths.curve(shoot2Start, shoot2Control1, shoot2).linear(shoot2Start, shoot2);
    }

    public Path flower1() {
        return Paths.line(flower1Start, flower1).linear(flower1Start, flower1);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
