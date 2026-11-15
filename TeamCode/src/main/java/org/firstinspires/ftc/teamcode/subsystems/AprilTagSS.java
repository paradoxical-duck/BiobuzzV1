package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.ArrayList;

public class AprilTagSS {
    RobotHardware robot = RobotHardware.getInstance();

    public final ArrayList<String> detectedClusters = new ArrayList<>();
    private boolean ready = false;
    public AprilTagSS(RobotHardware robot) {
        this.robot = robot;
    }

    public void update() {
        detectedClusters.clear();
        ready = false;

        for (AprilTagDetection detection : robot.arducam.getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = cluster.metadata.name;

                detectedClusters.add(name);

                if (cluster.percentClusterFound >= 50) { //if more than 2 april tags found (for redundancy/safety)
                    if ((name.startsWith("R") && RobotHardware.getAlliance() == RobotHardware.Alliance.RED) || (name.startsWith("B") && RobotHardware.getAlliance() == RobotHardware.Alliance.BLUE)) {
                        ready = true;
                    }
                }
            }
        }
    }

    public boolean readyToShoot() {
        return ready;
    }
    //todo this method returns wheher 2+ april tags visible i.e cell is up, so ready to shoot
}