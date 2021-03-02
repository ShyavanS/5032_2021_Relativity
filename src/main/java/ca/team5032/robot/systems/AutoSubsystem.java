package ca.team5032.robot.systems;

import java.util.Timer;
import java.util.TimerTask;

import ca.team5032.robot.OI;
import ca.team5032.robot.Robot;
import ca.team5032.robot.framework.subsystem.Subsystem;
import ca.team5032.robot.sensor.limelight.LimeLight;
import ca.team5032.robot.sensor.limelight.LimeLightTarget;
import ca.team5032.robot.utils.NetworkTableUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSubsystem extends Subsystem {

    private boolean targetFound;
    // private boolean autoFeed = true;
    private final int powerPortHeight = 10;
    private final int limeLightHeight = 3;
    private final int limeLightAngle = 30;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private double ty = NetworkTableUtils.getDouble(table, "ty");
    // private Timer driver = new Timer();
    // private Timer spooler = new Timer();
    private Timer auto = new Timer();

    public AutoSubsystem(Robot robot, boolean defaultEnabled) {
        super(robot, "AutoSubsystem", defaultEnabled);
        this.targetFound = false;
    }

    @Override
    public void autoInit() {
        getRobot().getLimeLight().startTarget(LimeLight.Pipeline.REFLECTIVE_TAPE);
        getRobot().getShooterSubsystem().loopReset();
    }

    @Override
    public void autoPeriodic() {
        // New auto system
        masterAlign();
        auto.schedule(new TimerTask() {
            @Override
            public void run() {
                getRobot().getLimeLight().finishTarget();
                getRobot().getDriveSubsystem().stop();
                getRobot().getIndexingSubsystem().stop();
                getRobot().getShooterSubsystem().stop();
            }
        }, 14000);
    // Buddy Auto w/ 5024 @ Humber
    //     //drive
    //     if (!autoFeed) {
    //         getRobot().getDriveSubsystem().drive(0.0, 0.6);
    //     driver.schedule(new TimerTask(){
        
    //         @Override
    //         public void run() {
    //             getRobot().getDriveSubsystem().drive(0.0, 0.0);
                
    //         }
    //     }, 1000);
    //     } else {
    //         //feed
    //         driver.schedule(new TimerTask(){
            
    //             @Override
    //             public void run() {
    //                 getRobot().getIntakeSubsystem().accept(-0.55);
    //                 getRobot().getIndexingSubsystem().channelBottom.set(0.55);
    //             }
    //         }, 3000);
    //         driver.schedule(new TimerTask(){
            
    //             @Override
    //             public void run() {
    //                 getRobot().getIntakeSubsystem().accept(0.0);
    //                 getRobot().getIndexingSubsystem().channelBottom.set(0.0);
    //             }
    //         }, 10000);
    //     }

        // if (Math.abs(getRobot().getDriveSubsystem().getRightDistance()) < 120) {
        //     getRobot().getDriveSubsystem().drive(0.0, -0.75);
        // }
        // else {
        //     getRobot().getDriveSubsystem().drive(0, 0);
        //     getRobot().getShooterSubsystem().shoot();
        //     spooler.schedule(new TimerTask() {
            
        //         @Override
        //         public void run() {
        //             getRobot().getIndexingSubsystem().channelBottom.set(-0.7);
        //         }
        //     }, 4000);
        //     spooler.schedule(new TimerTask(){
            
        //         @Override
        //         public void run() {
        //             getRobot().getIndexingSubsystem().channelBottom.set(0.0);
        //             getRobot().getShooterSubsystem().stop();
        //         }
        //     }, 12000);
        // }
    }

    // @Override
    // public void setDashboard() {
    //     SmartDashboard.putNumber("Right Distance", getRobot().getDriveSubsystem().getRightDistance());
    // }

    public double quadraticSolve(double a, double b, double c, boolean positive) {
        double root = 0;
        if (positive) {
            try {
                root = (-b + Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
            } catch(Exception e) {
                ;
            }
        } else {
            try {
                root = (-b - Math.sqrt(Math.pow(b, 2) - 4 * a * c)) / (2 * a);
            } catch(Exception e) {
                ;
            }
        }
        return root;
    }

    public void masterAlign() {
        boolean done = align();
        if (done) {
            double distanceToPort = getRobot().getLimeLight().getDistance();
            double launchVelocity = Math.sqrt((OI.GRAVITY * Math.pow(distanceToPort, 2)) / ((2 * Math.pow(Math.cos(OI.LIMELIGHT_SHOOTER_ANGLE), 2)) * (OI.LIMELIGHT_HEIGHT_OF_SHOOTER + distanceToPort * Math.tan(OI.LIMELIGHT_SHOOTER_ANGLE) - OI.LIMELIGHT_HEIGHT_OF_INNER_PORT)));
            if (launchVelocity > OI.MAX_SPEED) {
                launchVelocity = OI.MAX_SPEED;
                double a = -OI.GRAVITY / (2 * Math.pow(launchVelocity, 2) * Math.pow(Math.cos(OI.LIMELIGHT_SHOOTER_ANGLE), 2));
                double b = Math.tan(OI.LIMELIGHT_SHOOTER_ANGLE);
                double c = OI.LIMELIGHT_HEIGHT_OF_INNER_PORT;
                distanceToPort = Math.min(quadraticSolve(a, b, c, true), quadraticSolve(a, b, c, false));
                if (distanceToPort == getRobot().getLimeLight().getDistance()) {
                    getRobot().getDriveSubsystem().stop();
                    getRobot().getShooterSubsystem().loopSetNext(launchVelocity / OI.LAUNCHER_RADIUS); // C
                    if (getRobot().getShooterSubsystem().launchVelocity() >= launchVelocity) {
                        getRobot().getIndexingSubsystem().channelBottom.set(-0.7);
                    }
                    getRobot().getShooterSubsystem().loopCorrect(); // C
                    getRobot().getShooterSubsystem().loopPredict(); // C
                    double nextVoltage = getRobot().getShooterSubsystem().loopVoltage(); // C
                    getRobot().getShooterSubsystem().shootVoltage(nextVoltage); // C
                } else {
                    getRobot().getDriveSubsystem().tankDrive(0.2, 0.2); // Reduce speed if necessary
                }
            } else if (launchVelocity < OI.MIN_SPEED) {
                launchVelocity = OI.MIN_SPEED;
                double a = -OI.GRAVITY / (2 * Math.pow(launchVelocity, 2) * Math.pow(Math.cos(OI.LIMELIGHT_SHOOTER_ANGLE), 2));
                double b = Math.tan(OI.LIMELIGHT_SHOOTER_ANGLE);
                double c = OI.LIMELIGHT_HEIGHT_OF_INNER_PORT;
                distanceToPort = Math.min(quadraticSolve(a, b, c, true), quadraticSolve(a, b, c, false));
                if (distanceToPort == getRobot().getLimeLight().getDistance()) {
                    getRobot().getDriveSubsystem().stop();
                    getRobot().getShooterSubsystem().loopSetNext(launchVelocity / OI.LAUNCHER_RADIUS); // C
                    if (getRobot().getShooterSubsystem().launchVelocity() >= launchVelocity) {
                        getRobot().getIndexingSubsystem().channelBottom.set(-0.7);
                    }
                    getRobot().getShooterSubsystem().loopCorrect(); // C
                    getRobot().getShooterSubsystem().loopPredict(); // C
                    double nextVoltage = getRobot().getShooterSubsystem().loopVoltage(); // C
                    getRobot().getShooterSubsystem().shootVoltage(nextVoltage); // C
                } else {
                    getRobot().getDriveSubsystem().tankDrive(-0.2, -0.2); // Reduce speed if necessary
                }
            } else {
                getRobot().getDriveSubsystem().stop();
                getRobot().getShooterSubsystem().loopSetNext(launchVelocity / OI.LAUNCHER_RADIUS); // C
                if (getRobot().getShooterSubsystem().launchVelocity() >= launchVelocity) {
                    getRobot().getIndexingSubsystem().channelBottom.set(-0.7);
                }
                getRobot().getShooterSubsystem().loopCorrect(); // C
                getRobot().getShooterSubsystem().loopPredict(); // C
                double nextVoltage = getRobot().getShooterSubsystem().loopVoltage(); // C
                getRobot().getShooterSubsystem().shootVoltage(nextVoltage); // C
            }
        }
    }

    @Override
    public void tick() {
        // ty = NetworkTableUtils.getDouble(table, "ty");
        // getRobot().getDriveSubsystem().tankDrive(0.4, -0.4);
        // double depth = ((powerPortHeight - limeLightHeight) / Math.tan(limeLightAngle + ty));
        // double angleShooterValue = (Math.pow(2.3429 * depth, 2) - 19.39 * depth + 87.114);
        // if (angle.getVoltage() < 0.1) {
        //     getRobot().getShooterSubsystem().rotate(0.25);
        // } else if (angle.getVoltage() > 0.2) { // deadzone
        //     getRobot().getShooterSubsystem().rotate(-0.25);
        // } else {
        //     getRobot().getShooterSubsystem().rotate(0.0);
            
        // }
        // if (getRobot().getLimeLight().hasTarget()) {
        // }
    }

    public boolean align() {
        LimeLightTarget target = getRobot().getLimeLight().getTarget();
        if (target == null) return false;

        Vector2d offset = target.getOffset();
        if (offset.x == 0) return true;

        double slowingRadius = 16;
        double maxVelocity = 0.485;

        double xOffset = offset.x;
        double xTarget = 0.0;
        double desired = xTarget - xOffset;
        double distance = Math.abs(desired);
        
        double depth = ((powerPortHeight - limeLightHeight) / Math.tan(limeLightAngle + ty));
        double angleShooterValue = (Math.pow(2.3429 * depth, 2) - 19.39 * depth + 87.114);
        // if (angle.getVoltage() < 0.5) {
        //     //getRobot().getShooterSubsystem().rotate(0.25);
        // } else if (angle.getVoltage() > 0.2) { // deadzone
        //     //getRobot().getShooterSubsystem().rotate(-0.25);
        // } else {
        //     //getRobot().getShooterSubsystem().rotate(0.0);
            
        // }
        // if (getRobot().getLimeLight().hasTarget()) {
        // }

        desired = desired > 0 ? -1 : 1;
        if (distance < slowingRadius) {
            desired *= maxVelocity * (distance / slowingRadius);
        } else {
            desired *= maxVelocity;
        }

        if (distance > 1) {
            desired = desired > 0 ?
                    Math.max(desired, 0.4) :
                    Math.min(desired, -0.4);
        } else {
            getRobot().getDriveSubsystem().stop();
            return true;
        }

        SmartDashboard.putNumber("Desired Velocity", desired);

        getRobot().getDriveSubsystem().tankDrive(desired, desired);
        return false;
    }
}
