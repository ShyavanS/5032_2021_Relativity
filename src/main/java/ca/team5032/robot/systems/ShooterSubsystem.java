package ca.team5032.robot.systems;

import ca.team5032.robot.OI;
import ca.team5032.robot.Robot;
import ca.team5032.robot.framework.subsystem.Subsystem;
import ca.team5032.robot.tasks.Task;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.*;

// import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends Subsystem {

    // TODO: better names based on function
    private WPI_TalonSRX speedController;
    // private WPI_VictorSPX hoodController;

    // private Counter encoder;
    // private int rpm;
    // private int target = 100;
    // private double speed = 1.0;

    public ShooterSubsystem(Robot robot, boolean defaultEnabled) {
        super(robot, "ShooterSubsystem", defaultEnabled);
        this.speedController = new WPI_TalonSRX(6);
        // this.hoodController = new WPI_VictorSPX(OI.SYSTEM_SHOOTER_HOOD_VICTOR_ID);
        // this.encoder = new Counter(0);
        // this.angle = new AnalogInput(0);
        // this.encoder.setDistancePerPulse(1);
        initQuadrature();
	    this.speedController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, OI.TIMEOUT);
    }

    public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = speedController.getSensorCollection().getPulseWidthPosition();

		/**
		 * If there is a discontinuity in our measured range, subtract one half
		 * rotation to remove it
		 */
		if (OI.DISCONTINUITY_PRESENT) {

			/* Calculate the center */
			int newCenter;
			newCenter = (OI.BOOK_END_0 + OI.BOOK_END_1) / 2;
			newCenter &= 0xFFF;

			/**
			 * Apply the offset so the discontinuity is in the unused portion of
			 * the sensor
			 */
			pulseWidth -= newCenter;
		}

		/**
		 * Mask out the bottom 12 bits to normalize to [0,4095],
		 * or in other words, to stay within [0,360) degrees 
		 */
		pulseWidth = pulseWidth & 0xFFF;

		/* Update Quadrature position */
		speedController.getSensorCollection().setQuadraturePosition(pulseWidth, OI.TIMEOUT);
	}

    // public AnalogInput getAngle() {
    //     return this.angle;
    // }

    public double launchVelocity() {
        return speedController.getSensorCollection().getQuadratureVelocity() / OI.MAG_UNITS_PER_ROTATION * OI.LAUNCHER_CIRCUMFERENCE * OI.MAG_SPEED_TIME;
    }

    @Override
    public void tick() {

    }

    @Override
    public void kill() {
        this.speedController.stopMotor();
        // this.hoodController.stopMotor();
    }

    @Override
    public void setDashboard() {
        SmartDashboard.putNumber("Encoder speed (m/s)", launchVelocity());
        // SmartDashboard.putNumber("POT", angle.getVoltage());
        // SmartDashboard.putBoolean("Top Speed", speed);
        SmartDashboard.putNumber("Distance to target (m)", getRobot().getLimeLight().getDistance());
    }

    // public void resetEncoder() {
    //     this.encoder.reset();
    // }

    public void shoot() {
        // if (encoder.getRate() <= target * 0.6) {
        //     speedController.set(speed);
        //     // speed = false;
        // } else if (encoder.getRate() <= target * 0.7 && encoder.getRate() > target * 0.6) {
        //     speedController.set(speed * 0.9);
        //     // speed = false;
        // } else if (encoder.getRate() <= target * 0.8 && encoder.getRate() > target * 0.7) {
        //     speedController.set(speed * 0.8);
        //     // speed = false;
        // } else if (encoder.getRate() <= target * 0.9 && encoder.getRate() > target * 0.8) {
        //     speedController.set(speed * 0.7);
        //     // speed = false;
        // } else {
        //     speedController.set(speed * 0.6);
        //     // speed = true;
        //     getRobot().getIndexingSubsystem().channelBottom.set(-0.7);
        // }
        speedController.set(-1);
    }

    public void stop() {
        // this.hoodController.set(0.0);
        this.speedController.set(0.0);
        getRobot().getIndexingSubsystem().stop();
    }

    // public void rotate(double amount) {
    //     this.hoodController.set(amount);
    // }

    // private void pulse(double strength, int ms) {
    //     new Task() {
    //         @Override
    //         public void start() {
    //             speedController.set(strength);
    //         }

    //         @Override
    //         public void finish() {
    //             speedController.set(0.0);
    //         }
    //     }.runFor(ms).delay(0).rate(0).then(() -> {}).run();
    // }
}
