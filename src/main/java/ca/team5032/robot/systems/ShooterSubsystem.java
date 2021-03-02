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

import edu.wpi.first.wpilibj.controller.LinearQuadraticRegulator;
import edu.wpi.first.wpilibj.estimator.KalmanFilter;
// import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.LinearSystemLoop;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;
import edu.wpi.first.wpiutil.math.numbers.N1;

public class ShooterSubsystem extends Subsystem {

    // TODO: better names based on function
    private WPI_TalonSRX speedController;

    // Initialize stuff for special PID using flywheel inertia to derive gains without physical robot
    // TODO change everything marked with "C" to reflect physical robot using VelocityClosedLoop method from TalonSRX rather than WPILib LinearSystemLoop
    private final LinearSystem<N1, N1, N1> flywheelPlant; // C
    private final KalmanFilter<N1, N1, N1> observerFilter; // C
    private final LinearQuadraticRegulator<N1, N1, N1> controller; // C
    private final LinearSystemLoop<N1, N1, N1> loop; // C
    // private WPI_VictorSPX hoodController;

    // private Counter encoder;
    // private int rpm;
    // private int target = 100;
    // private double speed = 1.0;

    public ShooterSubsystem(Robot robot, boolean defaultEnabled) {
        super(robot, "ShooterSubsystem", defaultEnabled);
        this.speedController = new WPI_TalonSRX(6);
        // Stuff for PID again
        this.flywheelPlant = LinearSystemId.createFlywheelSystem( // C
            DCMotor.getAndymarkRs775_125(1),
            OI.FLYWHEEL_MOMENT_OF_INERTIA,
            OI.FLYWHEEL_GEARING); 
        this.observerFilter = new KalmanFilter<>( // C
            Nat.N1(),
            Nat.N1(),
            flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think the model is
            VecBuilder.fill(0.01), // How accurate we think the encoder is
            0.020);
        this.controller = new LinearQuadraticRegulator<>( // C
            flywheelPlant,
            VecBuilder.fill(6.0), // Velocity error tolerance, how much speed is allowed to be off in rad/s
            VecBuilder.fill(12.0), // Voltage tolerance, start at battery V, or 12V
            0.020); // Nominal time between loops
        // initialize loop for monitoring speed
        this.loop = new LinearSystemLoop<>( // C
            flywheelPlant,
            controller,
            observerFilter,
            12.0,
            0.020);
        // this.hoodController = new WPI_VictorSPX(OI.SYSTEM_SHOOTER_HOOD_VICTOR_ID);
        // this.encoder = new Counter(0);
        // this.angle = new AnalogInput(0);
        // this.encoder.setDistancePerPulse(1);
        initQuadrature();
        this.speedController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, OI.TIMEOUT);
    }

    public void initQuadrature() {
		/* get the absolute pulse width position */
		int pulseWidth = this.speedController.getSensorCollection().getPulseWidthPosition();

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
		this.speedController.getSensorCollection().setQuadraturePosition(pulseWidth, OI.TIMEOUT);
	}

    // public AnalogInput getAngle() {
    //     return this.angle;
    // }

    public double launchVelocity() {
        return (this.speedController.getSensorCollection().getQuadratureVelocity() / OI.MAG_UNITS_PER_ROTATION * OI.MAG_SPEED_TIME * 2 * Math.PI) * OI.LAUNCHER_RADIUS; // To get m/s tangential from angular velocity
    }

    // For getting encoder values to pass to PID solution
    public double angularVelocity() { // C
        return this.speedController.getSensorCollection().getQuadratureVelocity() / OI.MAG_UNITS_PER_ROTATION * OI.MAG_SPEED_TIME * 2 * Math.PI;
    }

    // For resetting loop for PID solution
    public void loopReset() { // C
        this.loop.reset(VecBuilder.fill(angularVelocity()));
    }

    // For correcting loop for PID solution
    public void loopCorrect() { // C
        this.loop.correct(VecBuilder.fill(angularVelocity()));
    }

    // For predicting loop for PID solution
    public void loopPredict() { // C
        this.loop.predict(0.020);
    }

    // Getting motor voltage
    public double loopVoltage() { // C
        return this.loop.getU(0);
    }

    // Set next for loop
    public void loopSetNext(double velocity) { // C
        this.loop.setNextR(VecBuilder.fill(velocity));
    }

    // Set motor voltage to shoot
    public void shootVoltage(double volts) { // C
        this.speedController.setVoltage(volts);
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
        this.speedController.set(-1);
    }

    public void stop() {
        // this.hoodController.set(0.0);
        this.speedController.set(0.0);
        loopSetNext(0.0); // C
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
