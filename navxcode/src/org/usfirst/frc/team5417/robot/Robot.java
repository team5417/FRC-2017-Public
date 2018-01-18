package org.usfirst.frc.team5417.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.usfirst.frc.team5417.cv2017.ChannelRange;
import org.usfirst.frc.team5417.cv2017.ComputerVision2017;
import org.usfirst.frc.team5417.cv2017.ComputerVisionResult;
import org.usfirst.frc.team5417.cv2017.Stopwatch;
import org.usfirst.frc.team5417.cv2017.opencvops.BooleanMatrix;

import com.ctre.MotorControl.CANTalon;
//import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

/**
 * This is a demo program showing the use of the RobotDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 *
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SampleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * WARNING: While it may look like a good choice to use for your code if you're
 * inexperienced, don't. Unless you know what you are doing, complex code will
 * be much more difficult under this system. Use IterativeRobot or Command-Based
 * instead if you're new.
 */
public class Robot extends SampleRobot {
	private CameraServer cameraServer = null;
	private CameraReader cameraReader = null;
	private UsbCamera gearCamera = null;
	private UsbCamera driverCamera = null;
	private CvSource computerVisionOutputStream = null;
	ChannelRange hueRange = new ChannelRange(155, 170);
	ChannelRange satRange = new ChannelRange(.5, .7);
	ChannelRange valRange = new ChannelRange(230, 255);
	private PowerDistributionPanel PDP = new PowerDistributionPanel(1);

	List<BooleanMatrix> horizontalTemplates = new ArrayList<BooleanMatrix>();
	List<BooleanMatrix> verticalTemplates = new ArrayList<BooleanMatrix>();

	int dilateErodeKernelSize = 5;
	int removeGroupsSmallerThan = 8;
	int numberOfScaleFactors = 20;
	double minimumTemplateMatchPercentage = 0.7;

	// dummy PID that wants to approach 4 feet
	// private PID distancePID = new PID(1, 20000, 0, PIDSourceType.kRate, 4);

	double[] gearLookUpTable = { 250, // 0 feet
			104, // 1 foot
			53.7, 37.5, 28, 22.5, 19, 16.4, 14.2, 12.4, 11.6, 10.6, // 11 feet
			0 // BEYOND
	};

	// XboxController stick = new XboxController(0);
	// XboxController stick1 = new XboxController(1);
	DigitalInput limitSwitchGround = new DigitalInput(0);
	DigitalInput limitSwitchRobot = new DigitalInput(1);
	DigitalInput limitSwitchGear = new DigitalInput(2);
	DigitalOutput statusLights = new DigitalOutput(3);
	DigitalOutput leftLights = new DigitalOutput(4);
	DigitalOutput rightLights = new DigitalOutput(5);
	boolean isGearLoaded = false;
	int timesUpdated = 0;
	CANTalon leftFrontMotor = new CANTalon(2);
	CANTalon leftRearMotor = new CANTalon(3);
	CANTalon rightFrontMotor = new CANTalon(4);-
	CANTalon rightRearMotor = new CANTalon(5);
	CANTalon leftShooterMotor = new CANTalon(6);
	CANTalon gearIntakeMotor = new CANTalon(7);
	CANTalon gearLoaderMotor = new CANTalon(8);
	CANTalon gearIntakeMotor2 = new CANTalon(11);
	CANTalon climberMotor1 = new CANTalon(9);
	CANTalon climberMotor2 = new CANTalon(10);
	Solenoid gearSolenoid = new Solenoid(0, 1);
	// DoubleSolenoid gearSolenoid = new DoubleSolenoid(0, 1);
	Solenoid shooterSolenoid1 = new Solenoid(2);
	Solenoid shooterSolenoid2 = new Solenoid(3);
	Compressor compressor = new Compressor(0);
	GearShift driveSystem;
	Solenoid shiftSolenoid1 = new Solenoid(0);
	RobotDrive myRobot = new RobotDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
	XBoxController driverStick = new XBoxController(new Joystick(0));
	XBoxController manipulatorStick = new XBoxController(new Joystick(1));
	AHRS ahrs;

	boolean centered = false;
	double leftY;
	double rightY;
	PIDController turnController;
	PIDController offsetFromCenterController;
	UpdatablePIDSource offsetFromCenter;
	UpdatablePIDOutput offsetFromCenterOutput;
	UpdatablePIDOutput turnOutput;
	static final double kP = .1;
	static final double kI = .001;
	static final double kD = 0.00;
	static final double kF = 0.00;
	int targetColumn;
	Stopwatch outOfSightStopwatch;
	double defaultLeftY = 0;
	double defaultRightY = 0;
	String autoSelected = "";
	double timeSinceLastLightUpdate = 0;

	double rightSpeed = .73;
	double forwardTime = 1.1;
	double finalSpeed = .55;
	double turnSpeed = .45;
	double leftAutoForwardTime = 1.2;
	double leftAutoFinalSpeed = .8;
	double hardTurnSpeed = .4;
	Servo lightServo = new Servo(0);
	double PWMdelayLeft = .001;
	double PWMdelayRight = .001;
	static final double kToleranceDegrees = 2;
	double rotateToAngleRate = 0;
	double goToCenterRate = 0;
	Stopwatch elapsedTimeWithNoDriveInputs = null;
	Stopwatch elapsedTimeSinceLastGearShift = Stopwatch.startNew();
	double secondCount = 0;

	final String defaultAuto = "Default Autonomous";
	final String gearTestAuto = "Left auto";
	final String otherAuto = "other auto";
	final String centerAuto = "Center auto";
	final String driveStraight = "Drive straight";
	final String rightAuto = "Right auto";
	SendableChooser<String> chooser = new SendableChooser<>();

	public Robot() {
		myRobot.setExpiration(0.5);

		// horizontalTemplates.add(new BooleanMatrix(40, 150, true));
		// horizontalTemplates.add(new BooleanMatrix(20, 150, true));
		// horizontalTemplates.add(new BooleanMatrix(20, 75, true));
		// horizontalTemplates.add(new BooleanMatrix(10, 75, true));

		verticalTemplates.add(new BooleanMatrix(150, 60, true));
		verticalTemplates.add(new BooleanMatrix(75, 30, true));
	}

	@Override
	public void robotInit() {
		statusLights.set(false);
		if (computerVisionOutputStream != null) {
			computerVisionOutputStream.free();
			computerVisionOutputStream = null;
		}

		if (cameraReader != null) {
			cameraReader.free();
			cameraReader = null;
		}

		if (gearCamera != null) {
			gearCamera.free();
			gearCamera = null;
		}
		cameraServer = CameraServer.getInstance();
		gearCamera = cameraServer.startAutomaticCapture();
		driverCamera = cameraServer.startAutomaticCapture();
		computerVisionOutputStream = cameraServer.putVideo("CV2017", 160, 120);

		// chooser.addDefault("Default Auto", defaultAuto);
		// chooser.addObject("My Auto", gearTestAuto);
		// SmartDashboard.putData("Auto modes", chooser);
		compressor.start();
		driveSystem = new GearShift(shiftSolenoid1);
		// leftShooterMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		// these are decent defaults
		// static final double kP = .1;
		// static final double kI = .001;
		// static final double kD = 0.00;
		// static final double kF = 0.00;

		ahrs = new AHRS(I2C.Port.kOnboard);
		turnOutput = new UpdatablePIDOutput();
		turnController = new PIDController(0.01, 0.001, 0.001, 0.00, ahrs, turnOutput);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.2, 0.2);
		turnController.setAbsoluteTolerance(0);
		turnController.setContinuous(true);

		offsetFromCenter = new UpdatablePIDSource();
		offsetFromCenterOutput = new UpdatablePIDOutput();
		offsetFromCenterController = new PIDController(0.01, 0.001, 0.001, 0.00, offsetFromCenter,
				offsetFromCenterOutput);
		offsetFromCenterController.setInputRange(-80.0f, 80.0f);
		offsetFromCenterController.setOutputRange(-0.5, 0.5);
		offsetFromCenterController.setAbsoluteTolerance(0);
		offsetFromCenterController.setContinuous(true);

		// leftShooterMotor.setMotionMagicCruiseVelocity(motMagicCruiseVeloc);
	}

	// public void autonomousInit(){
	// automonousCommand = (Command) chooser.getSelected();
	// automonousCommand.start();
	// }
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {
		// String autoSelected = chooser.getSelected();
		// String autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		compressor.stop();
		if (SmartDashboard.getBoolean("DB/Button 3", false)) {
			cameraReader = new CameraReader(cameraServer.getVideo("USB Camera 0"));
		} else {
			cameraReader = new CameraReader(cameraServer.getVideo("USB Camera 1"));
		}

		if (SmartDashboard.getBoolean("DB/Button 0", false)) {
			autoSelected = "Center auto";
		} else if (SmartDashboard.getBoolean("DB/Button 2", false)) {
			autoSelected = "Right auto";
		} else if (SmartDashboard.getBoolean("DB/Button 1", false)) {
			autoSelected = "Left auto";
		}
		SmartDashboard.putString("DB/String 0", autoSelected);
		double voltage = SmartDashboard.getNumber("DB/Slider 0", 12.6);
		SmartDashboard.putString("DB/String 6", "" + voltage);

		if (voltage < 12.55) {
			rightSpeed = .73;
			forwardTime = 1.1; // original: 1.1
			finalSpeed = .6;
			turnSpeed = .46;
			leftAutoForwardTime = 1.6;
			hardTurnSpeed = .50;
		} else if (voltage < 12.65) {
			rightSpeed = .72;
			forwardTime = 1.35;
			leftAutoForwardTime = 1.5;
			finalSpeed = .74;
			turnSpeed = .48;
			hardTurnSpeed = .45;
		} else if (voltage < 12.75) {
			rightSpeed = .73;
			forwardTime = 1.3;
			finalSpeed = .7;
			leftAutoForwardTime = 1.4;
			turnSpeed = .44;
			leftAutoFinalSpeed = .7;
			hardTurnSpeed = .4;
		} else if (voltage < 12.85) {
			rightSpeed = .74;
			forwardTime = 1.2;
			finalSpeed = .55;
			leftAutoForwardTime = 1.3;
			turnSpeed = .42;
			hardTurnSpeed = .4;
		} else {
			rightSpeed = .72;
			forwardTime = .8;
			finalSpeed = .5;
			turnSpeed = .42;
			hardTurnSpeed = .4;
		}

		SmartDashboard.putString("DB/String 7", "" + rightSpeed);
		switch (autoSelected) {
		case gearTestAuto:
			lightServo.set(0);
			compressor.stop();
			myRobot.setSafetyEnabled(false);
			turnController.enable();
			ahrs.reset();
			driveSystem.gearShiftHigh(0, 0);
			Timer.delay(.1);
			double time = 0;
			Stopwatch driveStraightStopwatch = Stopwatch.startNew();
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(.8, .8);
			Timer.delay(leftAutoForwardTime);
			SmartDashboard.putString("Auto status", "drive forward end");
			driveStraightStopwatch.stop();
			Stopwatch turnStopwatch = Stopwatch.startNew();
			boolean aligned = false;
			time = 0;
			myRobot.tankDrive(hardTurnSpeed, -hardTurnSpeed);
			Timer.delay(.8);
			myRobot.tankDrive(0, 0);
			Timer.delay(.1);
			while (!aligned & time < 7) {
				ComputerVisionResult cvResult = doComputerVision(this.cameraReader, this.verticalTemplates,
						this.gearLookUpTable);
				SmartDashboard.putString("DoCV Target Point",
						"(" + cvResult.targetPoint.getX() + ", " + cvResult.targetPoint.getY() + ")");
				if (cvResult.targetPoint.getX() == -1) {
					myRobot.tankDrive(.6, -.6);
					SmartDashboard.putString("Auto status", "vision loop: point out of sight");
					Timer.delay(.1);
					myRobot.tankDrive(0, 0);
					Timer.delay(.1);
				} else if (0 < cvResult.targetPoint.getX() && cvResult.targetPoint.getX() < 77) {
					myRobot.tankDrive(-turnSpeed, turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");

				} else if (cvResult.targetPoint.getX() > 83 && cvResult.targetPoint.getX() < 160) {
					myRobot.tankDrive(turnSpeed, -turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");
				} else if (cvResult.targetPoint.getX() >= 77 && cvResult.targetPoint.getX() <= 83) {
					aligned = true;
					myRobot.tankDrive(0, 0);
					SmartDashboard.putString("Auto status", "vision loop: point in center");
				}
				time = turnStopwatch.getTotalSeconds();
			}
			SmartDashboard.putString("Auto status", "vision loop end");
			turnStopwatch.stop();
			Timer.delay(.5);
			time = 0;
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(leftAutoFinalSpeed + .02, leftAutoFinalSpeed);
			Timer.delay(1.2);
			myRobot.tankDrive(0, 0);
			compressor.start();
			lightServo.set(.5);

			break;
		case defaultAuto:
		default:
			myRobot.setSafetyEnabled(false);
			break;
		case rightAuto:
			lightServo.set(0);
			compressor.stop();
			myRobot.setSafetyEnabled(false);
			turnController.enable();
			ahrs.reset();
			driveSystem.gearShiftHigh(0, 0);
			Timer.delay(.1);
			time = 0;
			driveStraightStopwatch = Stopwatch.startNew();
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(.8, .8);
			Timer.delay(leftAutoForwardTime);
			SmartDashboard.putString("Auto status", "drive forward end");
			driveStraightStopwatch.stop();
			turnStopwatch = Stopwatch.startNew();
			aligned = false;
			time = 0;
			myRobot.tankDrive(-hardTurnSpeed, hardTurnSpeed);
			Timer.delay(.8);
			myRobot.tankDrive(0, 0);
			Timer.delay(.1);
			while (!aligned & time < 5) {
				ComputerVisionResult cvResult = doComputerVision(this.cameraReader, this.verticalTemplates,
						this.gearLookUpTable);
				SmartDashboard.putString("DoCV Target Point",
						"(" + cvResult.targetPoint.getX() + ", " + cvResult.targetPoint.getY() + ")");
				if (cvResult.targetPoint.getX() == -1) {
					myRobot.tankDrive(-.6, .6);
					SmartDashboard.putString("Auto status", "vision loop: point out of sight");
					Timer.delay(.1);
					myRobot.tankDrive(0, 0);
					Timer.delay(.1);
				} else if (0 < cvResult.targetPoint.getX() && cvResult.targetPoint.getX() < 77) {
					myRobot.tankDrive(-turnSpeed, turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");

				} else if (cvResult.targetPoint.getX() > 83 && cvResult.targetPoint.getX() < 160) {
					myRobot.tankDrive(turnSpeed, -turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");
				} else if (cvResult.targetPoint.getX() >= 77 && cvResult.targetPoint.getX() <= 83) {
					aligned = true;
					myRobot.tankDrive(0, 0);
					SmartDashboard.putString("Auto status", "vision loop: point in center");
				}
				time = turnStopwatch.getTotalSeconds();
			}
			SmartDashboard.putString("Auto status", "vision loop end");
			turnStopwatch.stop();
			Timer.delay(.5);
			time = 0;
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(leftAutoFinalSpeed + .02, leftAutoFinalSpeed);
			Timer.delay(1.2);
			myRobot.tankDrive(0, 0);
			compressor.start();
			lightServo.set(.5);

			break;
		case centerAuto:
			lightServo.set(0);
			SmartDashboard.putString("voltage: ", "" + voltage);
			SmartDashboard.putString("rightSpeed: ", "" + rightSpeed);
			compressor.stop();
			myRobot.setSafetyEnabled(false);
			turnController.enable();
			ahrs.reset();
			driveSystem.gearShiftHigh(0, 0);
			Timer.delay(1);
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(.7, rightSpeed);
			SmartDashboard.putString("Auto status", "set forward speed");
			Timer.delay(forwardTime);
			SmartDashboard.putString("Auto status", "set forward time");
			myRobot.tankDrive(0, 0);
			Timer.delay(.5);
			gearIntakeMotor.set(.4);
			gearIntakeMotor2.set(-.4);
			Timer.delay(.05);
			gearIntakeMotor.set(0);
			gearIntakeMotor2.set(0);
			aligned = false;
			turnStopwatch = Stopwatch.startNew();
			time = 0;
			SmartDashboard.putString("Auto status", "beginning vision");
			while (!aligned & time < 5) {
				ComputerVisionResult cvResult = doComputerVision(this.cameraReader, this.verticalTemplates,
						this.gearLookUpTable);
				SmartDashboard.putString("DoCV Target Point",
						"(" + cvResult.targetPoint.getX() + ", " + cvResult.targetPoint.getY() + ")");
				if (cvResult.targetPoint.getX() == -1) {
					myRobot.tankDrive(0, 0);
					SmartDashboard.putString("Auto status", "vision loop: point out of sight");
				} else if (0 < cvResult.targetPoint.getX() && cvResult.targetPoint.getX() < 77) {
					myRobot.tankDrive(-turnSpeed, turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");

				} else if (cvResult.targetPoint.getX() > 83 && cvResult.targetPoint.getX() < 160) {
					myRobot.tankDrive(turnSpeed, -turnSpeed);
					SmartDashboard.putString("Auto status", "vision loop: point almost in center");
				} else if (cvResult.targetPoint.getX() >= 77 && cvResult.targetPoint.getX() <= 83) {
					aligned = true;
					myRobot.tankDrive(0, 0);
					SmartDashboard.putString("Auto status", "vision loop: point in center");
				}
				time = turnStopwatch.getTotalSeconds();
			}
			myRobot.tankDrive(.1, .1);
			Timer.delay(.5);
			myRobot.tankDrive(finalSpeed + .02, finalSpeed);
			Timer.delay(1.2);
			myRobot.tankDrive(0, 0);
			compressor.start();
			lightServo.set(1);
			break;
		case driveStraight:
			myRobot.setSafetyEnabled(false);
			turnController.enable();
			ahrs.reset();
			driveSystem.gearShiftLow(0, 0);
			Timer.delay(1);
			rotateToAngleRate = 0;
			turnController.setSetpoint(ahrs.getAngle());
			turnController.setOutputRange(-0.5, 0.5);
			time = 0;
			Stopwatch driveStraightOnlyStopwatch = Stopwatch.startNew();
			while (time < 2.2) {
				leftY = .6 + rotateToAngleRate;
				rightY = .6 - rotateToAngleRate;
				myRobot.tankDrive(leftY, rightY);
				time = driveStraightOnlyStopwatch.getTotalSeconds();
				updateFromTurnController(turnOutput.getValue());
			}
			myRobot.tankDrive(0, 0);

			turnController.disable();

		}
	}
	/*
	 * Runs the motors with tank steering.
	 */
	/*
	 * @Override
	 */
	/*
	 * protected void disabled() { if (isInitialized) { doOneTimeShutdown();
	 * isInitialized = false; } }
	 */

	public double getJoystickValueWithDeadZone(double value) {
		double deadZone = 0.2;

		if (isCloseToZero(value, deadZone)) {
			value = 0;
		}
		return value;
	}

	public ComputerVisionResult doComputerVision(CameraReader cameraReader, List<BooleanMatrix> templatesToUse,
			double[] lookUpTableToUse) {
		if (templatesToUse == null) {
			System.out.println("templatesToUse is null");
		}
		if (lookUpTableToUse == null) {
			System.out.println("lookUpTableToUse is null");
		}

		ComputerVision2017 gearCV2017 = new ComputerVision2017();
		ComputerVisionResult cvResult = gearCV2017.DoComputerVision(cameraReader, 160, hueRange, satRange, valRange,
				dilateErodeKernelSize, removeGroupsSmallerThan, numberOfScaleFactors, minimumTemplateMatchPercentage,
				templatesToUse, lookUpTableToUse);

		SmartDashboard.putNumber("DoCV distance", cvResult.distance);
		SmartDashboard.putString("DoCV Target Point",
				"(" + cvResult.targetPoint.getX() + ", " + cvResult.targetPoint.getY() + ")");

		if (cvResult.visionResult != null) {
			Mat euc3 = new Mat();
			cvResult.visionResult.assignTo(euc3, CvType.CV_8UC3);
			computerVisionOutputStream.putFrame(euc3);

			cvResult.visionResult.release();
			euc3.release();
		}
		return cvResult;
	}

	private boolean isCloseToZero(double value, double tolerance) {
		tolerance = Math.abs(tolerance);
		return value > -tolerance && value < tolerance;
	}

	@Override
	public void operatorControl() {
		myRobot.setSafetyEnabled(true);
		statusLights.set(true);
		leftLights.enablePWM(.5);
		rightLights.enablePWM(.5);
		rightLights.setPWMRate(1000);
		Stopwatch lightStopwatch = Stopwatch.startNew();
		double lfmax = 0, lbmax = 0, rfmax = 0, rbmax = 0;
		while (isOperatorControl() && isEnabled()) {
			// if(lfmax < PDP.getCurrent(0))
			// {
			// lfmax = PDP.getCurrent(0);
			// }
			// if(lbmax < PDP.getCurrent(1))
			// {
			// lbmax = PDP.getCurrent(1);
			// }
			// if(rfmax < PDP.getCurrent(14))
			// {
			// rfmax = PDP.getCurrent(14);
			// }
			// if(rbmax < PDP.getCurrent(15))
			// {
			// rbmax = PDP.getCurrent(15);
			// }
			//
			//
			// System.out.println("left Front " + lfmax);
			// System.out.println("left back " + lbmax);
			// System.out.println("right Front " + rfmax);
			// System.out.println("right back " + rbmax);
			//

			if (driverStick.isXHeldDown()) {
				ComputerVisionResult cvResult = doComputerVision(this.cameraReader, this.verticalTemplates,
						this.gearLookUpTable);
				SmartDashboard.putNumber("After DoCV distance", cvResult.distance);
				SmartDashboard.putString("After DoCV Target Point",
						"(" + cvResult.targetPoint.getX() + ", " + cvResult.targetPoint.getY() + ")");
				double offset = cvResult.didSucceed ? cvResult.targetPoint.getX() - 80.0 : 0;
				SmartDashboard.putNumber("Offset From Center", offset);
				if (offset != 0 && cvResult.didSucceed) {
					offsetFromCenterController.enable();
				} else if (offset == 0) {
					goToCenterRate = 0;
					offsetFromCenterController.disable();
					offsetFromCenter.update(offset);
				}
			}

			// forward on the stick is negative, we think. so, we subtract 1
			// and negate to deal with positive values for forward.
			// also, adjust to [0,2] range.

			leftY = -(getJoystickValueWithDeadZone(driverStick.getLYValue()) - 1);
			rightY = -(getJoystickValueWithDeadZone(driverStick.getRYValue()) - 1);

			
			// undo the [0,2] range adjustment
			leftY = Math.max(leftY, defaultLeftY) - 1;
			rightY = Math.max(rightY, defaultRightY) - 1;
			timeSinceLastLightUpdate = lightStopwatch.getTotalSeconds() - secondCount;
			if (timeSinceLastLightUpdate >= .1) {
				timesUpdated++;
				secondCount = lightStopwatch.getTotalSeconds();
				PWMdelayLeft = (Math.abs(leftY)+.01) / 100;
				PWMdelayRight = (Math.abs(rightY)+.01) / 100;
				leftLights.setPWMRate(1/PWMdelayLeft);
				rightLights.setPWMRate(1/PWMdelayRight);
			}

			if (isCloseToZero(leftY, 0.1) && isCloseToZero(rightY, 0.1)) {
				if (elapsedTimeWithNoDriveInputs == null) {
					elapsedTimeWithNoDriveInputs = Stopwatch.startNew();
				}
			} else {
				elapsedTimeWithNoDriveInputs = null;
			}

			updateFromTurnController(turnOutput.getValue());
			updateFromOffsetFromCenterController(offsetFromCenterOutput.getValue());

			// apply rotateToAngleRate if necessary
			if (turnController.isEnabled()) {
				leftY = .8 + rotateToAngleRate;
				rightY = .8 - rotateToAngleRate;
			} else if (offsetFromCenterController.isEnabled()) {
				leftY = leftY / 2 - goToCenterRate;
				rightY = rightY / 2 + goToCenterRate;
			} else if (driverStick.isBackHeldDown() & !centered) {
				ComputerVisionResult cvResult = doComputerVision(this.cameraReader, this.verticalTemplates,
						this.gearLookUpTable);
				if (cvResult.targetPoint.getX() == -1) {
					myRobot.tankDrive(-.4, .4);
				} else if (Math.abs(80 - cvResult.targetPoint.getX()) > 0) {
					myRobot.tankDrive(-.2, .2);
				} else if (Math.abs(80 - cvResult.targetPoint.getX()) < 40) {
					myRobot.tankDrive(-.1, .1);
				} else if (Math.abs(80 - cvResult.targetPoint.getX()) < 10) {
					centered = true;
				}
			}

			SmartDashboard.putNumber("left speed", leftY);
			SmartDashboard.putNumber("Right speed", rightY);

			if (elapsedTimeSinceLastGearShift.getTotalSeconds() <= 0.1) {
				leftY = 0;
				rightY = 0;
			}
			myRobot.tankDrive(leftY, rightY);

			Timer.delay(0.005); // wait for a motor update time

			// ///////////////////////////////////////////////////////
			// BEGIN BUTTON IF ELSE SECTION
			//
			//
			// if (driverStick.isFirstBackPressed()) {
			// switchCamera();
			// }

			// gear shift section, left bumper low - right bumper high
			// if (driverStick.isFirstLBPressed() &&
			// getElapsedSecondsWithNoDriveInputs() > 0.1) {
			// driveSystem.gearShiftLow(leftY, rightY);
			// elapsedTimeSinceLastGearShift = Stopwatch.startNew();
			// } else if (driverStick.isFirstRBPressed() &&
			// getElapsedSecondsWithNoDriveInputs() > 0.1) {
			// driveSystem.gearShiftHigh(leftY, rightY);
			// elapsedTimeSinceLastGearShift = Stopwatch.startNew();
			// }

			// single button to shift
			if (driverStick.isFirstRBPressed() && getElapsedSecondsWithNoDriveInputs() > 0.1) {
				driveSystem.gearShiftLow(leftY, rightY);
				elapsedTimeSinceLastGearShift = Stopwatch.startNew();
			} else if (driverStick.isFirstLBPressed() && getElapsedSecondsWithNoDriveInputs() > 0.1) {
				driveSystem.gearShiftHigh(leftY, rightY);
				elapsedTimeSinceLastGearShift = Stopwatch.startNew();
			}
			// navX control section
			else if (driverStick.isStartHeldDown()) {
				ahrs.reset();
				turnController.setSetpoint(ahrs.getAngle());
			}

			// Drive straight enable/disable
			if (driverStick.isDPadUpHeldDown()) {
				turnController.enable();
				defaultLeftY = 2.0;
				defaultRightY = 2.0;
			} else {
				defaultLeftY = 0;
				defaultRightY = 0;
				turnController.disable();
			}
			// climber section
			if (-manipulatorStick.getRYValue() > .1) {
				climberMotor1.set(-manipulatorStick.getRYValue());
				climberMotor2.set(-manipulatorStick.getRYValue());
				SmartDashboard.putNumber("Y joystick", -manipulatorStick.getRYValue());
			} else {
				climberMotor1.set(0);
				climberMotor2.set(0);

			}

			// TESTING

			if (manipulatorStick.getLTValue() >= .5) {
				// detects if left trigger is pressed more than halfway
				shooterSolenoid1.set(true);
				// retracts piston, lets balls enter shooter
				leftShooterMotor.set(-1);
			} else {
				shooterSolenoid1.set(false);
				leftShooterMotor.set(0);
			}

			SmartDashboard.putBoolean("isGearLoaded", isGearLoaded);
			Timer.delay(0.005); // wait for a motor update time
			SmartDashboard.putBoolean("limit switch gear", limitSwitchGear.get());
			SmartDashboard.putBoolean("limit switch robot", limitSwitchRobot.get());
			SmartDashboard.putBoolean("limit switch ground", limitSwitchGround.get());

			if (manipulatorStick.isFirstLBPressed()) {
				lightServo.set(0);
			} else if (manipulatorStick.isFirstRBPressed()) {
				lightServo.set(.5);
			}
			// END BUTTON IF ELSE SECTION
			// ///////////////////////////////////////////////////////

			if (driveSystem.getCurrentGear() == GearShift.kLowGear) {
				SmartDashboard.putString("GearShift", "LowGear");
			} else {
				SmartDashboard.putString("GearShift", "HighGear");
			}

			boolean switchPressure = compressor.getPressureSwitchValue();
			boolean compressorState = compressor.enabled();
			SmartDashboard.putString("DB/String 0", "compressor state:" + compressorState);
			SmartDashboard.putString("DB/String 1", "Pressure switch:" + switchPressure);
			SmartDashboard.putNumber("Pwm freq",  1/PWMdelayRight);
			SmartDashboard.putNumber("times",  timesUpdated);
			SmartDashboard.putNumber("Light timer", lightStopwatch.getTotalSeconds());
			SmartDashboard.putString("Turn controller", turnController.isEnabled() ? "true" : "false");
			SmartDashboard.putNumber("rotation", ahrs.getAngle());

			SmartDashboard.putNumber("time w/o drive inputs", getElapsedSecondsWithNoDriveInputs());
			SmartDashboard.putBoolean("A button", manipulatorStick.isAHeldDown());
		}
		if (!isEnabled()) {
			statusLights.set(false);
		}
	}

	private double getElapsedSecondsWithNoDriveInputs() {
		return elapsedTimeWithNoDriveInputs == null ? 0 : elapsedTimeWithNoDriveInputs.getTotalSeconds();
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}

	public void updateFromTurnController(double output) {
		SmartDashboard.putNumber("Rotate to angle rate", output);
		rotateToAngleRate = output;
	}

	public void updateFromOffsetFromCenterController(double output) {
		SmartDashboard.putNumber("OFC ctl output", output);
		goToCenterRate = output;
	}
}