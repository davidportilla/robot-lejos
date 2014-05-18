import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.TouchSensor;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.Touch;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.navigation.Pose;
import lejos.robotics.objectdetection.Feature;
import lejos.robotics.objectdetection.FeatureDetector;
import lejos.robotics.objectdetection.FeatureListener;
import lejos.robotics.objectdetection.FusorDetector;
import lejos.robotics.objectdetection.RangeFeatureDetector;
import lejos.robotics.objectdetection.TouchFeatureDetector;
import lejos.util.Delay;

public class NavigatorRobot implements FeatureListener {

	private static final int MAX_DETECT_ULTRASONIC = 70;
	private static final int PERIOD_ULTRASONIC = 50;
	// Sensors
	private UltrasonicSensor sonicSensor;
	private Touch leftBump;
	private Touch rightBump;
	private ColorSensor colorSensor;
	// Detectors
	protected RangeFeatureDetector usRangeDetector; // ultrasonic
	protected FeatureDetector detLeftBump; // touch left
	protected FeatureDetector detRightBump; // touch right
	protected FusorDetector touchFusionDetector; // touch both
	// Elements du robot
	private static final float DIAMETRE_ROUE = 41.0f; // en mm
	private static final float ECART_ROUES = 126.0f; // en mm
	// Pilot and navigation
	private DifferentialPilot differentialPilot;
	public OdometryPoseProvider pose;
	// Arm
	private RegulatedMotor armMotor;
	// References
	public static final float departX = 0;
	public static final float departY = 0;
	public static final float departTheta = 90;
	public static final float limitUpX = 2500;
	public static final float limitDownX = -100;
	public static final float limitLeftY = -700;
	public static final float limitRightY = 800;
	public static final float circleX = 500;
	public static final float circleY = 350;
	public static final float radius = 450;
	public static final int travelSpeed = 500; //mm par seconde
	public static final int rotateSpeed = 90; //Degres par seconde
	public static final int vitesseBras = 120; //Degres par seconde

	// Flag object detected
	public boolean objectDetected;
	public float objectDetectedDistance;
	public float objectDetectedAngle;

	public NavigatorRobot(SensorPort ultrasonic, SensorPort touchLeft,SensorPort touchRight, SensorPort color, RegulatedMotor leftMotor,	RegulatedMotor rightMotor, RegulatedMotor armMotor) {
		// init sensors
		this.sonicSensor = new UltrasonicSensor(ultrasonic);
		this.leftBump = new TouchSensor(touchLeft);
		this.rightBump = new TouchSensor(touchRight);
		this.colorSensor = new ColorSensor(color);
		// init detectors and listeners
		this.usRangeDetector = new RangeFeatureDetector(
sonicSensor, MAX_DETECT_ULTRASONIC, PERIOD_ULTRASONIC);
		usRangeDetector.addListener(this);
	this.detLeftBump = new TouchFeatureDetector(leftBump,-4.5, 7);
	this.detRightBump = new TouchFeatureDetector(rightBump, 4.5, 7);
		this.touchFusionDetector = new FusorDetector();
		touchFusionDetector.addDetector(detLeftBump);
		touchFusionDetector.addDetector(detRightBump);
		touchFusionDetector.addListener(this);
		// init pilot and navigation
		this.differentialPilot = new DifferentialPilot(
DIAMETRE_ROUE,	ECART_ROUES, leftMotor, rightMotor);
		this.pose = new OdometryPoseProvider(this.differentialPilot);
		pose.setPose(new Pose(departX, departY, departTheta));
		this.differentialPilot.addMoveListener(pose);
		this.differentialPilot.setTravelSpeed(travelSpeed);
		this.differentialPilot.setRotateSpeed(rotateSpeed);
		this.armMotor = armMotor;
		this.armMotor.setSpeed(vitesseBras);
		this.objectDetected = false;
		this.objectDetectedDistance = -1;
		this.sonicSensor.off();
	}

	public boolean scan(int degrees) {
		this.sonicSensor.reset();
		this.differentialPilot.setRotateSpeed(rotateSpeed / 2);
		this.differentialPilot.rotate(degrees);
		this.differentialPilot.setRotateSpeed(rotateSpeed);
		this.sonicSensor.off();
		return this.isObjectDetected();
	}

	public boolean isObjectDetected() {
		boolean objDet = this.objectDetected;
		this.objectDetected = false;
		return objDet;
	}


	public void resetObjectDetectedDistance() {
		this.objectDetectedDistance = -1;
	}

	public void rotate(int degrees) {
		this.differentialPilot.rotate(degrees, false);
	}

	public void travel(int mm) {
		this.differentialPilot.travel(mm);
	}

	public void alignToObjectDetectedOld() {
		this.sonicSensor.reset();
		this.differentialPilot.rotate(90);
		this.differentialPilot.setRotateSpeed(rotateSpeed / 4);
		this.differentialPilot.rotate(-180, true);
		while (this.usRangeDetector.scan() == null) {
			// Wait
		}
		this.differentialPilot.stop();
		float angle1 = pose.getPose().getHeading();
		if (angle1 < 0) {
			angle1 += 360;
		}
		System.out.println("angle 1: " + angle1);
		this.differentialPilot.rotate(-20, false);
		this.differentialPilot.rotate(-60, true);
		while (this.usRangeDetector.scan() != null) {
			// Wait
		}
		this.differentialPilot.stop();
		float angle2 = pose.getPose().getHeading();
		if (angle2 < 0) {
			angle2 += 360;
		}
		System.out.println("angle 2: " + angle2);
		this.differentialPilot.rotate(1.2*(angle1 - angle2) / 2);
		this.differentialPilot.setRotateSpeed(rotateSpeed);
		this.sonicSensor.off();
	}

	public void catchBall() {
		this.armMotor.rotate(-220);
		this.differentialPilot.setTravelSpeed(travelSpeed / 3);
		this.travel(190);
		this.armMotor.setSpeed(vitesseBras / 3);
		this.armMotor.rotate(70);
		this.armMotor.setSpeed(vitesseBras);
		this.differentialPilot.setTravelSpeed(travelSpeed);
		this.travel(-200);
	}

	public void allerPointRDV() {
		if (pose.getPose().getX() > 0) {
			this.differentialPilot.rotate(180 - pose.getPose().getHeading());
		 this.differentialPilot.travel(pose.getPose().getX());
		 this.differentialPilot.rotate(90);
		 this.differentialPilot.travel(pose.getPose().getY()-350);
		 this.differentialPilot.rotate(90);
		} else {
		 this.differentialPilot.rotate(0 - pose.getPose().getHeading());
		 this.differentialPilot.travel(-pose.getPose().getX());
		 this.differentialPilot.rotate(-90);
		 this.differentialPilot.travel(pose.getPose().getY()-350);
		 this.differentialPilot.rotate(90);
		}
	}

	@Override
	public void featureDetected(Feature feature, FeatureDetector detector) {
		if (detector instanceof RangeFeatureDetector) {
			double x = pose.getPose().getX()
			+ (feature.getRangeReading().getRange() + 10) * 10 * Math.cos(Math.toRadians(pose.getPose().getHeading()));
			double y = pose.getPose().getY()                            	          + (feature.getRangeReading().getRange() + 10) * 10 * Math.sin(Math.toRadians(pose.getPose().getHeading()));
		  if (!(this.pointIntoCircle(x, y, circleX, circleY, radius))) {
				Sound.playTone(10000, 1);
				this.objectDetected = true;
				this.objectDetectedAngle = this.pose.getPose().getHeading();
				this.objectDetectedDistance = feature.getRangeReading().getRange();
			} else {
				Sound.playTone(440, 1);
			}

		} else {
			// touch sensor
			this.differentialPilot.stop();
		}
	}

	private boolean pointIntoCircle(double x, double y, double xCircle, double yCircle, double radius) {
		return (Math.sqrt(Math.pow(x - xCircle, 2) + Math.pow(y - yCircle, 2)) < radius);
	}

	public class ColorAlarmThread extends Thread {
		@Override
		public void run() {
			while (true) {
				System.out.println(colorSensor.getLightValue());
				if (colorSensor.getLightValue() < 200) {
					// Black zone, danger ou retour zone
					differentialPilot.stop();
					// RETOURNER ZONE ZERO
				}
				try {
					Thread.sleep(150);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}

	public void startAlarmThread() {
		ColorAlarmThread cat = new ColorAlarmThread();
		cat.start();
	}

	public void goToRDV() {
		if (pose.getPose().getX() > 0) {
			differentialPilot.rotate(180 - pose.getPose().getHeading());
			differentialPilot.travel(pose.getPose().getX());
			differentialPilot.rotate(90);
			differentialPilot.travel(pose.getPose().getY()-350);
			differentialPilot.rotate(90);
		} else {
			differentialPilot.rotate(0 - pose.getPose().getHeading());
			differentialPilot.travel(-pose.getPose().getX());
			differentialPilot.rotate(-90);
			differentialPilot.travel(pose.getPose().getY()-350);
			differentialPilot.rotate(90);
		}
	}

	public void alignToZDR() {

		this.sonicSensor.reset();
		this.differentialPilot.setRotateSpeed(rotateSpeed / 3);
		this.differentialPilot.rotate(60);
		this.differentialPilot.rotate(-360, true);
		while (this.usRangeDetector.scan() == null) {
			// Wait
		}
		this.differentialPilot.stop();
		float angle7 = pose.getPose().getHeading();
		this.differentialPilot.rotate(-20, false);
		this.differentialPilot.rotate(-80, true);
		while (this.usRangeDetector.scan() != null) {
			// Wait
		}
		this.differentialPilot.stop();
		float angle8 = pose.getPose().getHeading();
		this.differentialPilot.rotate(1.25*(angle7 - angle8) / 2);
		this.differentialPilot.setRotateSpeed(rotateSpeed);
		this.sonicSensor.off();
	}

	public void leftBall() {
		this.travel(500);
		while (differentialPilot.isMoving());
		this.armMotor.rotate(-70, false);
		Delay.msDelay(1500);
		this.armMotor.rotate(220, true);
		this.travel(-300);
	}

	public void backHome() {
		this.rotate(90);
		this.travel(-400);
	}
}

