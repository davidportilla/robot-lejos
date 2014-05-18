import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;

public class MainClass {

	public static void main(String[] args) {
		NavigatorRobot myRobot = new NavigatorRobot(SensorPort.S2,
SensorPort.S4, SensorPort.S3, SensorPort.S1, Motor.A, Motor.C, Motor.B);
		boolean objDet = false;
		ext: for (int i = 1; i <= 4; i++) {
			myRobot.rotate(120);
			for (int n = 1; n < 9; n++) {
				myRobot.objectDetected = false;
				objDet = myRobot.scan(-240 / 8);
				if (objDet) {
					System.out.println("OBJ DET AT: "
					+ (int) myRobot.objectDetectedDistance);
					break ext;
				}
			}
			myRobot.rotate(115);
			myRobot.travel(600);
		}
		if (objDet) {
			if ((myRobot.objectDetectedDistance - 30) < 0) {
				myRobot.travel((int)
(myRobot.objectDetectedDistance - 30) * 10);
			} else {
myRobot.travel((int) (myRobot.objectDetectedDistance - 30) * 10);
			}
			myRobot.alignToObjectDetectedOld();
			myRobot.catchBall();
			myRobot.goToRDV();
			myRobot.leftBall();
			myRobot.backHome();
		} else {
			System.out.println("FAIL");
		}
		Button.ENTER.waitForPressAndRelease();
	}
}

 
