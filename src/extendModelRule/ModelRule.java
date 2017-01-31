package extendModelRule;

import java.util.ArrayList;
import java.util.List;

import com.jme3.math.Vector3f;

import eu.opends.basics.MapObject;
import eu.opends.canbus.CANClient;
import eu.opends.car.SteeringCar;
import eu.opends.main.Simulator;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;

public class ModelRule {
	private final float DefaultAccel = -1f;
	private final float DefaultSpeed = 40f;
	private final float SavingSpeed = 30f;
	private final float CurveSpeed = 20f;
	private final float CreepSpeed = 1f;
	private final float SefetySensorDistance = 30f;
	private final float EmergencySensorDistance = 30f;
	private final float NaviPointSensorDistance = 40f;
	private final float HittingDistance = 3f;
	private final float RoadWidth = 2f;
	private final boolean rotation = false;
	private final boolean ondebug = false;
	private boolean auto = true;
	private boolean inGoal = false;
	private int flameCount = 0;
	private float accelerationValue = 0;
	private Simulator sim;
	private SteeringCar car;
	private List<MapObject> naviPointList;
	private MapObject[] route;
	private String[] routeName = { "01", "02", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15", "22" };
	private int currentRoute = 0;
	private int RouteSize;

	public ModelRule(Simulator sim, SteeringCar car, List<MapObject> modelList) {
		this.sim = sim;
		this.car = car;
		this.naviPointList = new ArrayList<MapObject>();
		for (MapObject model : modelList) {
			if (model.getName().contains("point")) {
				this.naviPointList.add(model);
			}
		}
		this.route = new MapObject[100];
		for (String routeName : this.routeName) {
			for (MapObject naviPoint : naviPointList) {
				if (naviPoint.getName().contains(routeName)) {
					route[currentRoute++] = naviPoint;
				}
			}
		}
		RouteSize = currentRoute;
		currentRoute = 0;
	}

	public void activateModelRule() {
		if (ondebug) {
			flameCount++;
			if (flameCount == 100) {
				System.out.println(car.getPosition());
				flameCount = 0;
			}
			if (auto) {
				System.out.println(route[currentRoute].getName());
				System.out.println(getCarDirection());
				System.out.println(getTargetDirection(route[currentRoute].getLocation()));
			}
		}
		if (auto) {
			if (inGoal) {
				brake();
				offAccel();
				brake();
				return;
			}
			steeringToTarget(route[currentRoute].getLocation());
			if (emergencySensor()) {
				emergencyRule();
			}
			if (trafficSensor()) {
				forTrafficRule();
			} else if (naviPointSensor()) {
				forNaviPointRule();
				if (hittingNaviPointSensor() != null) {
					hittingNaviPointRule(hittingNaviPointSensor());
				}
			} else {
				defaultRule();
			}
		}
	}

	private boolean trafficSensor() {
		for (TrafficObject vehicle : PhysicalTraffic.getTrafficObjectList()) {
			if (findingTraffic(vehicle.getPosition())) {
				if (findTrafficForward2(vehicle.getPosition())) {
					return true;
				}
			}
		}
		return false;
	}

	private boolean emergencySensor() {
		for (TrafficObject vehicle : PhysicalTraffic.getTrafficObjectList()) {
			if (findTrafficForward(vehicle.getPosition())) {
				if (inEmergencyDistance(vehicle.getPosition())) {
					return true;
				}
			}
		}
		return false;
	}

	private boolean naviPointSensor() {
		for (MapObject naviPoint : naviPointList) {
			if (findingNaviPoint(naviPoint.getLocation())) {
				if (findNaviPointForward(naviPoint.getLocation())) {
					return true;
				}
			}
		}
		return false;
	}

	private MapObject hittingNaviPointSensor() {
		for (MapObject naviPoint : naviPointList) {
			if (hittingNaviPoint(naviPoint.getLocation())) {
				return naviPoint;
			}
		}
		return null;
	}

	private boolean findingTraffic(Vector3f target) {
		return this.car.getPosition().distance(target) < SefetySensorDistance;
	}

	private boolean findingNaviPoint(Vector3f target) {
		return this.car.getPosition().distance(target) < NaviPointSensorDistance;
	}

	private boolean hittingNaviPoint(Vector3f target) {
		return this.car.getPosition().distance(target) < HittingDistance;
	}

	private boolean inEmergencyDistance(Vector3f target) {
		return this.car.getPosition().distance(target) < EmergencySensorDistance;
	}

	private boolean findTrafficForward(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection > carDirection + 170 || targetDirection < carDirection - 170)
				&& targetDirection < carDirection + 190 && targetDirection > carDirection - 190;
	}

	private boolean findTrafficForward2(Vector3f target) {
		if (route[currentRoute].getLocation().getX() == car.getPosition().getX()) {
			return ((target.getZ() < route[currentRoute].getLocation().getZ()
					&& target.getZ() > car.getPosition().getZ())
					|| (target.getZ() > route[currentRoute].getLocation().getZ()
							&& target.getZ() < car.getPosition().getZ()))
					&& target.getX() < car.getPosition().getX() + RoadWidth
					&& target.getX() > car.getPosition().getX() - RoadWidth;
		}
		if (route[currentRoute].getLocation().getZ() == car.getPosition().getZ()) {
			return ((target.getX() < route[currentRoute].getLocation().getX()
					&& target.getX() > car.getPosition().getX())
					|| (target.getX() > route[currentRoute].getLocation().getX()
							&& target.getX() < car.getPosition().getX()))
					&& target.getZ() < car.getPosition().getZ() + RoadWidth
					&& target.getZ() > car.getPosition().getZ() - RoadWidth;
		}
		double a = (route[currentRoute].getLocation().getZ() - car.getPosition().getZ())
				/ (route[currentRoute].getLocation().getX() - car.getPosition().getX());
		double c = car.getPosition().getZ() - a * car.getPosition().getX();
		double d = (a * target.getX() - target.getZ() + c) / Math.sqrt(a * a + 1);
		double sx = ((target.getZ() - c) * a + target.getX()) / (a * a + 1);
		if (ondebug) {
			System.out.println(' ');
			System.out.println(sx);
			System.out.println(car.getPosition().getX());
			System.out.println(route[currentRoute].getLocation().getX());
		}
		return d < RoadWidth && d > -RoadWidth
				&& ((sx < route[currentRoute].getLocation().getX() && sx > car.getPosition().getX())
				|| (sx > route[currentRoute].getLocation().getX() && sx < car.getPosition().getX()));
	}

	private boolean findNaviPointForward(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection > carDirection + 150 || targetDirection < carDirection - 150)
				&& targetDirection < carDirection + 210 && targetDirection > carDirection - 210;
	}

	private boolean findTargetLeft1(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection < carDirection - 1 || targetDirection > carDirection + 181)
				&& targetDirection > carDirection - 179 && targetDirection < carDirection + 359;
	}

	private boolean findTargetLeft2(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection < carDirection - 5 || targetDirection > carDirection + 185)
				&& targetDirection > carDirection - 175 && targetDirection < carDirection + 355;
	}

	private boolean findTargetLeft3(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection < carDirection - 20 || targetDirection > carDirection + 200)
				&& targetDirection > carDirection - 160 && targetDirection < carDirection + 340;
	}

	private boolean findTargetRight1(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection > carDirection + 1 || targetDirection < carDirection - 181)
				&& targetDirection < carDirection + 179 && targetDirection > carDirection - 359;
	}

	private boolean findTargetRight2(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection > carDirection + 5 || targetDirection < carDirection - 185)
				&& targetDirection < carDirection + 175 && targetDirection > carDirection - 355;
	}

	private boolean findTargetRight3(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection > carDirection + 20 || targetDirection < carDirection - 200)
				&& targetDirection < carDirection + 160 && targetDirection > carDirection - 340;
	}

	private boolean findTargetBehind(Vector3f target) {
		double targetDirection = getTargetDirection(target);
		double carDirection = getCarDirection();
		return (targetDirection < carDirection + 10 && targetDirection > carDirection - 10)
				|| targetDirection > carDirection + 350 || targetDirection < carDirection - 350;
	}

	private void forTrafficRule() {
		setSpeed(CreepSpeed);
		// steerLift();
		if (ondebug)
			System.out.println("findTrafficForward");
	}

	private void emergencyRule() {
		offAccel();
		if (ondebug)
			System.out.println("emergency");
	}

	private void forNaviPointRule() {
		setSpeed(SavingSpeed);
		if (ondebug)
			System.out.println("preparationForCurving");
	}

	private void hittingNaviPointRule(MapObject hittingPoint) {
		setSpeed(CurveSpeed);
		if (ondebug)
			System.out.println("curving");
		if (hittingPoint == route[currentRoute]) {
			currentRoute++;
			if (currentRoute == RouteSize) {
				if (rotation) {
					currentRoute = 0;
				} else {
					currentRoute = 0;
					inGoal = true;
				}
			}
		}
	}

	private void defaultRule() {
		if (setSpeed(DefaultSpeed)) {
			if (ondebug)
				System.out.println("defaultAccel");
		} else {
			if (ondebug)
				System.out.println("savingAccel");
		}
	}

	private double getTargetDirection(Vector3f target) {
		return Math.toDegrees(Math.atan2(target.getX() - this.car.getPosition().getX(),
				target.getZ() - this.car.getPosition().getZ()));
	}

	private double getCarDirection() {
		double direction = Math.toDegrees(car.getRotation().toAngleAxis(new Vector3f(0, 1, 0)));
		return car.getRotation().getY() > 0 ? direction : -direction;
	}

	private boolean setSpeed(float objectiveSpeed) {
		sim.getSteeringTask().getPrimaryTask().reportGreenLight();
		accelerationValue = car.getCurrentSpeedKmh() < objectiveSpeed ? this.DefaultAccel : -this.DefaultAccel;
		sim.getThreeVehiclePlatoonTask().reportAcceleratorIntensity(Math.abs(accelerationValue));
		car.setAcceleratorPedalIntensity(accelerationValue);
		return car.getCurrentSpeedKmh() < objectiveSpeed;
	}

	private void offAccel() {
		sim.getSteeringTask().getPrimaryTask().reportGreenLight();
		car.setAcceleratorPedalIntensity(0);
	}

	private void brake() {
		car.setBrakePedalIntensity(1f);
		sim.getThreeVehiclePlatoonTask().reportBrakeIntensity(1f);
		car.disableCruiseControlByBrake();
	}

	private void steerLift() {
		float steeringValue = .3f;
		CANClient canClient = Simulator.getCanClient();
		if (canClient != null)
			canClient.suppressSteering();
		sim.getSteeringTask().setSteeringIntensity(-3 * steeringValue);
		car.steer(steeringValue);
	}

	private void steeringToTarget(Vector3f target) {
		float steeringValue;
		if (findTargetLeft3(target)) {
			if (ondebug)
				System.out.println("steerLeft3");
			steeringValue = .3f;
		} else if (findTargetLeft2(target)) {
			if (ondebug)
				System.out.println("steerLeft2");
			steeringValue = .2f;
		} else if (findTargetLeft1(target)) {
			if (ondebug)
				System.out.println("steerLeft1");
			steeringValue = .1f;
		} else if (findTargetRight3(target)) {
			if (ondebug)
				System.out.println("steerRight3");
			steeringValue = -.3f;
		} else if (findTargetRight2(target)) {
			if (ondebug)
				System.out.println("steerRight2");
			steeringValue = -.2f;
		} else if (findTargetRight1(target)) {
			if (ondebug)
				System.out.println("steerRight1");
			steeringValue = -.1f;
		} else {
			steeringValue = 0;
		}
		CANClient canClient = Simulator.getCanClient();
		if (canClient != null)
			canClient.suppressSteering();
		sim.getSteeringTask().setSteeringIntensity(-3 * steeringValue);
		car.steer(steeringValue);
	}
}
