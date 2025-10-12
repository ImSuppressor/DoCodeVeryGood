# PID and Road Runner Explained for FTC Mecanum Robots

## 1. How PID Works in Robotics

**PID** stands for **Proportional–Integral–Derivative**. It is a feedback control loop that helps your robot automatically correct its motion to reach a target.

- **P (Proportional)**: Reacts to how far off you are from the target.
- **I (Integral)**: Reacts to how long you’ve been off target, fixes small steady errors.
- **D (Derivative)**: Reacts to how fast the error is changing, prevents overshoot.

**Example: Driving Forward 100 cm**
- Current position = 90 cm → error = 10 cm.
- P term boosts motor power proportionally.
- I term corrects if robot consistently undershoots.
- D term slows correction as robot approaches target.

### With Mecanum + Dead Wheels
- **Motor Velocity PID**: Maintains motor speed.
- **Pose PID**: Uses dead wheels to correct x, y, heading errors.
- Dead wheels improve accuracy by avoiding slip errors from mecanum wheels.

---

## 2. PID in Java Code Example
```java
public class PIDController {
    private double kP, kI, kD;
    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.lastTime = System.nanoTime();
    }

    public double update(double target, double current) {
        double currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        double error = target - current;
        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        double output = kP * error + kI * integralSum + kD * derivative;
        lastError = error;
        lastTime = currentTime;
        return output;
    }
}
```

- `update()` returns the power correction for motors based on error.

---

## 3. Road Runner Parameters Explained

### Parameters Section
| Parameter | Meaning | Effect |
|-----------|---------|-------|
| axialGain | P-term for X-axis | Faster/slower forward correction |
| axialVelGain | Velocity feedforward X | Faster acceleration response |
| headingGain | P-term for heading | Faster rotation correction |
| headingVelGain | Velocity feedforward heading | Smooth rotational motion |
| inPerTick | Inches per encoder tick | Converts encoder ticks to distance |
| kA | Acceleration feedforward | Smooth acceleration/deceleration |

### Telemetry Section
| Telemetry | Meaning |
|-----------|--------|
| heading | Current orientation in degrees |
| headingError | Difference to target heading |
| x | Current forward position |
| xError | Distance still needed to reach target |
| y | Current sideways (strafe) position |
| yError | Sideways distance needed to target |

**Difference Between y and yError:**
- `y`: current sideways position
- `yError = targetY - currentY`: how far you need to move sideways

---

## 4. How Road Runner Parameters Affect PID and Motors

### Control Flow
```
Target trajectory -> Compute error -> PID gains -> Feedforward terms -> Drive signal -> Motor power -> Robot moves
```

### Feedback vs Feedforward
- **PID gains (axialGain, headingGain)**: How aggressively the robot corrects errors
- **Feedforward (kV, kA, kStatic)**: Base power to achieve target velocity/acceleration

### Motor Power Computation
```java
leftFront.setPower(yPower + xPower + headingPower);
leftRear.setPower(yPower - xPower + headingPower);
rightFront.setPower(yPower - xPower - headingPower);
rightRear.setPower(yPower + xPower - headingPower);
```
- `xPower`, `yPower`, `headingPower` come from PID + feedforward.

**Summary Table:**
| Parameter | Effect on PID/Motors |
|-----------|--------------------|
| axialGain | Stronger correction for X-axis error |
| headingGain | Stronger correction for rotation error |
| kV | Sets base motor power for velocity |
| kA | Adjusts motor power for acceleration |
| kStatic | Ensures robot starts moving |
| inPerTick | Scales error calculation correctly |

---

This structure ensures your mecanum + dead wheel robot moves **smoothly, accurately, and predictably** following trajectories.

