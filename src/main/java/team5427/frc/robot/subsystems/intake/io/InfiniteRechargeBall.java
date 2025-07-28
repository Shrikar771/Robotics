package team5427.frc.robot.subsystems.intake.io;

import static edu.wpi.first.units.Units.Inches;

import org.dyn4j.geometry.Geometry;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import static edu.wpi.first.units.Units.Grams;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class InfiniteRechargeBall extends GamePieceOnFieldSimulation{
    public static final GamePieceInfo INFINITE_RECHARGABLE_BALL_INFO = new GamePieceInfo("Ball", Geometry.createCircle(Units.inchesToMeters(14)/2), Inches.of(7), Grams.of(142), COEFFICIENT_OF_FRICTION, DEFAULT_LINEAR_DAMPING, TYPICAL_FIXTURE_COUNT);
    public InfiniteRechargeBall(Translation2d initialPosition){
        super(INFINITE_RECHARGABLE_BALL_INFO, new Pose2d(initialPosition, new Rotation2d()));

    }
}
