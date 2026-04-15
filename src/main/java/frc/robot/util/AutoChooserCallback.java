package frc.robot.util;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.questnav.QuestNav;
import java.util.List;
import java.util.function.Consumer;

public class AutoChooserCallback implements Consumer<Command> {

  private final Drive drive;
  private final QuestNav questNav;

  public AutoChooserCallback(Drive drive, QuestNav questNav) {
    this.drive = drive;
    this.questNav = questNav;
  }

  @Override
  public void accept(Command t) {
    try {
      List<PathPlannerPath> pathsInAuto = PathPlannerAuto.getPathGroupFromAutoFile(t.getName());
      PathPlannerPath startingPath = pathsInAuto.get(0);
      Pose2d startingPose =
          startingPath
              .getStartingHolonomicPose()
              .orElse(startingPath.getStartingDifferentialPose());

      boolean isFlipped = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

      drive.setPose(
          isFlipped
              ? startingPose.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
              : startingPose);
      questNav.setPose(
          isFlipped
              ? startingPose.rotateAround(FieldConstants.fieldCenter, Rotation2d.k180deg)
              : startingPose);
    } catch (Exception e) {
      DriverStation.reportWarning("Could not load auto path: " + t.getName(), false);
    }
  }
}
