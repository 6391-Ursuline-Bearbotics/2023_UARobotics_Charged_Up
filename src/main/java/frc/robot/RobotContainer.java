/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

// Subsystem Imports
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.Constants.DRIVE;
// Constant Imports
import frc.robot.Constants.OI;
// Special Imports
import frc.robot.UA6391.XboxController6391;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final PhotonVision m_PhotonVision = new PhotonVision();
  
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_swerveSubsystem;

  private boolean fast = true;
  
  @Log(tabName = "Dashboard")
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  XboxController6391 drv = new XboxController6391(OI.DRVCONTROLLERPORT, 0.1);
  XboxControllerSim m_driverControllerSim = new XboxControllerSim(OI.DRVCONTROLLERPORT);
  private final ControlScheme m_scheme = new ControlScheme(drv);

  // The operator's controller
  XboxController6391 op = new XboxController6391(OI.OPCONTROLLERPORT, 0.1);
  XboxControllerSim m_operatorControllerSim = new XboxControllerSim(OI.OPCONTROLLERPORT);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    
    m_swerveSubsystem.setDefaultCommand(new RunCommand(() -> dt.setModuleStates(m_scheme.getJoystickSpeeds()), m_swerveSubsystem));

    LiveWindow.disableAllTelemetry();

    m_PhotonVision.fieldSetup(m_swerveSubsystem.dt.getField());

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(OI.PRACTICE);

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // When the left bumper is pressed on driver controller controls are slower
    drv.BumperL.onTrue(new ConditionalCommand(new InstantCommand(() -> {setDriveMaxSpeed(
        DRIVE.MAX_STRAFE_SPEED_SLOW, DRIVE.MAX_ROTATE_SPEED_SLOW);
        fast = false;}),
      new InstantCommand(() -> {setDriveMaxSpeed(DRIVE.MAX_STRAFE_SPEED_FAST, DRIVE.MAX_ROTATE_SPEED_FAST);
        fast = true;}),
      () -> fast));
  
    // When start button is pressed reorient the field drive to the current heading
    drv.StartButton.onTrue(Commands.runOnce(() -> dt.zeroGyroscope()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  @Log
  public String getPose() {
    return dt.getPose().toString();
  }

  @Config(name = "Drive Max Speed", width = 2, height = 2)
  private void setDriveMaxSpeed(
          @Config(name = "maxStrafe", defaultValueNumeric = .35) double maxStrafe,
          @Config(name = "maxRotate", defaultValueNumeric = .35) double maxRotate) {
    m_swerveSubsystem.dt.setMaxSpeeds(maxStrafe, maxStrafe, maxRotate);
  }

  @Config(name = "Drive Max Ramp", width = 2, height = 2)
  private void setDriveMaxRamp(
          @Config(name = "maxStrafeRamp", defaultValueNumeric = 10) double maxStrafeRamp,
          @Config(name = "maxRotateRamp", defaultValueNumeric = 10) double maxRotateRamp) {
    m_scheme.modifySlew(maxStrafeRamp, maxStrafeRamp, maxRotateRamp);
  }
}
