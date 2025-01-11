package frc.utils.devices;

import static edu.wpi.first.units.Units.RPM;

import java.io.IOException;
import java.net.FileNameMap;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoUtils {

    // PROCESSING AUTO COMMANDS
    // _______________________________________________________________________________

    // right now commands are just references to paths
    // "1" ---> path named "1"
    // they need to actually reference multiple things
    // "1s" ---> grab piece 1, score into speaker

    // A composite auto is represented by a command string
    // each command is separated by a comma
    // e.g. "1,2,3" would score game pieces 1, 2 and 3
    // This function takes in that string and returns an array with the separated commands
    // The elements in the array can then be used to get paths and assemble an auto
    // Basically, this removes commas
    public static String[] SeparateCommandString(String _commandString) {
        // make sure the string isn't empty or null or anything before continuing
        if (_commandString.length() == 0) {return null;}

        // defines how many commands are in the string
        // it starts at 1 because there cannot be 0 commands in the string as long as it's not empty
        int commandCount = 1;
        List<Integer> commandIndices = new ArrayList<Integer>();

        // NOTE - to avoid using recursive functions and while loops
        // I'm using two different for loops
        // One for counting the number of commands,
        // And one for getting the commands
        // TODO: make this simpler

        commandIndices.add(0);

        // loop through the entire string to see how many commands there are
        for (int i = 0; i < _commandString.length(); i++) {
            // if there is a comma at the current position increment the commandCount by 1
            if (_commandString.charAt(i) == ',') {
                commandCount += 1;
                commandIndices.add(i + 1);
            }
        }

        // the array to be returned
        String[] commands = new String[commandCount];

        // populating each command with the provided start indices and end indices that will be found
        for (int i = 0; i < commandIndices.size(); i++) {
            // where does the current command end
            int stopIndex = commandIndices.get(i);

            for (int j = commandIndices.get(i); j < _commandString.length(); j++) {
                // found another comma, so the command is complete
                if (_commandString.charAt(j) == ',') {
                    stopIndex = j;
                    
                    // break the loop and move on to the next command
                    break;
                }
                else if (j == _commandString.length() - 1) {
                    stopIndex = j + 1;

                    break;
                }
            }

            // define the command as the set of chars from one comma to another
            commands[i] = _commandString.substring(commandIndices.get(i), stopIndex);
        }

        // return all our commands
        return commands;
    }

    // Make an auto using a string[] of commands
    // NOTE - for now each command is just the name of a path,
    // and the paths are assembled into an auto
    // TODO: think more about how commands are going to work, brainstorm with filkin abt this
    public static Command BuildAutoFromCommands(String[] _commands, Drivetrain subsystem) throws FileVersionException, IOException, ParseException {
        // Define the auto as a command and add the first followPath command to it

        Pose2d startingPose = PathPlannerPath.fromPathFile(_commands[0]).getStartingHolonomicPose().get();
        Command followCommand = Commands.runOnce(() -> subsystem.resetOdometry(startingPose));

        PathPlannerPath finalPath = null;

        // loop through the command strings and add them to the auto
        for (int i = 0; i < _commands.length; i++) {
            // get a path file from the name of the command
            PathPlannerPath currentPath = PathPlannerPath.fromPathFile(_commands[i]);
            
            if (i == 0) {
                // if this is the first path, create a new path class with it's path points
                // also include constraints and end state

                // NOTE - the constraints defined for the first path will apply to ALL PATHS because they're combined,
                // this means that you CANNOT define different constraints for the other paths
                finalPath = new PathPlannerPath(
                    currentPath.getWaypoints(), 
                    currentPath.getRotationTargets(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    currentPath.getGlobalConstraints(), 
                    currentPath.getIdealStartingState(), 
                    currentPath.getGoalEndState(),
                    false);
            }
            else {
                // define what points are already a part of the final path and what ones are to be added
                List<Pose2d> existingPoses = finalPath.getPathPoses();
                List<Pose2d> newPoses = currentPath.getPathPoses();

                List<RotationTarget> rotationTargets = finalPath.getRotationTargets();
                if (rotationTargets.size() == 0) {
                    rotationTargets = new ArrayList<RotationTarget>();
                }
                rotationTargets.add(new RotationTarget(finalPath.getPathPoses().size() - 1, finalPath.getGoalEndState().rotation()));
                for (int j = 0; j < currentPath.getRotationTargets().size(); j++) {
                    rotationTargets.add(currentPath.getRotationTargets().get(j));
                }
                    
                // loop through all new points and throw them on top of the existing points in the list
                // NOTE - we SKIP THE FIRST POINT because it should already be in the list, the end state of the last path
                for (int j = 1; j < newPoses.size(); j++) {
                    Pose2d poseToAdd = newPoses.get(j);
                    existingPoses.add(poseToAdd);
                }

                // re-construct the path from all the points
                finalPath = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(existingPoses),
                    rotationTargets,
                    Collections.emptyList(),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    finalPath.getGlobalConstraints(), 
                    finalPath.getIdealStartingState(), 
                    currentPath.getGoalEndState(), 
                    false);
            }
        }

        
        RobotConfig robotConfig = geRobotConfig();

            // create a followPath command from the path, then add it to the auto using .andThen()
            followCommand = followCommand.andThen(
                new FollowPathCommand(
                    finalPath,
                subsystem::getPose, // Robot pose supplier
                subsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                subsystem::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController(
                    Constants.AutoConstants.translationConstants, 
                    Constants.AutoConstants.rotationConstants, 
                    0.02),
                robotConfig, // The robot configuration
                () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
                },
                subsystem // Reference to this subsystem to set requirements
        )
            );

        // return the auto as a command
        return followCommand;
    }

    public static RobotConfig geRobotConfig() {
        RobotConfig config = null;
        try{
        config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
        // Handle exception as needed
        e.printStackTrace();
        }

        return config;
    }

    // MATH STUFF
    // ______________________________________________________________________________________________

    /**
     * 
     * @param inputTransform the input vector, to be rotated
     * @param angle the angle of the rotation in DEGREES 
     * @return the vector with the rotation applied
     */
    public static Transform2d applyRotationMatrix(Transform2d inputTransform, double angle) {
        double newX = inputTransform.getX() * Math.cos(angle * (Math.PI / 180)) + inputTransform.getY() * -Math.sin(angle * (Math.PI / 180));
        double newY = inputTransform.getX() * Math.sin(angle * (Math.PI / 180)) + inputTransform.getY() * Math.cos(angle * (Math.PI / 180));

        return new Transform2d(newX, newY, new Rotation2d());
    }

    /**
     * Projects a vector represented by a Transform2d class onto another Transform2d class
     * @param a the vector that is being projected
     * @param b the vector that it's being projected onto
     * @return the resulting projected vector
     */
    public static Transform2d projectOntoVector(Transform2d a, Transform2d b) {
        double dot = dotProduct(a, b);
        double mag = magnitude(b);
        
        return b.times(dot / (mag * mag));
    }

    /**
     * Calculates the dot product of two vectors, as Transform2d classes
     * @param a input vector
     * @param b input vector
     * @return the dot product of the two vectors
     */
    public static double dotProduct(Transform2d a, Transform2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    /**
     * Calculates the magnitude of a vector, as a Transform2d class
     * @param a the input vector
     * @return the magnitude (length) of the vector
     */
    public static double magnitude(Transform2d a) {
        return Math.sqrt(a.getX() * a.getX() + a.getY() * a.getY());
    } 

    /**
     * Returns a normalized vector (length of 1) that points in the direction the robot is facing
     * @param heading the heading in DEGREES of the robot
     * @return the heading, as a normalized vector
     */
    public static Transform2d getHeadingVector(double heading) {
        return applyRotationMatrix(new Transform2d(1, 0, new Rotation2d()), heading);
    }
}