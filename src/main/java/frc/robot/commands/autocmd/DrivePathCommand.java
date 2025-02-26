package frc.robot.commands.autocmd;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;

public class DrivePathCommand {
    public static Command drivePath(String pathToFollow) {
        System.out.println(pathToFollow);
        PathPlannerPath path;
        try {
            path = PathPlannerPath.fromPathFile(pathToFollow);
        return AutoBuilder.followPath(path);
    } catch (FileVersionException | IOException | ParseException e) {
            // TODO Auto-generated catch block
        e.printStackTrace();
        }
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return null;
    }
}