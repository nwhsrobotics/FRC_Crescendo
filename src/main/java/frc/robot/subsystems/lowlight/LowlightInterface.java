package frc.robot.subsystems.lowlight;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LowlightConstants;
import lowlight.control.Control.Command;
import lowlight.control.Control.Result;

public class LowlightInterface {
    private Socket socket;
    private Timer connectAttemptTimer = new Timer();
    private boolean freeConnectAttemptDepleted = false;
    private Queue<Command> commandQueue = new LinkedList<Command>();

    private void restoreSocket() {
        if ((socket != null && socket.isClosed()) ||
            (!connectAttemptTimer.hasElapsed(LowlightConstants.INTERFACE_RECONNECT_ATTEMPT_INTERVAL_S)) && freeConnectAttemptDepleted) {
            return;
        }
        freeConnectAttemptDepleted = true;
        connectAttemptTimer.restart();

        try {
            socket = new Socket(LowlightConstants.LOWLIGHT_IP, 777);
            socket.setSoTimeout(500);
        }
        catch (UnknownHostException error) {
            System.out.println("Achtung! Lowlight hostname does not exist.");
        }
        catch (IOException error) {
            System.out.println("Achtung! Failed to connect to Lowlight.");
        }
    }

    private Result readResult() throws IOException {
        InputStream input = socket.getInputStream();
        return Result.parseDelimitedFrom(input);
    }

    private void processCommandQueue() throws IOException {
        Command command = commandQueue.poll();
        if (command == null) {
            return;
        }
        
        OutputStream output = socket.getOutputStream();
        command.writeTo(output);
        output.flush();

        Result result = readResult();
        System.out.println("Hinweis: Lowlight interface received response for command "
            + command.getId()
            + " with "
            + (result.getOk() ? "ok" : "not ok")
            + " result. Message says:\n"
            + result.getMessage());
    }

    private edu.wpi.first.wpilibj2.command.InstantCommand lowlightInterfaceTask = new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
        restoreSocket();
        if (socket.isConnected()) {
            try {
                processCommandQueue();
            }
            catch (IOException error) {
                System.out.println("Achtung! Failed to process Lowlight command due to I/O error.");
            }
        }
        
        run();
    });

    public void run() {
        edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().schedule(lowlightInterfaceTask);
    }

    public void registerCamera(Camera camera) {
        Command command = Command.newBuilder()
            .setId(0)
            .setIndex(camera.getIndex())
            .setQuality(camera.getQuality())
            .build();

        commandQueue.add(command);
    }
}
