package udp_server;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.fri.FRICartesianOverlay;
import com.kuka.fri.FRIChannelInformation;
import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRIMachineProtection;
import com.kuka.fri.FRISession;
import com.kuka.fri.IFRISessionListener;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.geometry.World;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.motion.IMotion;
import com.kuka.motion.IMotionContainer;
import com.kuka.motion.IMotionContainerListener;
import com.kuka.motion.MotionContainerState;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.scenegraph.ISceneGraph;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;
import com.kuka.sensitivity.controlmode.JointImpedanceControlMode;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import javax.inject.Inject;

public class ControlParamUpdater extends RoboticsAPIApplication {
    @Inject
    private LBRMed lbr;
    @Inject
    private World world;
    @Inject
    private ISceneGraph sceneGraph;
    @Inject
    private IApplicationUI applicationUi;

    // control mode
    private enum CONTROL_MODE {
        POSITION_CONTROL,
        JOINT_IMPEDANCE_CONTROL,
        CARTESIAN_IMPEDANCE_CONTROL;
    }

    private String client_name_ = "192.170.10.1";
    private int send_period_;
    private String[] send_periods_ = { "1", "2", "5", "10" }; // send period in ms

    private FRIConfiguration fri_configuration_;
    private FRISession fri_session_;
    private FRICartesianOverlay fri_overlay_;
    private FRIChannelInformation fri_channel_information_;

    private AbstractMotionControlMode control_mode_;

    public static String[] getNames(Class<? extends Enum<?>> e) {
        return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
    }

    private String[] control_modes_ = getNames(CONTROL_MODE.class);

    private IMotionContainer fri_motion = null;
    private IMotionContainerListener fri_motion_listener = new IMotionContainerListener() {
        @Override
        public void onContainerStateChanged(IMotionContainer container,
                MotionContainerState state) {
            getLogger().info("FRI motion state changed to " + state.toString());
            if (state == MotionContainerState.ERROR) {
                running = false;
            }
        }

        @Override
        public void containerFinished(IMotionContainer container) {
        }

        @Override
        public void motionFinished(IMotionContainer container, IMotion motion) {
        }

        @Override
        public void motionStarted(IMotionContainer container, IMotion motion) {
        }
    };
    private ObjectFrame cartesian_eff;
    private Tool handTool;

    // UDP Server
    private boolean running;
    private DatagramSocket socket = null;
    private InetAddress remoteIp = null;
    private int remotePort;
    private int serverPort = 30001; // 30000 to 30010; FRI uses 30200; External Control uses 30300

    private DatagramPacket recvPacket;
    private String endTag = ";";
    private byte[] send_buffer = new byte[1024];

    // Methods
    public void request_user_config() {
        // send period
        int selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select the desired FRI send period [ms]:",
                send_periods_);
        send_period_ = Integer.valueOf(send_periods_[selectedButtonIndex]);
        getLogger().info("Send period set to: " + send_period_);
        // control mode
        selectedButtonIndex = applicationUi.displayModalDialog(
                ApplicationDialogType.QUESTION,
                "Select the desired FRI control mode: ",
                control_modes_);

        CONTROL_MODE control_mode = CONTROL_MODE.values()[selectedButtonIndex];
        switch (control_mode) {
            case POSITION_CONTROL:
                control_mode_ = new PositionControlMode();
                break;
            case JOINT_IMPEDANCE_CONTROL:
                control_mode_ = new JointImpedanceControlMode(200, 200, 200, 200, 200, 200, 200);
                break;
            case CARTESIAN_IMPEDANCE_CONTROL:
                control_mode_ = new CartesianImpedanceControlMode();
                break;
        }
        getLogger().info("Control mode set to: " + control_mode.name());
    }

    public void configure_fri() {
        fri_configuration_ = FRIConfiguration.createRemoteConfiguration(lbr, client_name_);
        fri_configuration_.setSendPeriodMilliSec(send_period_);
        getLogger().info("Creating FRI connection to " + fri_configuration_.getHostName());
        getLogger().info(
                "SendPeriod: " + fri_configuration_.getSendPeriodMilliSec() + "ms |"
                        + " ReceiveMultiplier: " + fri_configuration_.getReceiveMultiplier());

        getLogger().info("Cartesian Monitoring without TCP or base configuration!");
        fri_configuration_.setBase(world.getRootFrame());
        fri_configuration_.setTcp(cartesian_eff);

        fri_session_ = new FRISession(fri_configuration_);
        fri_overlay_ = new FRICartesianOverlay(fri_session_); // CARTESIAN_POSE is the default command mode
        fri_session_.addFRISessionListener(
                new IFRISessionListener() {
                    @Override
                    public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
                        fri_channel_information_ = friChannelInformation;
                        getLogger()
                                .info("Session State change " + friChannelInformation.getFRISessionState().toString());
                    }

                    @Override
                    public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
                        getLogger().info("Quality change signalled " + friChannelInformation.getQuality());
                        getLogger().info("Jitter " + friChannelInformation.getJitter());
                        getLogger().info("Latency " + friChannelInformation.getLatency());
                    }
                });
        // try to connect
        try {
            fri_session_.await(20, TimeUnit.SECONDS);
        } catch (final TimeoutException e) {
            getLogger().error(e.getLocalizedMessage());
            return;
        }
        getLogger().info("FRI connection established.");
    }

    // Status

    // Control methods
    private void stop_motion() {
        // IMotionContainer m4 =
        // robot.getFlange().moveAsync(lin(world.findFrame("/P4")));
        // m2.await();
        // ThreadUtil.milliSleep(500);
        // m3.cancel();
        // m4.await();
        // TODO: confirm the current motion, finish the current motion?
        fri_motion.cancel();
        // TODO: go to the start point?
    }

    private boolean change_tool(String toolname) {
        boolean success = false;
        try {
            handTool = (Tool) lbr.findObject(toolname);
            handTool.attachTo(lbr);
            success = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return success;
    }

    static private String str_ack = new String("ACK");
    static private String cmd_exit = new String("EXIT");

    // UDP Server
    protected void handle(String str) {
        getLogger().info("Recv: " + str);
        sendStatusString(str_ack);
        if (str.startsWith(cmd_exit)) {
            running = false;
            getLogger().info("Exit signal from external.");
        }
    }

    protected boolean sendStatusString(String statusStr) {
        if (remoteIp == null) {
            getLogger().error("No connection with client yet.");
            return false;
        }
        boolean success = false;
        statusStr += endTag;
        send_buffer = statusStr.getBytes();
        DatagramPacket sendPacket = new DatagramPacket(send_buffer, send_buffer.length,
                remoteIp, remotePort);
        try {
            socket.send(sendPacket);
            success = true;
        } catch (IOException e) {
            e.printStackTrace();
        }

        return success;
    }

    // RoboticsAPIApplication
    @Override
    public void initialize() throws Exception {
        sceneGraph.clean();

        if (!change_tool("HandTool")) {
            getLogger().error("Can't find tool 'HandTool'");
            cartesian_eff = lbr.getFlange();
        } else {
            cartesian_eff = handTool.getRootFrame(); // TODO: should use child frame
        }

        // Move to Start Position
        cartesian_eff.move(ptp(lbr.findFrame("KneeRS_Start")).setJointVelocityRel(0.25));

        // set FRI parameters
        request_user_config();
        // configure the FRI
        configure_fri();

        // If a violation occurs, the robot attempts to execute the path commanded
        // by the FRI client without exceeding the limit values
        // It is possible that the commanded path cannot be executed precisely and
        // that the robot leaves the path.
        fri_overlay_.setFriMachineProtection(FRIMachineProtection.ROBUST);
    }

    @Override
    public void run() throws Exception {
        // Listening port
        byte[] recvBuffer = new byte[2000];
        try {
            socket = new DatagramSocket(serverPort);
            socket.setSoTimeout(0); // Infinite
        } catch (IOException ex) {
            getLogger().error("Failed to setup server on port " + Integer.toString(serverPort));
            running = false;
        }
        getLogger().info("Starting Listenning to " + Integer.toString(serverPort));
        running = true;

        while (running) {
            // FRI Overlay
            if (fri_motion == null || fri_motion.getState().isFinal()) { // Final: Finished, Canceled, or Error
                getLogger().info("Starting a new FRI overlay.");
                PositionHold posHold = new PositionHold(control_mode_, -1, TimeUnit.SECONDS);
                fri_motion = cartesian_eff.moveAsync(
                        posHold
                                .addMotionOverlay(fri_overlay_));
                fri_motion.addContainerListener(fri_motion_listener);
            }

            // Listen to UDP
            boolean msgRecv = false;
            recvPacket = new DatagramPacket(recvBuffer, recvBuffer.length);
            try {
                socket.receive(recvPacket);
                if (remoteIp == null) {
                    remotePort = recvPacket.getPort();
                    remoteIp = recvPacket.getAddress();
                    getLogger().info("Accept connection from " +
                            remoteIp.toString() + ":" + Integer.toString(remotePort));
                } else if (!remoteIp.equals(recvPacket.getAddress()) || remotePort != recvPacket.getPort()) {
                    remotePort = recvPacket.getPort();
                    remoteIp = recvPacket.getAddress();
                    getLogger().warn("New connection from " +
                            remoteIp.toString() + ":" + Integer.toString(remotePort));
                }
                msgRecv = true;
            } catch (SocketTimeoutException e) {
                ; // So simply no message
            } catch (SocketException e) {
                ; // Socket is closed
            } catch (Exception e) {
                getLogger().error("Error in UDP server.");
                e.printStackTrace();
                break;
            }

            if (msgRecv) {
                String recvStr = new String(recvPacket.getData(), 0, recvPacket.getLength());
                handle(recvStr);
            }
        }

        getLogger().info("Exiting Application...");

        // Stop the robot
        lbr.getFlange().move(
                new PositionHold(new PositionControlMode(), 5, TimeUnit.SECONDS));
    }

    @Override
    public void dispose() {
        getLogger().info("Close FRI connection");
        if (null != fri_session_) {
            fri_session_.close();
        }

        getLogger().info("Close UDP Server");
        if (socket != null) {
            if (!(socket.isClosed()))
                socket.close();
        }
    }
}
