package udp_server;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import com.kuka.fri.FRICartesianOverlay;
import com.kuka.fri.FRIChannelInformation;
import com.kuka.fri.FRIConfiguration;
import com.kuka.fri.FRIMachineProtection;
import com.kuka.fri.FRISession;
import com.kuka.fri.IFRISessionListener;
import com.kuka.fri.common.FRISessionState;
import com.kuka.geometry.ObjectFrame;
import com.kuka.geometry.Tool;
import com.kuka.geometry.World;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.motion.IMotion;
import com.kuka.motion.IMotionContainer;
import com.kuka.motion.IMotionContainerListener;
import com.kuka.motion.MotionContainerState;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplicationState;
import com.kuka.roboticsAPI.motionModel.PositionHold;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKey;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyBar;
import com.kuka.roboticsAPI.uiModel.userKeys.IUserKeyListener;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyAlignment;
import com.kuka.roboticsAPI.uiModel.userKeys.UserKeyEvent;
import com.kuka.scenegraph.ISceneGraph;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.stream.Collectors;
import javax.inject.Inject;
import udp_server.ParameterServer.CARTESIAN_AXIS;

public class ControlParamUpdater extends RoboticsAPIApplication {
    @Inject
    private LBRMed lbr;
    @Inject
    private World world;
    @Inject
    private ISceneGraph sceneGraph;
    @Inject
    private IApplicationUI applicationUi;

    private ObjectFrame cartesian_eff;
    private Tool handTool;

    private String client_name_ = "192.170.10.1";
    private int send_period_;
    private String[] send_periods_ = { "1", "2", "5", "10" }; // send period in ms

    private FRIConfiguration fri_configuration_;
    private FRISession fri_session_;
    private FRICartesianOverlay fri_overlay_;
    private FRIChannelInformation fri_channel_information_;
    private AbstractMotionControlMode control_mode_;
    private int current_ctrl_mode_idx_;

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

    // Parameters
    private ParameterServer param_server;

    // UDP Server
    private boolean running;
    private DatagramSocket socket = null;
    private InetAddress remoteIp = null;
    private int remotePort;
    private int serverPort = 30001; // 30000 to 30010; FRI uses 30200; External Control uses 30300

    private DatagramPacket recvPacket;
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
                ParameterServer.get_controller_names());

        current_ctrl_mode_idx_ = selectedButtonIndex;
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
        fri_session_.addFRISessionListener(new IFRISessionListener() {
            @Override
            public void onFRISessionStateChanged(FRIChannelInformation friChannelInformation) {
                fri_channel_information_ = friChannelInformation;
                getLogger()
                        .info("Session State -> " + friChannelInformation.getFRISessionState().toString());
            }

            @Override
            public void onFRIConnectionQualityChanged(FRIChannelInformation friChannelInformation) {
                getLogger().info("[FRI Session] Quality -> " + friChannelInformation.getQuality()
                        + " | Jitter " + friChannelInformation.getJitter()
                        + " | Latency " + friChannelInformation.getLatency());
            }
        });
        fri_overlay_ = new FRICartesianOverlay(fri_session_); // CARTESIAN_POSE is the default command mode

        // If a violation occurs, the robot attempts to execute the path commanded
        // by the FRI client without exceeding the limit values
        // It is possible that the commanded path cannot be executed precisely and
        // that the robot leaves the path.
        fri_overlay_.setFriMachineProtection(FRIMachineProtection.ROBUST);
        // // In order to fully exploit the performance reserves of the robot, the
        // acceleration
        // // can optionally be increased
        // fri_overlay_.overrideJointAcceleration(accelerationOverride);

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

    private void show_params() {
        // FRI Channel Information
        getLogger().info("[FRI Channel] State: " + fri_channel_information_.getFRISessionState()
                + " | Quality: " + fri_channel_information_.getQuality()
                + " | Jitter: " + fri_channel_information_.getJitter()
                + " | Latency: " + fri_channel_information_.getLatency());

        // Controller Parameters
        param_server.show_controller_params();

        // Other status
        double[] external_force = param_server.get_external_force();
        String msg = new String("[External Force] Force: ");
        for (int i = 0; i < 3; i++) {
            msg += String.format("%.3f", external_force[i]) + ", ";
        }
        msg += " | Torque: ";
        for (int i = 3; i < 6; i++) {
            msg += String.format("%.3f", external_force[i]) + ", ";
        }
        getLogger().info(msg);
        msg = new String("[Motion Command Activity] "
                + Boolean.toString(param_server.get_motion_command_activity()));
        getLogger().info(msg);
    }

    // Control methods
    private void stop_motion() {
        // TODO: confirm the current motion, finish the current motion?
        fri_motion.cancel();
        // close_fri_session();
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

    // UDP Server
    static private class ProtocolCode {
        static private final String str_success = "R,S";
        static private final String str_failure = "R,F";
        static private final String separator_section = ",";
        static private final char separator_command = ';';
        static private final char RESPONSE = 'R';

        static private final char GET = 'G';
        static private final char SET = 'S';

        static private final char CTRL_POS = 'P';
        static private final char CTRL_JNT = 'J';
        static private final char CTRL_CART = 'C';
        static private final char STATUS = 'S';
        static private final char TOOL = 'T';
        static private final char EXIT = 'E';
        static private final char SHOW = 'W';

        static private final char DAMPING = 'D';
        static private final char STIFFNESS = 'S';
        static private final char ADDITIONAL_FORCE = 'A';

        static private final char DOF_X = 'X';
        static private final char DOF_Y = 'Y';
        static private final char DOF_Z = 'Z';
        static private final char DOF_A = 'A';
        static private final char DOF_B = 'B';
        static private final char DOF_C = 'C';
        static private final char DOF_NULLSPACE = 'N';
        static private final char DOF_TRANSL = 'T';
        static private final char DOF_ROT = 'R';
        static private final char DOF_ALL = 'E';
        static private final Map<Character, ParameterServer.CARTESIAN_AXIS> dof_char2enum;
        static {
            Map<Character, ParameterServer.CARTESIAN_AXIS> map = new HashMap<>();
            map.put(DOF_X, ParameterServer.CARTESIAN_AXIS.DOF_X);
            map.put(DOF_Y, ParameterServer.CARTESIAN_AXIS.DOF_Y);
            map.put(DOF_Z, ParameterServer.CARTESIAN_AXIS.DOF_Z);
            map.put(DOF_A, ParameterServer.CARTESIAN_AXIS.DOF_A);
            map.put(DOF_B, ParameterServer.CARTESIAN_AXIS.DOF_B);
            map.put(DOF_C, ParameterServer.CARTESIAN_AXIS.DOF_C);
            map.put(DOF_NULLSPACE, ParameterServer.CARTESIAN_AXIS.DOF_NULLSPACE);
            map.put(DOF_TRANSL, ParameterServer.CARTESIAN_AXIS.DOF_TRANSL);
            map.put(DOF_ROT, ParameterServer.CARTESIAN_AXIS.DOF_ROT);
            map.put(DOF_ALL, ParameterServer.CARTESIAN_AXIS.DOF_ALL);
            dof_char2enum = Collections.unmodifiableMap(map);
        }

        static private final char EXTERNAL_FORCE = 'F';
        static private final char MOTION_ACTIVITY = 'A';
    }

    protected boolean sendReplyMsg(String msg) {
        if (remoteIp == null) {
            getLogger().error("No connection with any client yet.");
            return false;
        }
        boolean success = false;
        msg += ProtocolCode.separator_command;
        send_buffer = msg.getBytes();
        DatagramPacket sendPacket = new DatagramPacket(send_buffer, send_buffer.length, remoteIp, remotePort);
        try {
            socket.send(sendPacket);
            success = true;
        } catch (IOException e) {
            e.printStackTrace();
        }

        return success;
    }

    private void query_error(String error_msg) {
        getLogger().error(error_msg);
        sendReplyMsg(ProtocolCode.str_failure);
    }

    protected void handle(String str) {
        // Debug
        getLogger().info("Recv: " + str);
        boolean restart_fri = false;

        if (str.charAt(str.length() - 1) != ProtocolCode.separator_command) {
            query_error("Unrecognized Ending");
            return;
        }
        str = str.substring(0, str.length() - 1);
        String[] data = str.split(ProtocolCode.separator_section);
        if (data.length < 1) {
            query_error("No data");
            return;
        }
        // if (data.length < 1 || !Arrays.stream(data).allMatch(s -> s.length() == 1 ||
        // ??)) {
        // query_error("Packet data should be an array of chars or double data");
        // return;
        // }
        if (data[0].charAt(0) == ProtocolCode.SET) {
            if (data.length < 2) {
                query_error("No data");
                return;
            }
            switch (data[1].charAt(0)) {
                case ProtocolCode.EXIT:
                    running = false;
                    getLogger().info("Exit signal from external.");
                    sendReplyMsg(ProtocolCode.str_success);
                    break;
                case ProtocolCode.SHOW:
                    show_params();
                    sendReplyMsg(ProtocolCode.str_success);
                    break;

                case ProtocolCode.CTRL_CART:
                    if (data.length < 5) {
                        query_error("No data");
                        return;
                    }

                    char param_type = data[2].charAt(0);
                    CARTESIAN_AXIS axis = ProtocolCode.dof_char2enum.get(data[3].charAt(0));
                    double[] value = Arrays.stream(Arrays.copyOfRange(data, 4, data.length))
                            .mapToDouble(Double::parseDouble)
                            .toArray();

                    switch (param_type) {
                        case ProtocolCode.STIFFNESS:
                            if (!param_server.set_cart_stiffnesses(axis, value)) {
                                sendReplyMsg(ProtocolCode.str_failure);
                            } else {
                                restart_fri = true;
                                sendReplyMsg(ProtocolCode.str_success);
                            }
                            break;
                        case ProtocolCode.DAMPING:
                            if (!param_server.set_cart_dampings(axis, value)) {
                                sendReplyMsg(ProtocolCode.str_failure);
                            } else {
                                restart_fri = true;
                                sendReplyMsg(ProtocolCode.str_success);
                            }
                            break;
                        case ProtocolCode.ADDITIONAL_FORCE:
                            if (!param_server.set_additional_forces(axis, value)) {
                                sendReplyMsg(ProtocolCode.str_failure);
                            } else {
                                restart_fri = true;
                                sendReplyMsg(ProtocolCode.str_success);
                            }
                            break;
                        default:
                            query_error("Unknown Cart Param");
                            break;
                    }
                    break;

                // TODO:
                case ProtocolCode.CTRL_JNT:
                case ProtocolCode.CTRL_POS:
                case ProtocolCode.TOOL:
                    query_error("Not Implemented");
                    break;
                default:
                    query_error("Unknown Command");
                    break;
            }
        } else if (data[0].charAt(0) == ProtocolCode.GET) {
            if (data.length < 2) {
                query_error("No data");
                return;
            }
            String reply;
            switch (data[1].charAt(0)) {
                case ProtocolCode.STATUS:
                    if (data.length < 3) {
                        query_error("No data");
                        return;
                    }
                    switch (data[2].charAt(0)) {
                        case ProtocolCode.EXTERNAL_FORCE:
                            double[] forces = param_server.get_external_force();
                            reply = ProtocolCode.RESPONSE
                                    + ProtocolCode.separator_section
                                    + Arrays.stream(forces).mapToObj(x -> String.format("%.3f", x)).collect(
                                            Collectors.joining(ProtocolCode.separator_section));
                            sendReplyMsg(reply);
                            break;
                        case ProtocolCode.MOTION_ACTIVITY:
                            boolean active = param_server.get_motion_command_activity();
                            reply = ProtocolCode.RESPONSE
                                    + ProtocolCode.separator_section
                                    + Boolean.toString(active);
                            sendReplyMsg(reply);
                            break;
                        default:
                            query_error("Unknown Command");
                    }
                    break;
                case ProtocolCode.CTRL_CART:
                    if (data.length < 4) {
                        query_error("No data");
                        return;
                    }
                    char param_type = data[2].charAt(0);
                    CARTESIAN_AXIS axis = ProtocolCode.dof_char2enum.get(data[3].charAt(0));
                    switch (param_type) {
                        case ProtocolCode.DAMPING:
                            double[] dampings = param_server.get_cart_dampings(axis);
                            reply = ProtocolCode.RESPONSE
                                    + ProtocolCode.separator_section
                                    + Arrays.stream(dampings).mapToObj(x -> String.format("%.3f", x)).collect(
                                            Collectors.joining(ProtocolCode.separator_section));
                            sendReplyMsg(reply);
                            break;
                        case ProtocolCode.STIFFNESS:
                            double[] stiffnesses = param_server.get_cart_stiffnesses(axis);
                            reply = ProtocolCode.RESPONSE
                                    + ProtocolCode.separator_section
                                    + Arrays.stream(stiffnesses).mapToObj(x -> String.format("%.3f", x)).collect(
                                            Collectors.joining(ProtocolCode.separator_section));
                            sendReplyMsg(reply);
                            break;
                        case ProtocolCode.ADDITIONAL_FORCE:
                            double[] forces = param_server.get_additional_forces(axis);
                            reply = ProtocolCode.RESPONSE
                                    + ProtocolCode.separator_section
                                    + Arrays.stream(forces).mapToObj(x -> String.format("%.3f", x)).collect(
                                            Collectors.joining(ProtocolCode.separator_section));
                            sendReplyMsg(reply);
                            break;
                        default:
                            query_error("Unknown Command");
                    }
                    break;
                case ProtocolCode.CTRL_JNT:
                case ProtocolCode.CTRL_POS:
                case ProtocolCode.TOOL:
                    query_error("Not Implemented");
                    break;

                default:
                    query_error("Unknown Command");
                    break;
            }
        } else {
            query_error("Unknown Symbol");
            return;
        }

        if (restart_fri) {
            stop_motion();
        }
    }

    // RoboticsAPIApplication
    @Override
    public void initialize() throws Exception {
        param_server = new ParameterServer(lbr, getLogger());

        // UI
        sceneGraph.clean();
        // User Key // Sunrise OS 16.25
        IUserKeyBar keybar = applicationUi.createUserKeyBar("FRI Server");
        IUserKey showFRIParamsKey = keybar.addUserKey(0, new IUserKeyListener() {
            @Override
            public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                show_params();
                // key.setLED(UserKeyAlignment.BOTTOM_MIDDLE, UserKeyLED.GREEN,
                // UserKeyLEDSize.SMALL);
            }
        }, true);
        showFRIParamsKey.setText(UserKeyAlignment.TOP_LEFT, "Params");
        IUserKey moveToStartKey = keybar.addUserKey(1, new IUserKeyListener() {
            @Override
            public void onKeyEvent(IUserKey key, UserKeyEvent event) {
                cartesian_eff.move(ptp(lbr.findFrame("KneeRS_Start")).setJointVelocityRel(0.25));
            }
        }, true);
        moveToStartKey.setText(UserKeyAlignment.TOP_LEFT, "Start Pos");
        keybar.publish();

        if (!change_tool("HandTool")) {
            getLogger().error("Can't find tool 'HandTool'");
            cartesian_eff = lbr.getFlange();
        } else {
            cartesian_eff = handTool.getRootFrame(); // TODO: should use child frame
        }

        // Move to Start Position
        // cartesian_eff.move(ptp(lbr.findFrame("KneeRS_Start")).setJointVelocityRel(0.25));

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
            ex.printStackTrace();
            running = false;
            return;
        }
        getLogger().info("Starting Listenning to " + Integer.toString(serverPort));
        running = true;

        // set FRI parameters
        request_user_config();
        // configure the FRI
        configure_fri();

        while (running) {
            // FRI Overlay
            if (fri_motion == null || fri_motion.getState().isFinal()) { // Final: Finished, Canceled, or Error

                // getLogger().info("Starting a new FRI overlay.");
                // // TODO on lbr-stack: auto-restart
                // // set FRI parameters
                // request_user_config();
                // // configure the FRI
                // configure_fri();

                getLogger().info("Starting a new FRI motion.");
                control_mode_ = param_server.set_controller_mode(current_ctrl_mode_idx_);
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

    private void close_fri_session() {
        getLogger().info("Close FRI connection");
        if (null != fri_session_) {
            while (fri_channel_information_.getFRISessionState() == FRISessionState.COMMANDING_ACTIVE) {
                ; // wait
            }
            fri_session_.close();
        }
    }

    @Override
    public void dispose() throws Exception {
        close_fri_session();

        getLogger().info("Close UDP Server");
        if (socket != null) {
            if (!(socket.isClosed()))
                socket.close();
        }

        super.dispose();
    }

    @Override
    public void onApplicationStateChanged(RoboticsAPIApplicationState state) {
        // TODO: set a variable
        getLogger().info("App state -> " + state.toString());
    }
}
