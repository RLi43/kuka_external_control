package udp_server;

import com.kuka.device.ForceData;
import com.kuka.med.devicemodel.LBRMed;
import com.kuka.roboticsAPI.motionModel.controlModeModel.AbstractMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.PositionControlMode;
import com.kuka.sensitivity.controlmode.CartesianImpedanceControlMode;
import com.kuka.sensitivity.controlmode.JointImpedanceControlMode;
import com.kuka.task.ITaskLogger;
import com.kuka.threading.ThreadUtil;
import java.util.Arrays;

public class ParameterServer {
    // TODO: To Decide
    private static final double MAX_JOINT_STIFFNESS = 5000.0;
    
    private LBRMed lbr;
    private ITaskLogger logger;
    
    // Enum
    // control mode
    public enum CONTROL_MODE {
        POSITION_CONTROL,
        JOINT_IMPEDANCE_CONTROL,
        CARTESIAN_IMPEDANCE_CONTROL
    };

    public enum CARTESIAN_AXIS {
        DOF_X,
        DOF_Y,
        DOF_Z,
        DOF_A,
        DOF_B,
        DOF_C,
        DOF_NULLSPACE,
        DOF_TRANSL,
        DOF_ROT,
        DOF_ALL
    };
    
    // Protocol Code
    
    
    // Data
    private CONTROL_MODE controller_mode;
    // Joint Impedance Controller
    // 0.0 - 1.0, (0.7)
    private double[] joint_dampings = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
    // >= 0, (1000)
    private double[] joint_stiffnesses = {1000, 1000, 1000, 1000, 1000, 1000, 1000};
    // Cartesian Impedance Controller
    // x, y, z, a, b, c, nullspace
    // 0.1 - 1.0 (0.7)
    // NullSpace    0.3 - 1.0 (0.7)
    private double[] cartesian_dampings = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7};
    // Trans    5 - 1500    0.0 - 5000.0 (2000.0)
    // Rot      10 - 90     0.0 - 300.0 (200.0)
    // NullSpace    >=0.0 (0.0)
    private double[] cartesian_stiffnesses = {2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0, 0.0};
    // Max force applied: 30 N
    private double[] additional_forces = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // Motion
    
    // External Force and torque at "flange" // TODO: support for different frames
    private class ExternalForceStamped{
        private double[] force = {0.0, 0.0, 0.0};
        private double[] torque = {0.0, 0.0, 0.0};
        private double force_quality = 0.0;
        private double torque_quality = 0.0;
        private long timestamp = 0;
        public ExternalForceStamped(ForceData data) {
            	timestamp = System.currentTimeMillis();
        	for(int i = 0; i < 3; i++) {
        	    	force[i] = data.getForce().get(i);
        	    	force_quality = Math.max(force_quality, data.getForceInaccuracy().get(i));
            		torque[i] = data.getTorque().get(i);
        	    	torque_quality = Math.max(torque_quality, data.getTorqueInaccuracy().get(i));
        	}
        }
        public double[] as_array() {
            return new double[] {force[0], force[1], force[2], torque[0], torque[1], torque[2]};
        }
    }
    private ExternalForceStamped external_force;
    
    // Motion command activity
    
    // Tools
    private String [] available_tools = {null, "HandTool", "Empty"};
    private static final String tool_flange = "Flange";
    private int current_tool = 0;
    
    // Getter and Setter
    public static String[] get_controller_names() {
	    return getNames(CONTROL_MODE.class);
    }

    public AbstractMotionControlMode set_controller_mode(int selectedButtonIndex) {
        AbstractMotionControlMode control_mode_;
        controller_mode = CONTROL_MODE.values()[selectedButtonIndex];
            switch (controller_mode) {
                case POSITION_CONTROL:
                    control_mode_ = new PositionControlMode();
                    break;
                case JOINT_IMPEDANCE_CONTROL:
                    control_mode_ = new JointImpedanceControlMode(
                	    joint_stiffnesses[0],
                	    joint_stiffnesses[1],
                	    joint_stiffnesses[2],
                	    joint_stiffnesses[3],
                	    joint_stiffnesses[4],
                	    joint_stiffnesses[5],
                	    joint_stiffnesses[6]);
                    ((JointImpedanceControlMode) control_mode_).setDamping(joint_dampings);
                    break;
                case CARTESIAN_IMPEDANCE_CONTROL:
                    control_mode_ = new CartesianImpedanceControlMode();
                    break;
                default:
                    System.out.println("Invalid Selection for control mode " 
                            + Integer.toString(selectedButtonIndex) 
                            + "; Using Position Control instead");
                    controller_mode = CONTROL_MODE.POSITION_CONTROL;
                        control_mode_ = new PositionControlMode();        	
            }
            logger.info("Control mode set to: " + controller_mode.name());
        return control_mode_;
    }

    // Joint Impedance Parameters
    public double [] get_joint_dampings(){
        return joint_dampings;
    }
    public double [] get_joint_stiffnesses(){
        return joint_stiffnesses;
    }
    private boolean joint_damping_check(double damping){
        if(damping > 1.0 || damping < 0.0) return false;
        return true;
    }
    private boolean joint_stiffness_check(double stiffness){
        if(stiffness > MAX_JOINT_STIFFNESS || stiffness < 0.0) return false;
        return true;
    }
    public boolean set_joint_dampings(double [] dampings){
        for(double d: dampings) {
            if(!joint_damping_check(d)) return false;
        }
        if(joint_dampings.length != 7) return false;
        joint_dampings = dampings.clone();
        return true;
    }
    public boolean set_joint_damping(int joint_idx, double damping){
        if(!joint_damping_check(damping)) return false;
        if(joint_idx < 0 || joint_idx > 6) return false;
        joint_dampings[joint_idx] = damping;
        return true;
    }
    public boolean set_joint_stiffnesses(double [] stiffnesses){
        for(double s: stiffnesses) {
            if(!joint_stiffness_check(s)) return false;
        }
        if(stiffnesses.length != 7) return false;
        joint_stiffnesses = stiffnesses.clone();
        return true;
    }
    public boolean set_joint_stiffness(int joint_idx, double stiffness){
        if(!joint_stiffness_check(stiffness)) return false;
        if(joint_idx < 0 || joint_idx > 6) return false;
        joint_stiffnesses[joint_idx] = stiffness;
        return true;
    }

    // Cartesian Imepedance Parameters
    private int cart_index(CARTESIAN_AXIS axis){
        switch (axis) {
            case DOF_X:
                return 0;
            case DOF_Y:
                return 1;
            case DOF_Z:
                return 2;
            case DOF_A:
                return 3;
            case DOF_B:
                return 4;
            case DOF_C:
                return 5;
            case DOF_NULLSPACE:
                return 6;
            default:
                throw new RuntimeException("Unknown Axis");
        }
    }
    public double [] get_cart_dampings(){
        return cartesian_dampings;
    }
    public double [] get_cart_dampings(CARTESIAN_AXIS axis){
        switch (axis) {
            case DOF_ALL:
                return cartesian_dampings;
            case DOF_TRANSL:
                return new double[] {cartesian_dampings[0], cartesian_dampings[1], cartesian_dampings[2]};
            case DOF_ROT:
                return new double[] {cartesian_dampings[3], cartesian_dampings[4], cartesian_dampings[5]};        
            default:
                return new double[] {get_cart_damping(axis)};
        }
    }
    public double get_cart_damping(CARTESIAN_AXIS axis){
        return cartesian_dampings[cart_index(axis)];
    }
    private boolean cart_damping_check(double damping, boolean nullspace){        
        if(damping > 1.0 || damping < 0.1) return false;
        if(nullspace && damping < 0.3) return false;
        return true;
    }
    private boolean cart_damping_check(double damping){
        return cart_damping_check(damping, false);
    }
    public boolean set_cart_damping(CARTESIAN_AXIS axis, double damping){
        if(!cart_damping_check(damping, axis==CARTESIAN_AXIS.DOF_NULLSPACE)) return false;
        cartesian_dampings[cart_index(axis)] = damping;
        return true;
    }
    public boolean set_cart_dampings(CARTESIAN_AXIS axis, double[] dampings){        
        switch (axis) {
            case DOF_ALL:
                if(dampings.length != 7) return false;
                for(int i = 0; i < 6; i++){
                    if(!cart_damping_check(dampings[i])) return false;
                }
                if(!cart_damping_check(dampings[6], true)) return false;
                cartesian_dampings = dampings.clone();
                return true;
            case DOF_TRANSL:            
                if(dampings.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!cart_damping_check(dampings[i])) return false;
                }
                for(int i = 0; i < 3; i++){
                    cartesian_dampings[i] = dampings[i];
                }
                return true;
            case DOF_ROT:
                if(dampings.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!cart_damping_check(dampings[i])) return false;
                }
                for(int i = 0; i < 3; i++){
                    cartesian_dampings[i+3] = dampings[i];
                }
                return true;
            default:
                if(dampings.length != 1) return false;
                if(!cart_damping_check(dampings[0], axis == CARTESIAN_AXIS.DOF_NULLSPACE)) return false;
                cartesian_dampings[cart_index(axis)] = dampings[cart_index(axis)];
                return true;
        }
    }
    public double [] get_cart_stiffnesses(){
        return cartesian_stiffnesses;
    }
    public double [] get_cart_stiffnesses(CARTESIAN_AXIS axis){
        switch (axis) {
            case DOF_ALL:
                return cartesian_stiffnesses;
            case DOF_TRANSL:
                return new double[] {cartesian_stiffnesses[0], cartesian_stiffnesses[1], cartesian_stiffnesses[2]};
            case DOF_ROT:
                return new double[] {cartesian_stiffnesses[3], cartesian_stiffnesses[4], cartesian_stiffnesses[5]};        
            default:
                return new double[] {get_cart_stiffness(axis)};
        }
    }
    public double get_cart_stiffness(CARTESIAN_AXIS axis){
        return cartesian_stiffnesses[cart_index(axis)];
    }
    
    private boolean cart_stiffness_check(double stiffness, CARTESIAN_AXIS axis){  
        if(stiffness < 0.0) return false;
        switch (axis) {
            case DOF_X:
            case DOF_Y:
            case DOF_Z:
            case DOF_TRANSL:
                if(stiffness > 5000.0) return false;
                return true;
            case DOF_A:
            case DOF_B:
            case DOF_C:
            case DOF_ROT:
                if(stiffness > 300.0) return false;
                return true;
            case DOF_NULLSPACE:
                if(stiffness > MAX_JOINT_STIFFNESS) return false;
                return true;
            default:
                throw new RuntimeException("Unknown Axis");
        }
    }
    public boolean set_cart_stiffness(CARTESIAN_AXIS axis, double stiffness){
        if(!cart_stiffness_check(stiffness, axis)) return false;
        cartesian_stiffnesses[cart_index(axis)] = stiffness;
        return true;
    }
    public boolean set_cart_stiffnesses(CARTESIAN_AXIS axis, double[] stiffnesses){        
        switch (axis) {
            case DOF_ALL:
                if(stiffnesses.length != 7) return false;
                for(int i = 0; i < 7; i++){
                    if(!cart_stiffness_check(stiffnesses[i], CARTESIAN_AXIS.values()[i])) return false;
                }
                cartesian_stiffnesses = stiffnesses.clone();
                return true;
            case DOF_TRANSL:            
                if(stiffnesses.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!cart_stiffness_check(stiffnesses[i], CARTESIAN_AXIS.values()[i])) return false;
                }
                for(int i = 0; i < 3; i++){
                    cartesian_stiffnesses[i] = stiffnesses[i];
                }
                return true;
            case DOF_ROT:
                if(stiffnesses.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!cart_stiffness_check(stiffnesses[i], CARTESIAN_AXIS.values()[i])) return false;
                }
                for(int i = 0; i < 3; i++){
                    cartesian_stiffnesses[i+3] = stiffnesses[i];
                }
                return true;
            default:
                if(stiffnesses.length != 1) return false;
                if(!cart_stiffness_check(stiffnesses[0], axis)) return false;
                cartesian_stiffnesses[cart_index(axis)] = stiffnesses[cart_index(axis)];
                return true;
        }
    }
    public double[] get_additional_forces(){
        return additional_forces;
    }
    public double get_additional_force(CARTESIAN_AXIS axis){
        if(axis == CARTESIAN_AXIS.DOF_NULLSPACE) throw new RuntimeException("Invalid Axis");
        return additional_forces[cart_index(axis)];
    }
    private boolean check_additional_force(double force, CARTESIAN_AXIS axis){
        if(force < 0.0) return false;
        switch (axis) {
            case DOF_X:
            case DOF_Y:
            case DOF_Z:
            case DOF_TRANSL:
                if(force > 30.0) return false; // TODO: max_control_force param
                return true;
            case DOF_A:
            case DOF_B:
            case DOF_C:
            case DOF_ROT:
                if(force > 20.0) return false; // TODO:
                return true;
            default:
                throw new RuntimeException("Invalid Axis");
        }
    }
    public boolean set_additional_forces(CARTESIAN_AXIS axis, double[] forces){
        switch (axis) {
            case DOF_ALL:
                if(forces.length != 6) return false;
                for(int i = 0; i < 6; i++){
                    if(!check_additional_force(forces[i], CARTESIAN_AXIS.values()[i])) return false;
                }
                for(int i = 0; i < 6; i++){
                    additional_forces[i] = forces[i];
                }
                return true;
            case DOF_TRANSL:
                if(forces.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!check_additional_force(forces[i], CARTESIAN_AXIS.DOF_TRANSL)) return false;
                }
                for(int i = 0; i < 3; i++){
                    additional_forces[i] = forces[i];
                }
                return true;
            case DOF_ROT:
                if(forces.length != 3) return false;
                for(int i = 0; i < 3; i++){
                    if(!check_additional_force(forces[i], CARTESIAN_AXIS.DOF_ROT)) return false;
                }
                for(int i = 0; i < 3; i++){
                    additional_forces[i+3] = forces[i];
                }
                return true;
            case DOF_NULLSPACE:
                throw new RuntimeException("Invalid Axis");
            case DOF_X:
            case DOF_Y:
            case DOF_Z:
            case DOF_A:
            case DOF_B:
            case DOF_C:
                if(forces.length != 1) return false;
                if(!check_additional_force(forces[0], axis)) return false;
                additional_forces[cart_index(axis)] = forces[0];
                return true;
            default:
                throw new RuntimeException("Unknown Axis");
        }
    }
    public boolean set_additional_force(CARTESIAN_AXIS axis, double force){
        switch (axis) {
            case DOF_X:
            case DOF_Y:
            case DOF_Z:
            case DOF_A:
            case DOF_B:
            case DOF_C:
                if(!check_additional_force(force, axis)) return false;
                additional_forces[cart_index(axis)] = force;
                return true;        
            default:
                throw new RuntimeException("Invalid Axis");
        }
    }

    public void show_controller_params(){
        logger.info(controller_mode.toString());
        ThreadUtil.milliSleep(100);
        String msg = new String("Controller Info: ");
        switch (controller_mode) {
            case POSITION_CONTROL:
                msg += "Position Control";
                break;
            case JOINT_IMPEDANCE_CONTROL:
                msg += "Joint Impedance";
                msg += " | Damping: " ;
                for(double d: joint_dampings){
                    msg += String.format("%.3f", d) + ", ";
                }
                msg += " | Stiffness: ";      
                for(double s: joint_stiffnesses){
                    msg += String.format("%.3f", s) + ", ";
                }  
                break;
            case CARTESIAN_IMPEDANCE_CONTROL:
                msg += "Cartesian Impedance";
                msg += " | Damping: " ;
                for(double d: cartesian_dampings){
                    msg += String.format("%.3f", d) + ", ";
                }
                msg += " | Stiffness: ";      
                for(double s: cartesian_stiffnesses){
                    msg += String.format("%.3f", s) + ", ";
                }  
                msg += " | Additional Force: ";      
                for(double a: additional_forces){
                    msg += String.format("%.1f", a) + ", ";
                }  
                break;
            default:
                throw new RuntimeException("Unknown controller mode");
        }
        logger.info(msg);
    }
    
    public boolean get_motion_command_activity() {
        // The request does not provide any information as to whether the robot is
        // currently in motion:
        //    � If the request returns the value �false� (no motion command active),
        //    this does not necessarily mean that the robot is stationary. For example,
        //    robot activity may be checked directly after a synchronous motion
        //    command with a break condition. If the break condition occurs, the
        //    check supplies the value �false� if the robot is braked and moving.
        //    � If the request returns the value �true� (motion command active), this
        //    does not necessarily mean that the robot is moving. For example, the
        //    request returns the value �true� if a robot executes the motion command
        //    positionHold(�) and is stationary.
        return lbr.hasActiveMotionCommand();
    }
    
    public double [] get_external_force() {
        // TODO: other tool eef frame?
        ForceData data = lbr.getExternalForceTorque(lbr.getFlange());
        external_force = new ExternalForceStamped(data);
        return external_force.as_array();
    }

    public String get_current_tool(){
        String tool_name = available_tools[current_tool];
        if(tool_name == null){
            tool_name = tool_flange;
        }
        return tool_name;
    }
    public boolean set_tool_by_index(int idx){
        if(idx < 0 || idx >= available_tools.length) return false;
        current_tool = idx;
        return true;
    }
    
    // Helper
    public static String[] getNames(Class<? extends Enum<?>> e) {
        return Arrays.toString(e.getEnumConstants()).replaceAll("^.|.$", "").split(", ");
    }
    
    public ParameterServer(LBRMed lbr_, ITaskLogger _logger) {
        lbr = lbr_;
        logger = _logger;
    }
}
