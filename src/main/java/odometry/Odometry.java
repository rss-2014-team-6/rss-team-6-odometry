package odometry;

import org.ros.message.MessageListener;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;
import rss_msgs.EncoderMsg;
import rss_msgs.OdometryMsg;


public class Odometry extends AbstractNodeMain {

    private int[] prev_ticks; // (left wheel, right wheel, make int[]
    public static final double WHEEL_RADIUS_IN_M = 0.0625;
    public static final double ENCODER_RESOLUTION = 2000;
    public static final double GEAR_RATIO = 65.5;
    public static final double TICKS_PER_REVOLUTION = ENCODER_RESOLUTION
            * GEAR_RATIO;
    public static final double WHEEL_METERS_PER_TICK = WHEEL_RADIUS_IN_M
            * Math.PI * 2 / (TICKS_PER_REVOLUTION);
    public static final double WHEELBASE = .43;
    /**
     * [x][y][theta]
     */
    private double[] pose = { 0, 0, 0 };
    private boolean reset = false;
    private OdometryMsg msg;
    private Publisher<OdometryMsg> pub;
    private Publisher<std_msgs.String> statePub;
    private Subscriber<EncoderMsg> encoderSub;
    
    public void update(int[] new_ticks) {
       
        if ((prev_ticks == null && (new_ticks[0] == 0 && new_ticks[1] == 0))
                || reset) {
            prev_ticks = new_ticks;
            reset = false;
        } else if (prev_ticks == null) {
            prev_ticks = new int[2];
        }
        int[] ticks = new int[2];
        for (int i = 0; i < 2; i++) {
            ticks[i] = new_ticks[i] - prev_ticks[i];
        }

        double s_left = (ticks[0]) * WHEEL_METERS_PER_TICK;
        double s_right = (ticks[1]) * WHEEL_METERS_PER_TICK;
        double theta = (s_left - s_right) / WHEELBASE;

        msg.setTheta(msg.getTheta() - theta);
        if (msg.getTheta() < 0) {
            msg.setTheta(msg.getTheta() + 2 * Math.PI);
        } else if (msg.getTheta() > 2 * Math.PI) {
            msg.setTheta(msg.getTheta() - 2 * Math.PI);
        }
        msg.setX(msg.getX() + (s_left + s_right) * Math.cos(msg.getTheta()) / 2.0);
        msg.setY(msg.getY() + (s_left + s_right) * Math.sin(msg.getTheta()) / 2.0);

	msg.setTime(System.currentTimeMillis());

        prev_ticks[0] = new_ticks[0];
        prev_ticks[1] = new_ticks[1];
        pub.publish(msg);
    }

    @Override
    public void onShutdown(Node node) {
        // TODO Auto-generated method stub

    }

    @Override
    public void onStart(ConnectedNode node) {
        pub = node.newPublisher("/odo/Odometry", "rss_msgs/OdometryMsg");
        msg = pub.newMessage();
        encoderSub = node.newSubscriber("/sense/Encoder", "rss_msgs/EncoderMsg");
        encoderSub.addMessageListener(new EncoderListener(this));

        // Start off by reseting
        reset = true;
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/odometry");
    }
}
