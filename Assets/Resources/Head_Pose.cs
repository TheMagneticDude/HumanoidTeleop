using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Contains Float64MultiArrayMsg

public class Head_Pose : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "/head_pose";

    // Frequency details
    public float publishMessageInterval = 0.1f; // Publish every 0.1 seconds
    private float timeElapsed;

    void Start()
    {
        // 1. Get the ROS Connection instance
        ros = ROSConnection.GetOrCreateInstance();

        // 2. Register the topic name with the Float64MultiArrayMsg type
        ros.RegisterPublisher<Float64MultiArrayMsg>(topicName);
    }

    void Update()
{
    timeElapsed += Time.deltaTime;

    if (timeElapsed >= publishMessageInterval)
    {
        // 1. Get raw inspector angles (0 to 360 degrees)
        Vector3 rawDegrees = transform.localEulerAngles;

        // 2. Convert 0->360 range into a smooth -180->180 range
        float pitchDeg = Mathf.DeltaAngle(0, rawDegrees.x);
        float yawDeg   = Mathf.DeltaAngle(0, rawDegrees.y);
        float rollDeg  = Mathf.DeltaAngle(0, rawDegrees.z);

        // 3. Convert to Radians (now ranges from -pi to +pi smoothly)
        double pitchRad = pitchDeg * Mathf.Deg2Rad;
        double yawRad   = yawDeg * Mathf.Deg2Rad;
        double rollRad  = rollDeg * Mathf.Deg2Rad;

        // 4. Strict Joint Guard: If your ROS robot joint physically locks at -pi/2 and pi/2 (-1.57 to 1.57)
        // uncomment these lines to clamp the values so they never exceed your robot's limits:
        /*
        float maxLimit = Mathf.PI / 2f; // 1.5708 rad (90 degrees)
        yawRad   = Mathf.Clamp((float)yawRad, -maxLimit, maxLimit);
        pitchRad = Mathf.Clamp((float)pitchRad, -maxLimit, maxLimit);
        rollRad  = Mathf.Clamp((float)rollRad, -maxLimit, maxLimit);
        */

        // 5. Pack and publish
        Float64MultiArrayMsg msg = new Float64MultiArrayMsg();
		//
        msg.data = new double[] { pitchRad, -yawRad, rollRad };
        
        ros.Publish(topicName, msg);

        //Debug.Log($"Smooth Out -> Yaw: {yawRad:F3} | Pitch: {pitchRad:F3} | Roll: {rollRad:F3}");

        timeElapsed = 0f;
    }
}

}
