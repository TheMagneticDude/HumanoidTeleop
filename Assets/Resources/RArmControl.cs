using UnityEngine;
using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using UnityEngine.XR.Hands; 

public class RArmControl : MonoBehaviour
{
    [Header("VR Tracking References")]
    public Transform vrHeadset;     
    public Transform trackingOrigin; 

    private ROSConnection ros;
    public string topicName = "/R_arm_pose";
    private float timeElapsed;
    public float publishMessageInterval = 0.1f; 

    [Header("Robot Dimensions (Meters)")]
    public float L1 = 0.253001f; // Shoulder to Elbow 
    public float L2 = 0.364296f; // Elbow to Wrist 

    [Header("IK Scaling")]
    public float trackingScale = 1.0f;

    [Header("Isaac Shoulder Offset from Head")]
    public Vector3 isaacshoulderOffset = new Vector3(-0.23f, -0.45f, 0.14f); 
	
	[Header("IRL Shoulder Offset from Head")]
	public Vector3 shoulderOffset = new Vector3(-0.18f, -0.25f, -0.05f);
    
    private double[] restPoseRad = new double[7] { 0.0, 0.0, 0.0, -5.0 * Mathf.Deg2Rad, 0.0, 0.0, 0.0 };
    private double[] jointCommandRad = new double[7];

    private XRHandSubsystem m_HandsSubsystem;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Float64MultiArrayMsg>(topicName);
    }
    
    void Update()
    {
        if (m_HandsSubsystem == null)
        {
            var subsystems = new List<XRHandSubsystem>();
            SubsystemManager.GetSubsystems(subsystems);
            if (subsystems.Count > 0) m_HandsSubsystem = subsystems[0];
        }

        timeElapsed += Time.deltaTime;
        
        if (timeElapsed >= publishMessageInterval)
        {
            if (TryGetTargetDistance(out Vector3 targetPos))
            {
                CalculateAndSendIK(targetPos);
            }
            else
            {
                SendRestPose();
            }
            timeElapsed = 0f;
        }
    }
    
    private bool TryGetTargetDistance(out Vector3 isaacTarget)
    {
        isaacTarget = Vector3.zero;

        if (vrHeadset == null || m_HandsSubsystem == null || !m_HandsSubsystem.running) return false;

        var rightHand = m_HandsSubsystem.rightHand;
        if (!rightHand.isTracked) return false;

        var wristJoint = rightHand.GetJoint(XRHandJointID.Wrist);
        if (wristJoint.TryGetPose(out Pose rawPose))
        {
            // 1. Establish the Torso/Chest Rotation
            Vector3 torsoForward = Vector3.ProjectOnPlane(vrHeadset.forward, Vector3.up).normalized;
            if (torsoForward == Vector3.zero) torsoForward = Vector3.forward;
            Quaternion torsoRot = Quaternion.LookRotation(torsoForward, Vector3.up);

            // 2. Get Absolute World Positions
            Vector3 wristWorldPos = (trackingOrigin != null) 
                                    ? trackingOrigin.TransformPoint(rawPose.position) 
                                    : rawPose.position;

            Vector3 shoulderWorldPos = vrHeadset.position + (torsoRot * shoulderOffset);

            // 3. CRITICAL: Convert World Offset into Local Torso Space!
            // This strips away the room rotation so "Forward" is always relative to your chest
            Vector3 localOffset = Quaternion.Inverse(torsoRot) * (wristWorldPos - shoulderWorldPos) * trackingScale;

            // 4. Map Local Unity to Isaac Sim Coordinates
            // Unity +Z (Forward) -> Isaac +X (Forward)
            // Unity +X (Right)   -> Isaac -Y (Left arm positive is Left)
            // Unity +Y (Up)      -> Isaac +Z (Up)
            isaacTarget = new Vector3(localOffset.z, -localOffset.x, localOffset.y);
            
            return true;
        }
        
        return false;
    }
    
    private void CalculateAndSendIK(Vector3 target)
    {
        float TX = target.x; // Forward
        float TY = target.y; // Left
        float TZ = target.z; // Up
        
        float r = Mathf.Sqrt(TX * TX + TY * TY);
        float theta = Mathf.Atan2(TY, TX); 
        float z = TZ;
        
        float h = Mathf.Sqrt(r * r + z * z);
        h = Mathf.Clamp(h, 0.01f, L1 + L2 - 0.001f);
        
        float cosElbow = (L1 * L1 + L2 * L2 - h * h) / (2f * L1 * L2);
        float innerElbowRad = Mathf.Acos(Mathf.Clamp(cosElbow, -1f, 1f));
        
        float urdfElbowRad = innerElbowRad;// - Mathf.PI;
        
        float gamma = Mathf.Atan2(z, r); 
        float alpha = Mathf.Acos(Mathf.Clamp((L1 * L1 + h * h - L2 * L2) / (2f * L1 * h), -1f, 1f)); 
        
        float phi = gamma - alpha; 
        
        float r_e = L1 * Mathf.Cos(phi);
        float z_e = L1 * Mathf.Sin(phi);
        
        float EX = r_e * Mathf.Cos(theta);
        float EY = r_e * Mathf.Sin(theta);
        float EZ = z_e; 
        
        float shoulderRoll = Mathf.Asin(Mathf.Clamp(EY / L1, -1f, 1f));
        float shoulderPitch = Mathf.Atan2(EX, -EZ);
        
        float shoulderYaw = 0f; 

        // Assign to Array
        jointCommandRad[0] = Mathf.Clamp(shoulderPitch, 180f * Mathf.Deg2Rad, -45f * Mathf.Deg2Rad); 
        jointCommandRad[1] = Mathf.Clamp(shoulderRoll, 0.0f, 99.999f * Mathf.Deg2Rad);
        jointCommandRad[2] = shoulderYaw;
        
        jointCommandRad[3] = Mathf.Clamp(urdfElbowRad, 130f * Mathf.Deg2Rad, 0f); 
        
        jointCommandRad[4] = 0f;
        jointCommandRad[5] = 0f;
        jointCommandRad[6] = 0f;

        PublishMessage();
    }

    private void SendRestPose()
    {
        for (int i = 0; i < 7; i++)
        {
            jointCommandRad[i] = restPoseRad[i];
        }
        PublishMessage();
    }

    private void PublishMessage()
    {
        Float64MultiArrayMsg msg = new Float64MultiArrayMsg();
        msg.data = new double[] 
        { 
            jointCommandRad[0], jointCommandRad[1], jointCommandRad[2],
            jointCommandRad[3], jointCommandRad[4], jointCommandRad[5], jointCommandRad[6]
        };

        ros.Publish(topicName, msg);
    }
}