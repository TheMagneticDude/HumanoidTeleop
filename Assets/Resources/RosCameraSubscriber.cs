using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class RosCameraSubscriber : MonoBehaviour
{
    [SerializeField] private string topicName = "camera_stream";
    
    private Texture2D texture2D;
    private Material targetMaterial;
    private bool isTextureInitialized = false;

    void Start()
    {
        // Get the material of the plane this script is attached to
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        if (meshRenderer != null)
        {
            targetMaterial = meshRenderer.material;
        }
        else
        {
            Debug.LogError("RosCameraSubscriber requires a MeshRenderer component on the same GameObject.");
            return;
        }

        // Register the subscriber with the ROS TCP Connector
        ROSConnection.GetOrCreateInstance().Subscribe<ImageMsg>(topicName, ReceiveMessage);
    }

    private void ReceiveMessage(ImageMsg imageMsg)
    {
        // Skip processing if there is no image data
        if (imageMsg.data == null || imageMsg.data.Length == 0) return;

        // Initialize the texture once we know the incoming dimensions
        if (!isTextureInitialized)
        {
            // ROS2 standard RGB8 maps directly to Unity TextureFormat.RGB24
            texture2D = new Texture2D((int)imageMsg.width, (int)imageMsg.height, TextureFormat.RGB24, false);
            targetMaterial.mainTexture = texture2D;
            isTextureInitialized = true;
        }

        // Load raw byte array data into the texture
        texture2D.LoadRawTextureData(imageMsg.data);
        texture2D.Apply();
    }
}
