# Unity Visualization and Interaction - Solutions

## Solution 4.1: Unity Environment Setup for Robotics

**Setting up Unity project with robotics packages:**

1. **Unity Installation:**
   - Download Unity Hub from unity.com
   - Install Unity 2021.3 LTS or newer
   - Create a new 3D project

2. **Installing ROS-TCP-Connector:**
   ```bash
   # In Unity Package Manager
   # Window > Package Manager > Add package from git URL
   # https://github.com/Unity-Technologies/ROS-TCP-Connector.git
   ```

3. **Basic robot visualization scene:**
   ```csharp
   using UnityEngine;
   using Unity.Robotics.ROSTCPConnector;

   public class RobotVisualization : MonoBehaviour
   {
       public GameObject robotModel;
       private ROSConnection ros;

       void Start()
       {
           ros = ROSConnection.GetOrCreateInstance();
           ros.Initialize("127.0.0.1", 10000);

           // Subscribe to robot state messages
           ros.Subscribe<JointState>("/joint_states", OnJointStateReceived);
       }

       void OnJointStateReceived(JointState jointState)
       {
           // Update robot model based on joint positions
           UpdateRobotModel(jointState);
       }
   }
   ```

4. **Test connection:**
   ```csharp
   // Publisher example
   void SendTestMessage()
   {
       var testMsg = new StringMessage("Hello ROS from Unity!");
       ros.Send("test_topic", testMsg);
   }
   ```

## Solution 4.2: Robot Model Integration

**Importing and setting up robot model:**

1. **CAD Model Preparation:**
   ```bash
   # Convert URDF to COLLADA (DAE) format
   urdf_to_collada robot.urdf robot.dae

   # Or export directly from CAD software as FBX
   ```

2. **Unity Import Settings:**
   ```csharp
   // Robot hierarchy setup
   public class RobotModelSetup : MonoBehaviour
   {
       [Header("Robot Links")]
       public Transform baseLink;
       public Transform torso;
       public Transform leftArm;
       public Transform rightArm;
       public Transform head;

       [Header("Joint Parameters")]
       public float[] jointLimitsMin;
       public float[] jointLimitsMax;

       void Start()
       {
           SetupRobotHierarchy();
           InitializeJointControllers();
       }

       void SetupRobotHierarchy()
       {
           // Ensure proper parent-child relationships
           torso.SetParent(baseLink);
           leftArm.SetParent(torso);
           rightArm.SetParent(torso);
           head.SetParent(torso);
       }

       void InitializeJointControllers()
       {
           // Set up joint constraints and limits
           foreach(Transform joint in GetComponentsInChildren<Transform>())
           {
               var jointComponent = joint.GetComponent<ConfigurableJoint>();
               if (jointComponent != null)
               {
                   ConfigureJointLimits(jointComponent);
               }
           }
       }
   }
   ```

3. **Scale and Units:**
   ```csharp
   // Ensure proper scaling (meters for robotics)
   public class RobotScaler : MonoBehaviour
   {
       public float scaleFactor = 1.0f; // Unity units = meters

       void Start()
       {
           transform.localScale = Vector3.one * scaleFactor;
       }
   }
   ```

## Solution 4.3: ROS-Unity Communication Implementation

**Bidirectional ROS-Unity communication:**

1. **ROS Node Setup:**
   ```python
   #!/usr/bin/env python3
   import rospy
   from sensor_msgs.msg import JointState
   from geometry_msgs.msg import Twist

   class RobotController:
       def __init__(self):
           rospy.init_node('unity_robot_controller')

           # Publishers
           self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
           self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

           # Subscribers
           rospy.Subscriber('/unity_cmd_vel', Twist, self.velocity_callback)

           self.rate = rospy.Rate(50)  # 50 Hz
           self.joint_positions = [0.0] * 6  # Example 6DOF robot

       def velocity_callback(self, msg):
           # Process velocity commands from Unity
           linear_vel = msg.linear.x
           angular_vel = msg.angular.z
           print(f"Received velocity command: {linear_vel}, {angular_vel}")

       def publish_joint_states(self):
           msg = JointState()
           msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
           msg.position = self.joint_positions
           msg.header.stamp = rospy.Time.now()
           self.joint_pub.publish(msg)

       def run(self):
           while not rospy.is_shutdown():
               self.publish_joint_states()
               self.rate.sleep()

   if __name__ == '__main__':
       controller = RobotController()
       controller.run()
   ```

2. **Unity ROS Connection:**
   ```csharp
   using UnityEngine;
   using Unity.Robotics.ROSTCPConnector;
   using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor;

   public class ROSUnityBridge : MonoBehaviour
   {
       ROSConnection ros;
       public RobotModel robotModel;

       void Start()
       {
           ros = ROSConnection.GetOrCreateInstance();
           ros.Initialize("127.0.0.1", 10000);

           // Subscribe to joint states
           ros.Subscribe<JointState>("/joint_states", OnJointStateReceived);
       }

       void OnJointStateReceived(JointState jointState)
       {
           // Update Unity robot model
           for (int i = 0; i < jointState.name.Count; i++)
           {
               robotModel.SetJointPosition(jointState.name[i], jointState.position[i]);
           }
       }

       public void SendVelocityCommand(float linear, float angular)
       {
           var twist = new Twist();
           twist.linear = new Vector3(linear, 0, 0);
           twist.angular = new Vector3(0, 0, angular);

           ros.Send("/unity_cmd_vel", twist);
       }
   }
   ```

3. **Connection Error Handling:**
   ```csharp
   public class RobustROSConnection : MonoBehaviour
   {
       ROSConnection ros;
       private bool isConnected = false;
       private float reconnectTimer = 0f;
       private const float RECONNECT_INTERVAL = 5f;

       void Start()
       {
           ConnectToROS();
       }

       void ConnectToROS()
       {
           try
           {
               ros = ROSConnection.GetOrCreateInstance();
               ros.Initialize("127.0.0.1", 10000);
               isConnected = true;
           }
           catch (System.Exception ex)
           {
               Debug.LogError($"Failed to connect to ROS: {ex.Message}");
               isConnected = false;
           }
       }

       void Update()
       {
           if (!isConnected)
           {
               reconnectTimer += Time.deltaTime;
               if (reconnectTimer >= RECONNECT_INTERVAL)
               {
                   ConnectToROS();
                   reconnectTimer = 0f;
               }
           }
       }
   }
   ```