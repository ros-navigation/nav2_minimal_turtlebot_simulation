# Nav2 Minimum Turtlebot4 Simulation

This is a minimum simulation for the Turtlebot4 for use with Nav2. This is setup to strip out external dependencies and complexities of the iRobot Create3 & TB4 ignition simulation panels to the bare minimum needed for simulating the robot for Nav2's bringup which hopefully will not require changes from distribution-to-distribution.

Includes the Gazebo bridge configuration and launch files to open the robot in a particular simulated world (provided)

TODO
- Simulate robot
- sensors all working in ros
- Test with nav2


WTF

        # RPLIDAR static transforms
        Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0', '0.0',
                'rplidar_link', [robot_name, '/rplidar_link/rplidar']],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

        # OAKD static transform
        # Required for pointcloud. See https://github.com/gazebosim/gz-sensors/issues/239
        Node(
            name='camera_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0',
                '1.5707', '-1.5707', '0',
                'oakd_rgb_camera_optical_frame',
                [robot_name, '/oakd_rgb_camera_frame/rgbd_camera']
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ]
        ),

    ])
  



    # Camera sensor bridge
    oakd_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image' +
             '@sensor_msgs/msg/Image' +
             '[ignition.msgs.Image'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points' +
             '@sensor_msgs/msg/PointCloud2' +
             '[ignition.msgs.PointCloudPacked'],
            ['/world/', world,
             '/model/', robot_name,
             '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo' +
             '[ignition.msgs.CameraInfo'],
            ],
        remappings=[
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'],
             'oakd/rgb/preview/image_raw'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/depth_image'],
             'oakd/rgb/preview/depth'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/points'],
             'oakd/rgb/preview/depth/points'),
            (['/world/', world,
              '/model/', robot_name,
              '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info'],
             'oakd/rgb/preview/camera_info')
            ]
    )
