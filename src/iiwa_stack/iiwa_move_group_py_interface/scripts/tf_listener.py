#!/usr/bin/env python
import rospy
import tf2_ros
from gazebo_msgs.msg import LinkStates


def gz_link_state_cb(data):
    """
    A callback function that is executed when a message reaches `/gazebo/link_states` topic
    """
    print('\rReceived: ' +
          str(data.pose[-1].position.x) + ', ' +
          str(data.pose[-1].position.y) + ', ' +
          str(data.pose[-1].position.z))
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def gz_link_states_listener():

    joint_states_sub = rospy.Subscriber('/gazebo/link_states',    # Topic name
                                        LinkStates,                      # Topic type (i.e. data_class)
                                        gz_link_state_cb, queue_size=1)
    rospy.init_node('gazebo_link_states_subscriber',  # Node name
                    anonymous=True)                   # To be able to run multiple listeners at the same time
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


def iiwa_tf_listener():
    rospy.init_node('picknmove_control')  # Node name

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)  # receives tf2 transformations and stores them in tf2_ros.Buffer
    # `tfBuffer` up to 10 sec.

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Get the transform from the source frame to the target frame.
            trans = tf_buffer.lookup_transform('iiwa_link_0',  # target_frame
                                               'iiwa_link_ee',  # source_frame (child frame)
                                               rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print('lookup_trans: {}'.format(trans))

        rate.sleep()


if __name__ == '__main__':
    iiwa_tf_listener()
