#!/usr/bin/env python

import message_filters
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy


class CameraHandleConfig:

    def __init__(self):
        self.initial_topic = rospy.get_param(
            param_name="~/camera_handle/initial_topic",
            default="/cam0/image_raw",
        )
        rospy.loginfo("initial topic: %s", self.initial_topic)

        self.output_topic = rospy.get_param(
            param_name="~/camera_handle/output_topic",
            default="/camera/image_raw",
        )
        rospy.loginfo("output topic: %s", self.output_topic)


class CameraHandle:

    def __init__(self, config=CameraHandleConfig()):
        self.config = config
        self.selected_topic = config.initial_topic
        self.current_subscriber = None

    def run(self):
        # create publisher, to publish frames from selected topic
        self.frames_publisher = rospy.Publisher(
            name=self.config.output_topic,
            data_class=Image,
            queue_size=1,
        )

        # listen to control topic
        rospy.Subscriber(
            name="/camera/handle",
            data_class=String,
            callback=self.on_handle_updated,
            queue_size=1,
        )

    def on_handle_updated(self, selected_topic):
        self.selected_topic = selected_topic.data
        rospy.loginfo("topic selected: %s", selected_topic)
        if self.current_subscriber:
            self.current_subscriber.unregister()

        self.current_subscriber = rospy.Subscriber(
            name=self.selected_topic,
            data_class=Image,
            callback=self.on_frame_received,
            queue_size=10,
        )

    def on_frame_received(self, image):
        if not rospy.is_shutdown():
            self.frames_publisher.publish(image)


def main():
    rospy.init_node(name="camera_handle")
    handle = CameraHandle()
    handle.run()
    rospy.spin()


if __name__ == "__main__":
    main()
