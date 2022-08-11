#!/usr/bin/env python

from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy


class CameraHandleConfig:

    def __init__(self):
        self.source_topics = rospy.get_param(
            param_name="~/camera_handle/source_topics",
            default=[],
        )
        rospy.loginfo("source topics: %s" % self.source_topics)
        assert len(self.source_topics) > 0, \
            "source_topics must not be an empty list"

        self.initial_topic = rospy.get_param(
            param_name="~/camera_handle/initial_topic",
            default=self.source_topics[0],
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

    def run(self):
        # create publisher, to publish frames from selected topic
        rospy.Publisher(
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

        # listen to all specified camera topics
        for topic_name in self.config.source_topics:
            rospy.Subscriber(
                name=topic_name,
                data_class=Image,
                callback=lambda data: self.on_frame_received(
                    image=data,
                    topic_name=topic_name,
                ),
                queue_size=1,
            )

    def on_handle_updated(self, selected_topic):
        rospy.loginfo("topic selected: %s", selected_topic)
        self.selected_topic = selected_topic

    def on_frame_received(self, image, topic_name):
        if topic_name == self.selected_topic:
            if not rospy.is_shutdown():
                rospy.loginfo("republish frame from topic %s to topic %s",
                              self.selected_topic, self.config.output_topic)
                self.frames_publisher.publish(image)


def main():
    rospy.init_node(name="camera_handle")
    handle = CameraHandle()
    handle.run()
    rospy.spin()
