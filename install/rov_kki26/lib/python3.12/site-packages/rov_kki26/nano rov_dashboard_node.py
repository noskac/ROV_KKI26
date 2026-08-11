#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import threading

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class VideoReceiverNode(Node):
    def __init__(self):
        super().__init__('video_receiver_node')
        
        # ROS 2 Publishers
        self.pub_cam1 = self.create_publisher(Image, '/rov/cam1/image_raw', 10)
        self.pub_cam2 = self.create_publisher(Image, '/rov/cam2/image_raw', 10)
        
        Gst.init(None)
        
        # Setup pipeline
        self.pipe1 = self.create_pipeline(5000, self.on_new_sample_cam1)
        self.pipe2 = self.create_pipeline(5001, self.on_new_sample_cam2)
        
        self.pipe1.set_state(Gst.State.PLAYING)
        self.pipe2.set_state(Gst.State.PLAYING)

        self.loop = GLib.MainLoop()
        self.thread = threading.Thread(target=self.loop.run, daemon=True)
        self.thread.start()
        
        self.get_logger().info("[INFO] Video Receiver Node Aktif! Menunggu stream UDP port 5000 & 5001...")

    def create_pipeline(self, port, callback):
        pipeline_str = (
            f"udpsrc port={port} ! application/x-rtp,encoding-name=H264,payload=96 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
        )
        pipeline = Gst.parse_launch(pipeline_str)
        sink = pipeline.get_by_name("sink")
        sink.connect("new-sample", callback)
        return pipeline

    def cv2_to_ros_msg(self, frame):
        msg = Image()
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def on_new_sample_cam1(self, sink):
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            h = caps.get_structure(0).get_value("height")
            w = caps.get_structure(0).get_value("width")
            
            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            if ok:
                frame = np.ndarray((h, w, 3), buffer=mapinfo.data, dtype=np.uint8)
                msg = self.cv2_to_ros_msg(frame)
                self.pub_cam1.publish(msg)
                buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    def on_new_sample_cam2(self, sink):
        sample = sink.emit("pull-sample")
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            h = caps.get_structure(0).get_value("height")
            w = caps.get_structure(0).get_value("width")
            
            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            if ok:
                frame = np.ndarray((h, w, 3), buffer=mapinfo.data, dtype=np.uint8)
                msg = self.cv2_to_ros_msg(frame)
                self.pub_cam2.publish(msg)
                buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    node = VideoReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipe1.set_state(Gst.State.NULL)
        node.pipe2.set_state(Gst.State.NULL)
        node.loop.quit()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()