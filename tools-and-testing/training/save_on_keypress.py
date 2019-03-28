#! /usr/bin/python

# rospy for the subscriber
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Instantiate CvBridge
bridge = CvBridge()

imNum = -1
lastImNum = -1

# Keylogger from StackOverflow
# https://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-python-from-the-terminal
import sys
import select
import termios

class KeyPoller():
    def __enter__(self):
        # Save the terminal settings
        self.fd = sys.stdin.fileno()
        self.new_term = termios.tcgetattr(self.fd)
        self.old_term = termios.tcgetattr(self.fd)

        # New terminal setting unbuffered
        self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)

        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)

    def poll(self):
        dr,dw,de = select.select([sys.stdin], [], [], 0)
        if not dr == []:
            return sys.stdin.read(1)
        return None

def image_callback(msg):
    global imNum, lastImNum
    # print("Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        # print imNum
        if imNum != lastImNum:
            fname = 'camera_image%s.jpeg' % imNum
            cv2.imwrite(fname, cv2_img)
            print "Done! Filename %s" % fname
            lastImNum = imNum
        else:
            print ".",
            sys.stdout.flush()

def printHelp():
    print "q: quit"
    print "r/s: save an image"
    print "h: print help"

def main():
    global imNum
    rospy.init_node('save_on_keypress')
    # Define your image topic
    image_topic = "/head_camera/rgb/image_raw"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    with KeyPoller() as keyPoller:
        printHelp()
        while not rospy.is_shutdown():
            key = keyPoller.poll()
            if key == 'q':
                print "Saved %d images." % (imNum + 1)
                rospy.signal_shutdown("Quit command received. Stopping...")
                break
            elif key == 'r' or key == 's':
                imNum = imNum + 1
                print "\nSaving an image...",
                sys.stdout.flush()
            elif key == 'h':
                printHelp()

if __name__ == '__main__':
    main()