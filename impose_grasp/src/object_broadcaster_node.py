#!/usr/bin/env python  

import rospy
from impose_grasp.nodes.object_broadcaster import ObjectBroadcaster
import cv2

if __name__ == '__main__':
    target_obj = rospy.get_param("/target_obj")

    obj_br = ObjectBroadcaster(target_obj)
    cv2.namedWindow("Image Stream", cv2.WINDOW_NORMAL)

    while not rospy.is_shutdown():
        obj_br.broadcast_obj_tf()
        # obj_br.test_broadcaster() 
        rgb = obj_br.rgb
        bbox = obj_br.det.bbox
        if bbox is not None:
            cropped_rgb = rgb[bbox[1]:bbox[3], bbox[0]:bbox[2]]
            # Split the image into individual color channels
            blue_channel, green_channel, red_channel = cv2.split(cropped_rgb)

            # Swap the blue and red channels
            swapped_image = cv2.merge((red_channel, green_channel, blue_channel))
            cv2.imshow("Image Stream", swapped_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    rospy.spin()