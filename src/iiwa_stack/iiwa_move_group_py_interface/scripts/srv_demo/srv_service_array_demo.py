#!/usr/bin/env python3

from __future__ import print_function

from iiwa_picknmove_control.srv import AddTwoFloatArrays, AddTwoFloatArraysResponse
import rospy
import numpy as np


def handle_add_float_arrays(req):
    ku_ee_pos = np.array(req.ku_ee_pos)
    ku_obj_pos = np.array(req.ku_obj_pos)
    print('req.ku_eep_pos type: {}'.format(type(ku_ee_pos)))
    print("Returning [%s + %s = %s]"%(ku_ee_pos,
                                      ku_obj_pos,
                                      (ku_ee_pos + ku_obj_pos)))

    return AddTwoFloatArraysResponse(ku_ee_pos + ku_obj_pos)


def add_float_arrays_server():
    rospy.init_node('add_float_arrays_server')
    s = rospy.Service('add_float_arrays', AddTwoFloatArrays, handle_add_float_arrays)
    print("Ready to add two array of floats.")
    rospy.spin()


if __name__ == "__main__":
    add_float_arrays_server()

