#!/usr/bin/env python
PKG = 'graspit_python_node'

from ros_interface import ROSInterface

import unittest


class TestRosInterface(unittest.TestCase):

    is_setup = False

    def setUp(self):
        if not self.is_setup:
            self.is_setup = True
            self.ros_interface = ROSInterface()

    def test_run_recognition_request(self):
        find_objects_response = self.ros_interface.handle_object_recognition_request()
        self.assertEqual(len(find_objects_response.object_name), 2)
        for i in range(len(find_objects_response.object_name)):
            self.assertEqual(find_objects_response.object_name[i], "all", "object name != all")
            find_objects_response.object_pose[i]

    def test_camera_origin_request(self):

        camera_origin = self.ros_interface.handle_camera_origin_request()

        expected_x = 1
        expected_y = 1
        expected_z = 1

        actual_x = camera_origin[0]
        actual_y = camera_origin[1]
        actual_z = camera_origin[2]

        self.assertEqual(actual_x, expected_x, "camera origin is " + str(actual_x) + "expected: " + expected_x)
        self.assertEqual(actual_y, expected_y, "camera origin is " + str(actual_y) + "expected: " + expected_y)
        self.assertEqual(actual_z, expected_z, "camera origin is " + str(actual_z) + "expected: " + expected_z)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_bare_bones', TestRosInterface)