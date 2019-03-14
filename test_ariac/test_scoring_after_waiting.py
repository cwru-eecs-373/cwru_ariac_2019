#!/usr/bin/env python

from __future__ import print_function

import sys
import time

import rospy
import rostest
from test_example_node import ExampleNodeTester


class ScoringTester(ExampleNodeTester):

    def test(self):
        expectedScore = float(sys.argv[1])
        rospy.loginfo('Using expected score of: ' + str(expectedScore))
        self.prepare_tester()

        # Starting the competition will cause products from the order to be spawned on shipping_box_0
        self._test_start_comp()
        time.sleep(1.0)
        self._test_order_reception()

        # Sleep a long time to check that the shipping box is stable
        time.sleep(60)

        # Submit the shipping box
        self._test_submit_shipment(shipment_type='order_0_shipment_0', agv_num=1)
        time.sleep(5.0)

        # Check the score
        self.assertEqual(self.current_comp_score, expectedScore)


if __name__ == '__main__':
    rospy.init_node('test_scoring_after_waiting', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(10.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_after_waiting', ScoringTester, sys.argv)
