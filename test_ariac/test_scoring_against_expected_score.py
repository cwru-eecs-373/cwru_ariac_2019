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
        if len(sys.argv) > 2:
            self.agv2_first = sys.argv[2] == '--agv-2-first'
        self.prepare_tester()

        # Starting the competition will cause products from the order to be spawned on shipping_box_0
        self._test_start_comp()

        if self.agv2_first:
            # Submit the tray on agv2
            self._test_submit_shipment(shipment_type='order_0_shipment_0', agv_num=2)
            time.sleep(5.0)
            self._test_submit_shipment(shipment_type='order_0_shipment_1', agv_num=1)
            time.sleep(5.0)
        else:
            # Submit the tray on shipping_box_0
            self._test_submit_shipment(shipment_type='order_0_shipment_0', agv_num=1)
            time.sleep(5.0)
            # Submit the tray on shipping_box_1
            self._test_submit_shipment(shipment_type='order_0_shipment_1', agv_num=2)
            time.sleep(5.0)

        self.assertEqual(self.current_comp_score, expectedScore)


if __name__ == '__main__':
    rospy.init_node('test_scoring_against_expected_score', anonymous=True)

    # Wait until /clock is being published; this can take an unpredictable
    # amount of time when we're downloading models.
    while rospy.Time.now().to_sec() == 0.0:
        print('Waiting for Gazebo to start...')
        time.sleep(1.0)
    # Take an extra nap, to allow plugins to be loaded
    time.sleep(20.0)
    print('OK, starting test.')

    rostest.run('test_ariac', 'test_scoring_against_expected_score', ScoringTester, sys.argv)
