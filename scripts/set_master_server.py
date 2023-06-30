#! /usr/bin/env python

import rospy
import argparse
import actionlib
import asyncio
from wled import WLED

import wled_ros.msg


async def set_master(hostname, on=True, brightness=127):
    async with WLED(hostname) as led:
        await led.master(on=on, brightness=brightness)


class SetMasterAction(object):
    _feedback = wled_ros.msg.SetMasterFeedback()
    _result = wled_ros.msg.SetMasterResult()

    def __init__(self, name, hostname):
        self._action_name = name
        self.wled_hostname = hostname
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            wled_ros.msg.SetMasterAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo(
            "%s: Hostname %s, ready for clients" % (self._action_name, hostname)
        )

    def execute_cb(self, goal):
        success = True

        rospy.loginfo(
            "%s: Set master to %s, brightness %s, set brightness? %r"
            % (
                self._action_name,
                goal.on.data,
                goal.brightness,
                goal.set_brightness,
            )
        )

        if not goal.set_brightness:
            asyncio.run(set_master(self.wled_hostname, goal.on.data))
        else:
            asyncio.run(set_master(self.wled_hostname, goal.on.data, goal.brightness))

        # Currently always return success=True
        self._result.success = success
        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)


if __name__ == "__main__":
    # Take WLED hostname as argument
    parser = argparse.ArgumentParser()
    parser.add_argument("--hostname", help="WLED hostname", default="wled-f6dafd.local")
    args, unknown = parser.parse_known_args()

    rospy.init_node("WLEDSetMaster")
    server = SetMasterAction(rospy.get_name(), args.hostname)
    rospy.spin()
