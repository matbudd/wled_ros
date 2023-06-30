#! /usr/bin/env python

import rospy
import actionlib
import argparse

import wled_ros.msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--on", help="Turn on", action="store_true", default=False)
    parser.add_argument("--off", help="Turn off", action="store_true", default=False)
    parser.add_argument("--brightness", help="Brightness", type=int)
    args = parser.parse_args()

    rospy.init_node("set_master_client")

    client = actionlib.SimpleActionClient("WLEDSetMaster", wled_ros.msg.SetMasterAction)
    client.wait_for_server()

    if args.brightness is not None:
        if args.brightness > 255 or args.brightness < 0:
            raise ValueError("Brightness must be between 0 and 255")

        goal = wled_ros.msg.SetMasterGoal(on=False, brightness=args.brightness)

    elif args.on:
        goal = wled_ros.msg.SetMasterGoal(on=True)

    elif args.off:
        goal = wled_ros.msg.SetMasterGoal(on=False)

    else:
        raise ValueError("Must specify either --on, --off, or --brightness")

    client.send_goal(goal)

    client.wait_for_result()
    print("Result: " + str(client.get_result()))
