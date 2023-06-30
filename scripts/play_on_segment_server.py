#! /usr/bin/env python

import rospy
import argparse
import actionlib
import asyncio
from wled import WLED

import wled_ros.msg


async def play_on_segment(
    hostname,
    duration=None,
    segment_id=0,
    brightness=None,
    palette=None,
    effect=None,
    effect_intensity=None,
    effect_speed=None,
    start=None,
    stop=None,
    color_primary=None,
    color_secondary=None,
    color_tertiary=None,
):
    async with WLED(hostname) as led:
        await led.segment(
            segment_id=segment_id,
            brightness=brightness,
            color_primary=color_primary,
            color_secondary=color_secondary,
            color_tertiary=color_tertiary,
            palette=palette,
            effect=effect,
            intensity=effect_intensity,
            speed=effect_speed,
            start=start,
            stop=stop,
        )
        await led.master(on=True)

        if duration is not None:
            await asyncio.sleep(duration)
            await led.master(on=False)


class PlayOnSegmentAction(object):
    _feedback = wled_ros.msg.PlayOnSegmentFeedback()
    _result = wled_ros.msg.PlayOnSegmentResult()

    def __init__(self, name, hostname):
        self._action_name = name
        self.wled_hostname = hostname
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            wled_ros.msg.PlayOnSegmentAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo("%s: Ready for clients" % self._action_name)

    def execute_cb(self, goal):
        success = True

        rospy.loginfo(
            "%s: Recevied goal: %s"
            % (
                self._action_name,
                goal,
            )
        )

        primary_colour = (
            (
                goal.state.primary_colour.r,
                goal.state.primary_colour.g,
                goal.state.primary_colour.b,
            )
            if goal.state.primary_colour.valid
            else None
        )
        secondary_colour = (
            (
                goal.state.secondary_colour.r,
                goal.state.secondary_colour.g,
                goal.state.secondary_colour.b,
            )
            if goal.state.secondary_colour.valid
            else None
        )
        tertiary_colour = (
            (
                goal.state.tertiary_colour.r,
                goal.state.tertiary_colour.g,
                goal.state.tertiary_colour.b,
            )
            if goal.state.tertiary_colour.valid
            else None
        )

        asyncio.run(
            play_on_segment(
                self.wled_hostname,
                duration=goal.duration if goal.duration > 0 else None,
                segment_id=goal.segment_id,
                brightness=goal.state.brightness if goal.state.brightness > 0 else None,
                palette=goal.state.palette_name if goal.state.palette_name else None,
                effect=goal.state.effect_name,
                effect_intensity=goal.state.effect_intensity,
                effect_speed=goal.state.effect_speed,
                start=goal.state.start if goal.state.start >= 0 else None,
                stop=goal.state.stop if goal.state.stop >= 0 else None,
                color_primary=primary_colour,
                color_secondary=secondary_colour,
                color_tertiary=tertiary_colour,
            )
        )

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

    rospy.init_node("WLEDPlayOnSegment")
    server = PlayOnSegmentAction(rospy.get_name(), args.hostname)
    rospy.spin()
