#! /usr/bin/env python

import rospy
import actionlib
import argparse

import wled_ros.msg


def colour_arg_to_msg(colour_arg):
    if colour_arg is None:
        return wled_ros.msg.Colour(valid=False)

    if len(colour_arg) != 3:
        raise ValueError("Colour must be a list of three values")

    return wled_ros.msg.Colour(
        r=colour_arg[0], g=colour_arg[1], b=colour_arg[2], valid=True
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("--segment-id", help="Segment ID", type=int, default=0)
    parser.add_argument(
        "--duration", help="Duration. Default to forever", type=float, default=-1
    )
    parser.add_argument(
        "--brightness", help="Brightness. Default to 0 (no change)", type=int, default=0
    )
    parser.add_argument(
        "--start",
        help="Start. Default to no change",
        type=int,
        default=-1,
    )
    parser.add_argument(
        "--stop",
        help="Stop. Default to no change",
        type=int,
        default=-1,
    )

    parser.add_argument(
        "--effect-name",
        help="Effect name. Default to 'Solid'",
        type=str,
        default="Solid",
    )
    parser.add_argument(
        "--effect-intensity",
        help="Effect intensity. Default to 128",
        type=int,
        default=128,
    )
    parser.add_argument(
        "--effect-speed",
        help="Effect speed. Default to 128",
        type=int,
        default=128,
    )

    # Colours
    parser.add_argument(
        "--primary-colour",
        help="Primary colour. Default to no change",
        type=int,
        nargs=3,
    )
    parser.add_argument(
        "--secondary-colour",
        help="Secondary colour. Default to no change",
        type=int,
        nargs=3,
    )
    parser.add_argument(
        "--tertiary-colour",
        help="Tertiary colour. Default to no change",
        type=int,
        nargs=3,
    )

    args = parser.parse_args()

    rospy.init_node("play_on_segment_client")

    client = actionlib.SimpleActionClient(
        "WLEDPlayOnSegment", wled_ros.msg.PlayOnSegmentAction
    )
    client.wait_for_server()

    if args.brightness > 255 or args.brightness < 0:
        raise ValueError("Brightness must be between 0 and 255")

    goal = wled_ros.msg.PlayOnSegmentGoal(
        segment_id=args.segment_id,
        duration=args.duration,
        state=wled_ros.msg.SegmentState(
            brightness=args.brightness,
            start=args.start,
            stop=args.stop,
            effect_name=args.effect_name,
            effect_intensity=args.effect_intensity,
            effect_speed=args.effect_speed,
            primary_colour=colour_arg_to_msg(args.primary_colour),
            secondary_colour=colour_arg_to_msg(args.secondary_colour),
            tertiary_colour=colour_arg_to_msg(args.tertiary_colour),
        ),
    )

    client.send_goal(goal)

    client.wait_for_result()
    print("Result: " + str(client.get_result()))
