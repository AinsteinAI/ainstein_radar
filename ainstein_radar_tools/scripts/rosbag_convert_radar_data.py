#!/usr/bin/env/python

import sys
import rosbag
from ainstein_radar_msgs.msg import RadarTarget
from ainstein_radar_msgs.msg import RadarTargetArray

def radar_data_msg(topic, datatype, md5sum, msg_def, header):
    """ Message filter for RadarData message type. 

    Message connection filter which returns True for messages of deprecated type 
    radar_sensor_msgs/RadarData.
    """
    if(datatype=="radar_sensor_msgs/RadarData"):
        return True;
    return False;


def other_msg(topic, datatype, md5sum, msg_def, header):
    """ Message filter for everything EXCEPT RadarData message type. 

    Message connection filter which returns True for messages other than those of deprecated 
    type radar_sensor_msgs/RadarData.
    """
    if(datatype!="radar_sensor_msgs/RadarData"):
        return True;
    return False;


def convert_radar_data(bagfile):
    """ Convert RadarData messages in a bagfile to RadarTargetArray.

    Converts all radar_sensor_msgs/RadarData messages in the input bagfile to raw and tracked 
    ainstein_radar_msgs/RadarTargetArray messages.
    
    Args:
        bagfile (String): Filename of input bagfile to convert.

    Returns:
        (None)

    """
    with rosbag.Bag(bagfile.split('.bag')[0] + '_converted.bag', 'w') as outbag:
        with rosbag.Bag(bagfile) as inbag:
            # Pass all messages other than RadarData through unmodified
            for topic, msg, t in inbag.read_messages(connection_filter=other_msg):
                outbag.write(topic, msg, t)
            
            # Convert all RadarData messages to new ainstein_radar_msgs type    
            for topic, msg, time in inbag.read_messages(connection_filter=radar_data_msg):
                # Copy raw targets
                array_msg_raw = RadarTargetArray()
                array_msg_raw.header = msg.header
                for target in msg.raw_targets:
                    array_msg_raw.targets.append(RadarTarget(target_id=target.target_id,
                                                             snr=target.snr,
                                                             range=target.range,
                                                             speed=target.speed,
                                                             azimuth=target.azimuth,
                                                             elevation=target.elevation-90.0))
                                                             
                    outbag.write(topic + '/raw', array_msg_raw, time)

                # Copy tracked targets
                array_msg_tracked = RadarTargetArray()
                array_msg_tracked.header = msg.header
                for target in msg.tracked_targets:
                    array_msg_tracked.targets.append(RadarTarget(target_id=target.target_id,
                                                             snr=target.snr,
                                                             range=target.range,
                                                             speed=target.speed,
                                                             azimuth=target.azimuth,
                                                                 elevation=target.elevation-90.0))
                                                             
                    outbag.write(topic + '/tracked', array_msg_tracked, time)


    
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("usage: rosbag_convert_radar_data.py bagfile")
    else:
        convert_radar_data(sys.argv[1])
