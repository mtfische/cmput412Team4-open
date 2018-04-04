#!/usr/bin/python

import rospy
import numpy as np
from kobuki_msgs.msg import Sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
rospy.init_node('play_sound')
pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size = 1)
sound = Sound()
sound.value = 0

soundSource = "/home/malcolm/ar.wav"
soundHandle = SoundClient()
while 1:
    rospy.sleep(1)
    soundHandle.playWave(soundSource)
