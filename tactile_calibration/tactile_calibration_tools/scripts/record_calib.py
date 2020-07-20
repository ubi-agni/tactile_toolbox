#!/usr/bin/env python
import rospy

import numpy as np
import rosbag
from rospy.rostime import Duration, Time
from tactile_msgs.msg import TactileState
import argparse
from collections import OrderedDict
import yaml
import time
from enum import Enum
from copy import deepcopy
from termios import tcflush, TCIFLUSH
import sys
from select import select
import message_filters
from datetime import datetime

from tactile_calibration_tools.calibration_utils import *

#import curses
#import os


#import curses
#stdscr = curses.initscr()
#curses.noecho()
#stdscr.nodelay(1) # set getch() non-blocking


import matplotlib.pyplot as plt

REF_CALIB_RATIO = -0.0154
REF_CALIB_OFFSET = 53.793
DEFAULT_INPUT_RESOL = 12 # in bits
CHANGE_DETECT_THRESH = 4 # 5 unit of velocity positive or negative
DEFAULT_REPETITION = 2
DEFAULT_DETECT_THRESHOLD = 100 # in raw unit
DEFAULT_KEY_TIMEOUT = 2 # 2 seconds
DEFAULT_RECORDING_DURATION = 10 # 30 seconds
MAX_MESSAGE_STORED = 100000 # 100 seconds at 1 kHz of data rate

# prepare storage
global raw_topic_init, recording_cell, state, args, msgs, ref_topic_init, raw_vec, raw_previous_vec, saved
raw_vec = []
raw_previous_vec = []
raw_topic_init = False
ref_raw_vec = []
tare_vec = []
tare = 0
ref_topic_init = None # if None, the ref topic will not be checked because unused
msgs = []
state = None
calibrate_ref = False

recording_channel = None
count_repetition = 0

args = None
saved = False


class RecordingState(Enum):
    INIT = 1
    DETECT = 2
    CONFIRM_DETECT = 3
    RECORD = 4
    PROCESS = 5
    SAVE = 6

# callback for single topic
def raw_topic_cb(msg):
    global raw_topic_init, state, recording_channel, raw_vec, tare_vec, msgs
    if not raw_topic_init:
        # first message arrived
        raw_topic_init = True
    else:
        if recording_channel is not None:
            m = deepcopy(msg)
            if ref_topic_init is None:  # the ref is part of this message too
                # fetch recorded channel and ref
                if (recording_channel >= 0  and recording_channel <  len(msg.sensors[0].values) and args.ref_channel >= 0 and args.ref_channel < len(msg.sensors[0].values)):
                    if calibrate_ref:
                        ref = calibrate_affine(msg.sensors[0].values[args.ref_channel], args.ref_ratio, args.ref_offset) - (tare if tare is not None else 0)
                    else:
                        ref = msg.sensors[0].values[args.ref_channel]
                    m.sensors[0].values = [msg.sensors[0].values[recording_channel] , ref]
            else:  # the ref is not in this message, # should not happen, shoulb be handled by another callback
                # fetch only recorded channel
                m.sensors[0].values = msg.sensors[0].values[recording_channel]
            msgs.append(m)
            # check max size
            if len(msgs) > MAX_MESSAGE_STORED:
                print "Reached data size limit for raw data, stopping recording"
                recording_channel = None
                state=RecordingState.PROCESS
        else:
            if tare_recording is not None and calibrate_ref:
                if args.ref_channel >= 0 and args.ref_channel < len(msg.sensors[0].values):
                    ref = calibrate_affine(msg.sensors[0].values[args.ref_channel], args.ref_ratio, args.ref_offset)
                    tare_vec.append(ref)
              
            # only store the last values
            raw_vec = msg.sensors[0].values


# callback for time synchronized topics
def raw_ref_topic_cb(rawmsg, refmsg):
    if not raw_topic_init:
        # first message arrived
        raw_topic_init = True
        ref_topic_init = True
    else:
        if recording_channel is not None:
            m = deepcopy(rawmsg)
            values = []
            # fetch recorded channel
            values.append(rawmsg.sensors[0].values[recording_channel])
            # fetch ref channel
            if calibrate_ref:
                values.append(calibrate_affine(refmsg.sensors[0].values[args.ref_channel], args.ref_ratio, args.ref_offset) - tare)
            else:
                values.append(refmsg.sensors[0].values[args.ref_channel])
            m.sensors[0].values = values
            msgs.append(m)
        else:
            # only store the last values
            raw_vec = rawmsg.sensors[0].values

def reset_recording():
    count_repetition = 0
    msgs = []

def save_data(calibration_filename):
    # TODO ask user if suggested name is fine, or let the user provide one, or skip
    with rosbag.Bag(calibration_filename, 'w') as outbag:
        for msg in msgs:
            outbag.write(args.raw_topic, msg, msg.header.stamp)
    print "saved data in file", calibration_filename
    return True

def wait_key_press(timeout):
    tcflush(sys.stdin, TCIFLUSH)
    rlist, wlist, xlist = select([sys.stdin], [], [], timeout)
    if rlist:
       return True
    return False
#def wait_key_press(win, timeout):
    #now = rospy.Time.now()
    # while((rospy.Time.now()-now).to_sec() < timeout):
        # try:
           # key = win.getkey()         
           # return True
        # except Exception as e:
           # # No input   
           # pass   
    # return False

def user_menu():
    print "press c to continue, r to retry, d to detect a new cell, s to save and quit, or q to quit without saving"
    tcflush(sys.stdin, TCIFLUSH)
    #user_choice = stdscr.getch()
    return raw_input("(c/r/d/s/q) ?\n")
   

#def main(win): 
    #win.nodelay(True)
    #key=""
    #win.clear()


if __name__ == "__main__":
   # execute only if run as a script
    parser = argparse.ArgumentParser()

    # args : 1 bagfilename, 2 topic, sensor number, calib number
    parser.add_argument("raw_topic", type=str,
                      help="raw topic to record")
    parser.add_argument("ref_channel", type=int,
                      help="index of the ref channel to calibrate against")
    parser.add_argument("--ref_topic", type=str,
                      help="reference topic if different from raw topic")
    parser.add_argument("--data_channel", type=int,
                      help="force to record the specified data channel")
    parser.add_argument("--ref_tare", type=float, 
                      help="reference tare (substracted from calibrated value only)")
    parser.add_argument("--ref_ratio", type=float, default=REF_CALIB_RATIO,
                      help="reference ratio (indicated on the tool)")
    parser.add_argument("--ref_offset", type=float, default=REF_CALIB_OFFSET,
                      help="reference offset (indicated on the tool)")
    parser.add_argument("--no_tare",  action="store_true", help="deactivate initial tare process")
    parser.add_argument("--plot",  action="store_true", help="show the plots")
    parser.add_argument("--input_resolution", type=int, default=DEFAULT_INPUT_RESOL,
                      help="input resolution in bits")
    parser.add_argument("--repetition", type=int, default=DEFAULT_REPETITION,
                      help="number of repetitions awaited for the push/release movement")
    args = parser.parse_args()

    rospy.init_node('tactile_calibration_recorder', anonymous=True)

    #initialize local vars
    input_range_max = 2**args.input_resolution
    detected_channel = None
    start_recording_time = None
    saved = False
    
    if args.ref_ratio and args.ref_offset:
        calibrate_ref = True

    # prepare state machine
    state = RecordingState.INIT
    rate = rospy.Rate(10) # 10hz
    # prepare publisher of tactile "instruction" state

    # subscribe to raw topic or to raw and ref topics with a time synchronizer
    if args.ref_topic:
        raw_sub = message_filters.Subscriber(args.raw_topic, TactileState)
        ref_sub =  message_filters.Subscriber(args.ref_topic, TactileState)
        ts = message_filters.ApproximateTimeSynchronizer([raw_sub, ref_sub], 10, 0.1, allow_headerless=False)
        #ts = message_filters.TimeSynchronizer([raw_sub, ref_sub], 10)
        ts.registerCallback(raw_ref_topic_cb)
    else:
        raw_sub = rospy.Subscriber(args.raw_topic, TactileState, raw_topic_cb)

    # state machine loop
    while not rospy.is_shutdown():
        now = rospy.get_time()

        # State Init
        if state==RecordingState.INIT:
            # wait for initial data in one or 2 topics
            if (raw_topic_init and (args.ref_topic is None or (args.ref_topic is not None and ref_topic_init))):
                if (args.data_channel):  # cell was already chosen in command line arguments
                    print "cell", args.data_channel," will be recorded, if incorrect press enter to go to detection mode, otherwise wait", str(DEFAULT_KEY_TIMEOUT), "sec"
                    #win.addstr("cell " + str(args.data_channel) +
                    #    " will be recorded, if incorrect press a key to enter detection mode, otherwise wait " + str(DEFAULT_KEY_TIMEOUT) + " sec\n")
        # State Tare
        if state==RecordingState.TARE:
            # activate tare recording
            if not tare_recording:
                
                tare_vec=[]
                tare_recording = True
            
            if len(tare_vec) > DEFAULT_TARE_RECORDINGS:
                # enough samples
                tare_recording = False
                # compute tare from tare_vec
                tare=compute_tare(tare_vec)
                state=RecordingState.NEXTCHANNEL
            #TODO: handle timeout if no data for a while
                    key_pressed = False
                    if wait_key_press(DEFAULT_KEY_TIMEOUT):
                        user_choice = user_menu()
                        if user_choice == 'q':
                            exit(0)
                        if user_choice == 's':
                            state=RecordingState.SAVE
                            key_pressed = True
                        if user_choice == 'r':
                            state=RecordingState.INIT
                            key_pressed = True
                        if user_choice == 'd':
                            raw_previous_vec = []
                            state=RecordingState.DETECT
                            key_pressed = True
                        # any other will not do anything and continue
                    if not key_pressed:
                        print "starting recording, please press the cell with the calibration tool in a push/release motion during the next", str(DEFAULT_RECORDING_DURATION) , " sec, ", args.repetition, "times in a row"
                        print " press enter to interrupt recording..."
                        start_recording_time = rospy.Time.now()
                        recording_cell = detected_cell = args.data_channel
                        state=RecordingState.RECORD
                else:
                    # reset previous values
                    raw_previous_vec = []
                    state=RecordingState.DETECT

        # State Detect next channel
        if state==RecordingState.DETECT:
            # initialize detection
            if len(raw_previous_vec) == 0:
                raw_previous_vec = raw_vec
                print raw_previous_vec 
                detected_channel = None
                # announce detection is underway
                print "Detection in progress, please press the channel to be calibrated or press enter to interrupt"
            # detect changes
            detected_channel = detect_channel_press(raw_previous_vec, raw_vec, args.ref_channel, DEFAULT_DETECT_THRESHOLD)
            if detected_channel is not None:
                state = RecordingState.CONFIRM_DETECT
            
            # check if key pressed to interrupt recording
            if wait_key_press(0.1):
                user_choice = user_menu()
                if user_choice == 'q':
                    # TODO warn a second time, that all recording will be lost ?
                    exit(0)
                if user_choice == 's':
                    state=RecordingState.SAVE
                if user_choice == 'd' or user_choice == 'r':
                    state=RecordingState.DETECT
                    raw_previous_vec=[]
                # any other will continue detection if user_choice == 'c':

        # State Confirm detected channel
        if state==RecordingState.CONFIRM_DETECT:
            if detected_channel is not None:  # channel was chosen
                if wait_key_press(DEFAULT_KEY_TIMEOUT):
                    # key pressed, reset previous values
                    raw_previous_vec = []
                    state=RecordingState.DETECT
                        print "channel", detected_channel,"was detected to be pressed, but was either already recorded, or not in the range, procceed with this cell anyway ?"
                else:
                    print "channel", detected_channel,"was detected to be pressed, if incorrect press enter, otherwise wait", str(DEFAULT_KEY_TIMEOUT), "sec"
                    # no key pressed, start recording
                    # print "starting recording, please press the cell with the calibration tool in a push/release motion", args.repetition, "times in a row"
                    start_recording_time = rospy.Time.now()
                    state=RecordingState.RECORD
                # if confirmed
            else: # this should not happen, reset
                state=RecordingState.DETECT
            # else

        # State Record channel
        if state==RecordingState.RECORD:
            if recording_channel is None:  # initialize recording
                recording_channel = detected_channel  # actually starts the recording of frames in the callback
                print " press enter to interrupt recording..."
            else: # we are recording
                # TODO analyse the last recorded values and detect push/release
                # if count_repetition >= args.repetition:
                # for now we just use a time and stop after a certain time
                if (rospy.Time.now()-start_recording_time).to_sec() > DEFAULT_RECORDING_DURATION :
                    recording_channel = None
                    state=RecordingState.PROCESS

            # check if key pressed to interrupt recording
            if wait_key_press(0.1):
                # key pressed
                # stop recording
                recording_channel =  None
                print "recording stopped"
                already_recorded_duration = rospy.Time.now() - start_recording_time
                user_choice = user_menu()
                if user_choice == 'q':
                    # TODO warn a second time, that all recording will be lost ?
                    exit(0)
                if user_choice == 's':
                    state=RecordingState.SAVE
                if user_choice == 'd':
                    state=RecordingState.DETECT
                    raw_previous_vec=[]
                if user_choice == 'r':
                    reset_recording()
                    print "restarting recording, please press the cell with the calibration tool in a push/release motion during the next", str(DEFAULT_RECORDING_DURATION) , " sec, ", args.repetition, "times in a row"
                    start_recording_time = rospy.Time.now()
                    state=RecordingState.RECORD
                if user_choice == 'c':
                    start_recording_time = rospy.Time.now() - already_recorded_duration
                    print "continuing recording, for ", str(round(DEFAULT_RECORDING_DURATION-already_recorded_duration.to_sec(),1)) , " sec"
                    
            # else:
            # TODO display pressure

        # State Process recording
        if state==RecordingState.PROCESS:
            # check if data is valid, otherwise ask for recording again with some instruction how to improve
            ## basic data size check
            ## push/release check

            # data is valid
            state=RecordingState.SAVE

        # State Save recording
        if state==RecordingState.SAVE:
            date_time_obj = datetime.now()
            date_time = date_time_obj.strftime("%Y-%m-%d-%H-%M-%S")
            saved=save_data("calib_" + str(detected_channel) + "_" + date_time + ".bag")
            
            break;

        rate.sleep()
    # check if we saved the data or not
    if not saved:
        # save here
        date_time_obj = datetime.now()
        date_time = date_time_obj.strftime("%Y-%m-%d-%H-%M-%S")
        saved=save_data("calib_" + str(detected_cell) + "_" + date_time + ".bag")



