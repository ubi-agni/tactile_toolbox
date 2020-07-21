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
DEFAULT_TARE_RECORDINGS = 1000

# prepare storage
global raw_topic_init, recording_channel, state, args, msgs, ref_topic_init, raw_vec, raw_previous_vec, tare_vec, saved
raw_vec = []
raw_previous_vec = []
raw_topic_init = False
ref_raw_vec = []
tare_vec = []
tare_recording = None
tare = 0
ref_topic_init = None # if None, the ref topic will not be checked because unused
msgs = []
state = None
calibrate_ref = False

recording_channel = None
processed_channels = OrderedDict()
count_repetition = 0

args = None
saved = False


class RecordingState(Enum):
    INIT = 1
    TARE = 2
    NEXTCHANNEL = 3
    DETECT = 4
    CONFIRM_DETECT = 5
    RECORD = 6
    PROCESS = 7
    SAVE = 8
    END = 9

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
    global raw_topic_init, ref_topic_init, state, recording_channel, raw_vec, tare_vec, msgs
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
            
            # check max size
            if len(msgs) > MAX_MESSAGE_STORED:
                print "Reached data size limit for raw data, stopping recording"
                recording_channel = None
                state=RecordingState.PROCESS
        else:
            if tare_recording is not None and calibrate_ref:
                if args.ref_channel >= 0 and args.ref_channel < len(refmsg.sensors[0].values):
                    ref = calibrate_affine(refmsg.sensors[0].values[args.ref_channel], args.ref_ratio, args.ref_offset)
                    tare_vec.append(ref)

            # only store the last values
            raw_vec = rawmsg.sensors[0].values

def reset_recording():
    global count_repetition, msgs
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

def user_menu(choices={'c': "continue", 'r':"retry", 's': "save", 'q': "quit without saving"}):
    letter_string = "("
    for letter in choices:
        letter_string += letter + "/"
        print  letter, "to", choices[letter]
    letter_string += ") ?\n"
    #print "press c to continue, r to retry, d to detect a new channel, s to save and quit, or q to quit without saving"
    tcflush(sys.stdin, TCIFLUSH)
    #user_choice = stdscr.getch()
    return raw_input(letter_string)

def user_yesno(default=True):
    if default:
        text = "[y]/n ?"
    else:
        text = "y/[n] ?"
    tcflush(sys.stdin, TCIFLUSH)
    while(1):
        ret = raw_input(text)
        if ret == "":
            return default
        else:
            if ret == 'y' or ret == 'Y':
                return True
            if ret == 'n' or ret == 'N':
                return False
        print "wrong choice, try again"
        tcflush(sys.stdin, TCIFLUSH)

def move_previous_recording(processed_channels, detected_channel, folder):
    fullfilename = processed_channels[detected_channel]
    filename = os.path.basename(fullfilename)
    dirname = os.path.dirname(os.path.realpath(fullfilename))
    if dirname != "":
        dirname +="/"+folder
    else:
        dirname =folder
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    os.rename(fullfilename, dirname +"/" + filename)
    print "moving", filename, " into folder called ", folder

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
    parser.add_argument("--data_channel", type=int, nargs='+',
                      help="force to record the specified data channel")
    parser.add_argument("--detect_threshold", type=float, default=DEFAULT_DETECT_THRESHOLD,
                      help="number of channels to record (will record 0 to num_channels-1)")
    parser.add_argument("--num_channels", type=int,
                      help="number of channels to record (will record 0 to num_channels-1)")
    parser.add_argument("--ref_tare", type=float,
                      help="reference tare (substracted from calibrated value only)")
    parser.add_argument("--ref_ratio", type=float,  nargs='?', const=REF_CALIB_RATIO,
                      help="reference ratio (indicated on the tool)")
    parser.add_argument("--ref_offset", type=float, nargs='?', const=REF_CALIB_OFFSET,
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
    quit_request = False
    
    if args.ref_ratio and args.ref_offset:
        calibrate_ref = True

    # prepare state machine
    state = RecordingState.INIT
    rate = rospy.Rate(20) # 10hz
    # prepare publisher of tactile "instruction" state

    # subscribe to raw topic or to raw and ref topics with a time synchronizer
    if args.ref_topic:
        print "Initializing topic synchronizer"
        raw_sub = message_filters.Subscriber(args.raw_topic, TactileState)
        ref_sub =  message_filters.Subscriber(args.ref_topic, TactileState)
        ts = message_filters.ApproximateTimeSynchronizer([raw_sub, ref_sub], 10, 0.1, allow_headerless=False)
        #ts = message_filters.TimeSynchronizer([raw_sub, ref_sub], 10)
        ts.registerCallback(raw_ref_topic_cb)
    else:
        print "Initializing topic subscriber"
        raw_sub = rospy.Subscriber(args.raw_topic, TactileState, raw_topic_cb)

    # state machine loop
    print "Started, waiting for data on topic", args.raw_topic
    while not rospy.is_shutdown():
        now = rospy.get_time()

        # State Init
        if state==RecordingState.INIT:
            # wait for initial data in one or 2 topics
            if (raw_topic_init and (args.ref_topic is None or (args.ref_topic is not None and ref_topic_init))):
                print "Found data on topics"
                # handle the channel list
                if (args.data_channel):  # user provided channel(s) in command line arguments
                    channel_list = args.data_channel
                else:  # no provided list
                    # check if a range is given
                    if args.num_channels:
                        channel_list = range(0, args.num_channels)
                        # handle ref in case it is in the same vector
                        if not args.ref_topic:  # remove the ref_channel from the list
                            if args.ref_channel in channel_list:
                                channel_list.remove(args.ref_channel)
                        # else the ref is on a separate topic, use all data channels
                    else: # detect channels on the go, and continue until user stops
                        channel_list = None
                
                # handle if tare is required
                if args.ref_tare:  # use user tare
                    tare = args.ref_tare
                if args.no_tare or args.ref_tare:
                    state=RecordingState.NEXTCHANNEL
                else:
                    if calibrate_ref:
                        print "starting tare recording, keep the calibration tool still in the rest position, press enter when ready"
                        if wait_key_press(10):
                            state=RecordingState.TARE
                        else:
                            print "No key was pressed in 10 seconds, what do you want to do ?"
                            user_choice = user_menu({'t': "tare", 'k': "skip tare", 'q': "quit without saving"})
                            if user_choice == 'q':
                                state=RecordingState.END
                            if user_choice == 'k':
                                state=RecordingState.NEXTCHANNEL
                            if user_choice == 'k':
                                state=RecordingState.TARE
                    else:
                        print "reference cannot be calibrated, so will not be tared"
                        tare = None
                        state=RecordingState.NEXTCHANNEL

        # State Tare
        if state==RecordingState.TARE:
            # activate tare recording
            if not tare_recording:
                tare_vec = []
                tare_recording = True
                print "Tare in progress..."
            
            if len(tare_vec) > DEFAULT_TARE_RECORDINGS:
                # enough samples
                tare_recording = False
                # compute tare from tare_vec
                tare = compute_tare(tare_vec)
                print "tare =", tare
                state=RecordingState.NEXTCHANNEL
            #TODO: handle timeout if no data for a while

        # State Next Channel
        if state==RecordingState.NEXTCHANNEL:
            if channel_list is None: # no list, means ask if continue for a next one 
                print "Do you want to detect a new channel ?"
                user_choice = user_yesno(default=True)
                if user_choice == False:  # end there
                    state=RecordingState.END
                else:
                    raw_previous_vec=[]
                    state=RecordingState.DETECT
                
            else: # there is a list
                if len(channel_list): # if not at the end of the channel list
                    print len(channel_list), "channels remaining to be recorded"
                    print "Proceeding to next channel, press enter to interrupt or wait for", DEFAULT_KEY_TIMEOUT, "seconds"
                    key_pressed = False
                    if wait_key_press(DEFAULT_KEY_TIMEOUT):
                        user_choice = user_menu({'c': "continue to detection", 'q': "quit without saving"})
                        if user_choice == 'q':
                            state=RecordingState.END
                            key_pressed = True
                        if user_choice == 'c':
                            raw_previous_vec = []
                            state=RecordingState.DETECT
                            key_pressed = True
                        # any other will not do anything and continue
                    if not key_pressed:
                        raw_previous_vec = []
                        state=RecordingState.DETECT
                else: # at the end of the list, proceed to quit
                    print "All channels have been recorded, quitting"
                    state=RecordingState.END
  
        # State Detect
        if state==RecordingState.DETECT:
            # initialize detection
            if len(raw_previous_vec) == 0:
                raw_previous_vec = raw_vec
                detected_channel = None
                # announce detection is underway
                print "Selection detection in progress"
                print " please press the channel that is to be calibrated hardware wise."
                print " You can always interrupt by pressing enter."
                print " ", str(DEFAULT_KEY_TIMEOUT) , "seconds after detection, calibration recording starts automaticly."
                print "Then you have another", str(DEFAULT_RECORDING_DURATION) ,"seconds for recording."
                print "Use the calibtool upright and press and release your selected channel evenly over", args.repetition,"iterations. \n\n" 
            # detect changes
            detected_channel = detect_channel_press(raw_previous_vec, raw_vec, args.ref_channel, args.detect_threshold)
            if detected_channel is not None:
                state = RecordingState.CONFIRM_DETECT
            else:
                # check if key pressed to interrupt recording
                if wait_key_press(0.1):
                    user_choice = user_menu({'c': "continue", 'r':"retry detect", 's': "save", 'q': "quit without saving"})
                    if user_choice == 'q':
                        # TODO warn a second time, that all recording will be lost ?
                        state=RecordingState.END
                    if user_choice == 's':
                        state=RecordingState.SAVE
                    if user_choice == 'r':
                        state=RecordingState.DETECT
                        raw_previous_vec=[]
                    # any other will continue detection if user_choice == 'c':

        # State Confirm detected channel
        if state==RecordingState.CONFIRM_DETECT:
            if detected_channel is not None:  # channel was chosen
                print "channel", detected_channel, "was detected"
                # check if already recorded this channel
                if detected_channel in processed_channels:
                    print " this channel was already recorded, what do you want to do ?"
                    user_choice = user_menu({'c': "continue, move the previous recording, and re-record this channel", 'd':"detect a new channel", 'q': "quit"})
                    if user_choice == "" or user_choice == 'c':
                        # move previously recorded calib file for this channel in an old folder
                        # recorded in this session
                        move_previous_recording(processed_channels, detected_channel, "old")
                        # TODO Handle files matching a pattern "calib_ detected_channel_*.bag
                    if user_choice == 'q':
                        state=RecordingState.END
                    if user_choice == 'd':
                        state=RecordingState.DETECT

                if state==RecordingState.CONFIRM_DETECT:  # no change in state = continue
                    if channel_list is not None:  # check if channel is part of the list
                        if detected_channel not in channel_list:
                            print " but this channel not in the channel list"
                            print channel_list
                            print "proceed with this channel anyway ?"
                            if user_yesno(default=False) == False:
                                raw_previous_vec = []
                                state = RecordingState.DETECT
                    else:
                        print " if incorrect press enter, otherwise wait", str(DEFAULT_KEY_TIMEOUT), "sec"
                        if wait_key_press(DEFAULT_KEY_TIMEOUT):
                            # key pressed, reset previous values
                            raw_previous_vec = []
                            state=RecordingState.DETECT
                    if state==RecordingState.CONFIRM_DETECT:  # no change in state = continue with recording
                        # start recording
                        print "starting recording"
                        state=RecordingState.RECORD
                    # if confirmed

            else: # this should not happen, reset
                state=RecordingState.DETECT

        # State Record channel
        if state==RecordingState.RECORD:
            if recording_channel is None:  # initialize recording
                reset_recording()
                start_recording_time = rospy.Time.now()
                recording_channel = detected_channel  # actually starts the recording of frames in the callback
                # print "starting recording, please press the channel with the calibration tool in a push/release motion", args.repetition, "times in a row"
                print " please press the channel with the calibration tool in a push/release motion during the next", str(DEFAULT_RECORDING_DURATION) , " sec, ", args.repetition, "times in a row"
                        
                print " press enter to interrupt recording..."
            else: # we are recording
                # TODO analyse the last recorded values and detect push/release
                # if count_repetition >= args.repetition:
                # for now we just use a time and stop after a certain time
                elapsed_time = (rospy.Time.now()-start_recording_time).to_sec() 
                if elapsed_time > DEFAULT_RECORDING_DURATION :
                    recording_channel = None
                    state=RecordingState.PROCESS
                    print "\nrecording ended"
                else:
                  print "\r Remaining time :", round(DEFAULT_RECORDING_DURATION-elapsed_time),
                  sys.stdout.flush()

            # check if key pressed to interrupt recording
            if wait_key_press(0.1):
                # key pressed
                # stop recording
                recording_channel =  None
                print "recording stopped"
                already_recorded_duration = rospy.Time.now() - start_recording_time
                user_choice = user_menu({'c': "continue recording", 'r':"restart recording", 'd': "detect a new cell", 's': "save now", 'q': "quit without saving"})
                if user_choice == "" or user_choice == 'c':
                    start_recording_time = rospy.Time.now() - already_recorded_duration
                    recording_channel = detected_channel
                    print "continuing recording, for ", str(round(DEFAULT_RECORDING_DURATION-already_recorded_duration.to_sec(),1)) , " sec"
                else:
                    if user_choice == 'q':
                        # TODO warn a second time, that all recording will be lost ?
                        state=RecordingState.END
                    if user_choice == 's':
                        state=RecordingState.SAVE
                    if user_choice == 'd':
                        state=RecordingState.DETECT
                        raw_previous_vec=[]
                    if user_choice == 'r':
                        print "restarting recording"
                        state=RecordingState.RECORD

            # else:
            # TODO display pressure

        # State Process recording
        if state==RecordingState.PROCESS:
            # check if data is valid, otherwise ask for recording again with some instruction how to improve
            ## basic data size check
            # TODO
            ## push/release check and plot the last recording
            [inc, dec] = get_push_release_from_msgs(msgs, CHANGE_DETECT_THRESH, args.plot)
            
            if inc is None:
                print " failed to extract push/release, verify the data visually, do you want to save anyway ?"
                if user_yesno(default=False):
                    state = RecordingState.SAVE
                else:
                    user_choice = user_menu({'d': "detect a new cell", 'r': "restart recording", 's': "save now"})
                    if user_choice == 's' or user_choice == 'c':
                        state=RecordingState.SAVE
                    if user_choice == 'd':
                        state=RecordingState.DETECT
                        raw_previous_vec=[]
                    if user_choice == 'r':
                        print "restarting recording"
                        state=RecordingState.RECORD

            else:
                if len(inc) < args.repetition:
                    print len(inc)," push/release actions found when", args.repetition, "were expected"
                    print "restart recording ?"
                    if user_yesno(default=False):
                        print "restarting recording, please press the channel with the calibration tool in a push/release motion during the next", str(DEFAULT_RECORDING_DURATION) , " sec, ", args.repetition, "times in a row"
                        state = RecordingState.RECORD
                    else:
                        state=RecordingState.SAVE
                else:
                    # data is valid
                    state=RecordingState.SAVE

        # State Save recording
        if state==RecordingState.SAVE:
            save_filename = None
            if len(msgs) and detected_channel is not None: 
                date_time_obj = datetime.now()
                date_time = date_time_obj.strftime("%Y-%m-%d-%H-%M-%S")
                save_filename = "calib_" + str(detected_channel) + "_" + date_time + ".bag"
                saved=save_data(save_filename)

                if saved:
                    if channel_list is not None:
                        # remove is was in list
                        if detected_channel in channel_list:
                            channel_list.remove(detected_channel)
                        
            else:
                print "No data to save or No channel selected"
            if detected_channel is not None:
                processed_channels[detected_channel]= save_filename
            # loop
            if not quit_request:
                saved = False
                reset_recording()
                state=RecordingState.NEXTCHANNEL
            else:
                saved = True # even if not saved, we need to quit now
                state=RecordingState.END
                

        # State End
        if state==RecordingState.END:
            if not saved:
                quit_request = True
                state=RecordingState.SAVE
            else:
                print "Channels recorded ", processed_channels.keys()
                if channel_list is not None and len(channel_list) > 0:
                    print "some channels were not recorded :", channel_list
                
                if args.plot:
                    print "close plot windows to quit"
                    plt.show(block = True)
                break;

        rate.sleep()




