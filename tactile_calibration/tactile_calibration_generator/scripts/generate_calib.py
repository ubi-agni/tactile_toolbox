#!/usr/bin/env python
import numpy as np
import rosbag
from tactile_msgs.msg import TactileState
import argparse
from collections import OrderedDict
import csv
import yaml
import time

#pip install pwlf
import pwlf

import matplotlib.pyplot as plt

REF_CALIB_RATIO = -0.0154
REF_CALIB_OFFSET = 53.793
DEFAULT_INPUT_RESOL = 12 # in bits
CHANGE_DETECT_THRESH = 4 # 5 unit of velocity positive or negative
DEFAULT_SEGMENTS = 5


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

def find_inflection_index(x):
    direction = 0 # check for any change
    increasing_idx = []
    decreasing_idx = []
    sign = 1
    init_val = None
    # TODO add counting of repetition of push/release
    for i, val in enumerate(vel):
        if direction == 0: # checking for changes at all
            if init_val is None:
                init_val = val
            if abs(val) > CHANGE_DETECT_THRESH:
                if val-init_val > 0: # first change is increasing (push = increasing)
                    sign = 1
                    direction = 2
                else:  # first change is decreasing (push = decreasing)
                    sign = -1
                    direction = -2
        if direction == 1: # checking for increasing
            if val > CHANGE_DETECT_THRESH: # if above threshold consider increasing
                direction = 2 # switch to increasing
                increasing_idx.append(i)
                continue
        if direction == 2: # increasing
            if val > 0:
                increasing_idx.append(i)
                continue
            else:
                direction = -1 # switch to check for decreasing
                continue
        if direction == -1:
            if val < -CHANGE_DETECT_THRESH: # if below threshold consider decreasing
                direction = -2 # switch to decreasing
                decreasing_idx.append(i)
                continue
        if direction == -2: # decreasing
            if val < 0:
                decreasing_idx.append(i)
                continue
            else:
                direction = 1 # switch to check for increasing
                continue
    if sign > 0:
        return [np.array(increasing_idx), np.array(decreasing_idx)]
    else:  # inverse decreasing and increasing to have always "pushing actions" first
        return [np.array(decreasing_idx), np.array(increasing_idx)]


if __name__ == "__main__":
    # execute only if run as a script
    parser = argparse.ArgumentParser()

    # args : 1 bagfilename, 2 topic, sensor number, calib number
    parser.add_argument("bagfilename", type=str,
                      help="bag filename to open")
    parser.add_argument("topic", type=str,
                      help="topic to process")
    parser.add_argument("--no_extrapolation", action="store_true",
                      help="deactivate extrapolation of the mapping at both ends (O and input_range-1)")
    parser.add_argument("--data_channel", type=int,
                      help="index of the data channel to calibrate")
    parser.add_argument("--ref_channel", type=int,
                      help="index of the ref channel to calibrate against")
    parser.add_argument("--ref_tare_val", type=float,
                      help="use provided reference tare value (given in newton) instead of using rest pose values")
    parser.add_argument("--ref_ratio", type=float, default=REF_CALIB_RATIO,
                      help="reference ratio (indicated on the tool)")
    parser.add_argument("--ref_offset", type=float, default=REF_CALIB_OFFSET,
                      help="reference offset (indicated on the tool)")
    parser.add_argument("--plot",  action="store_true", help="show the plots")
    parser.add_argument("--output_csv",  action="store_true", help="enable output of a lookup.csv with the resulting mapping")
    parser.add_argument("--input_resolution", type=int, default=DEFAULT_INPUT_RESOL,
                      help="input resolution in bits")
    parser.add_argument("--segments", type=int, default=DEFAULT_SEGMENTS,
                      help="number of segments for piece-wise-linear")
    args = parser.parse_args()


    input_range_max = 2**args.input_resolution
    data_channel = None
    calib_channel = None
    ref_channel = None
    
    # validate options
    if args.ref_is_raw and not (args.ref_ratio and args.ref_offset):
        print "ref_is_raw was activate but ref_ratio and/or ref_offset are missing"
        exit(0)
    
    
    # select mode of operation either calib_xx with xx the calibrated cell idx, or provided calib cell idx, ref cell indx
    if args.data_channel and args.ref_channel:
        calib_channel = args.data_channel
        data_channel = calib_channel
        ref_channel = args.ref_channel
    else:
        # try extract data_channel from filename
        if "calib_" in args.bagfilename:
            split_filename = args.filename.split('_')
            if split_filename[1].is_digit():
                calib_channel = int(split_filename[1])
                data_channel = 0
                ref_channel = 1
        else:
            if args.data_channel:
                calib_channel = args.data_channel
                data_channel = 0
                ref_channel = 1
            else:
                print "could not find a channel number in the filename (expected 'calib_##_*.bag') and no data_channel provided"
                exit(0)
 

    # read the bag file
    bag = rosbag.Bag(args.bagfilename)
    ref_raw_vec = []
    raw_vec = []
    # TODO Guillaume : also handle the sensor name in case there are more than one sensor (like on iObject+)
    sensor_name = None
    print "Reading", args.bagfilename," looking for ", args.topic, "channel", data_channel, " and ref ", ref_channel 
    warned_size_tactile_vec = False
    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        # if there is data
        if len(msg.sensors):
            if sensor_name is None:
                sensor_name = msg.sensors[0].name
            # warn if no channel provided but data is larger than 2 values
            if not warned_size_tactile_vec:
                if len(msg.sensors[0].values) > 2 and not (args.data_channel and args.ref_channel):
                    print "# Warning #, data size larger than 2 elements but not both data_channel and ref_channel were given"
                    warned_size_tactile_vec = True
                
            # validate index are in the range
            if (ref_channel < len(msg.sensors[0].values) and data_channel < len(msg.sensors[0].values)):
                if msg.sensors[0].values[data_channel] < input_range_max:
                    # extract the dedicated channels for raw and reference_raw
                    # 2.d    crop saved values to pair of channels [CalibTool;determined taxe]: /TactileGlove/sensors[0]/values[17] and /TactileGlove/sensors[0]/values[determined taxel]
                    ref_raw_vec.append(msg.sensors[0].values[ref_channel])
                    raw_vec.append(msg.sensors[0].values[data_channel])
                else: # input range badly chosen
                    print "Data ", msg.sensors[0].values[data_channel], " is higher than input resolution ", input_range_max
                    bag.close()
                    exit(-1)
            # else drop the data of this message
    bag.close()
    if (len (raw_vec)==0 or len(ref_raw_vec) ==0):
        print "no data retrieved, check topic name "
        exit(-1)

    # process the data
    # 3. Offline Lookuptable generation
    # convert the data
    ref_raw = np.array(ref_raw_vec)
    raw = np.array(raw_vec)

    # compute ref in newton 3.b    Calculate Ground Truth in Newton GTN: with Model GTN = -0,0154 * CtRAW+ 53,793    where   CrRAW = Calib-tool-RAW [12Bit] = "field.sensors0.values17" out of recorded rosbag file.
    ref_newton = args.ref_ratio * ref_raw + args.ref_offset
     # tare 3.c    Tare with each: GTtN = GTN - Mean(TareValues)
    ref_newton_tare = ref_newton - args.ref_tare_val

    # find the sections in which pressure increases/decreases
    # TODO Guillaume : also look at the range of ref data to not get a decreasing force at the end of the increasing raw data
    ## smooth the data to avoid derivate sign change on noise
    smooth_raw = smooth(raw,19)
    ## compute derivative of the input data
    vel = np.gradient(smooth_raw)
    ## find inflections, inc_idx are increasing "pressure" index (which might be raw value decreasing or increasing)
    [inc_idx, dec_idx]=find_inflection_index(vel);

    # check there were some push/release
    if len(inc_idx)==0 or len(dec_idx)==0:
        print "could not find push/release action in the data, verify the file or the channels"
        exit(0)
    # 3.a View recorded data in graphplot in order to validate correctness (no spurious wrong data or high noise)
    if args.plot:
        plt.plot(range(len(smooth_raw)),smooth_raw, 'g-', lw=1) # smooth raw values
        plt.plot(range(len(ref_newton_tare)),ref_newton_tare*-100.0, 'm-', lw=1) # smooth ref values, on negative side to avoid cluttering
        plt.plot(inc_idx, smooth_raw[inc_idx], 'rx', lw=2) # increasing pressure values
        plt.plot(dec_idx, smooth_raw[dec_idx], 'bx', lw=2) # decreasing pressure values
        plt.plot(range(len(vel)), vel*50, 'k-', lw=1) # velocity scaled up
        plt.show()

    # 3.d0    Database funktion: With each lookuptable entrance "Lookup[0-4095]"  (in excel it is  AVERAGEIF function)
    missing_inc_input_values = [] # list of increasing input pressure values that do not appear in the recorded data.
    missing_dec_input_values = [] # list of decreasing input pressure values that do not appear in the recorded data.
    lookup_inc = OrderedDict()
    lookup_dec = OrderedDict()
    for val in range(input_range_max):

        # Get the index of increasing elements with value val   3.d1        Seek Lookup(n=0) within RAW-Taxel-values
        #print "inc_idx ", inc_idx
        idx_of_val_inc = np.where(raw[inc_idx] == val)
        #print "idx_of_val_inc ", idx_of_val_inc
        #print "inc_idx[idx_of_val_inc] ",inc_idx[idx_of_val_inc]
        #print "ref_newton_tare ", ref_newton_tare[inc_idx[idx_of_val_inc]]

        if len(idx_of_val_inc[0]) > 0:
            # TODO: depending how many index were found, process or not
            # store mean val 3.d2-d5
            lookup_inc[val] = round(np.mean(ref_newton_tare[inc_idx[idx_of_val_inc]]),3);
        else: # no element for that value, store it in an index
            missing_inc_input_values.append(val)

        # Get the index of decreasing elements with value val   3.d1        Seek Lookup(n=0) within RAW-Taxel-values
        idx_of_val_dec = np.where(raw[dec_idx] == val)
        if len(idx_of_val_dec[0]) > 0:
            # TODO: depending how many index were found, process or not
            # store mean val 3.d2-d5
            lookup_dec[val] = round(np.mean(ref_newton_tare[dec_idx[idx_of_val_dec]]),3);
        else: # no element for that value, store it in an index
            missing_dec_input_values.append(val)

    lists_lookup_inc = lookup_inc.items() # sorted by key, return a list of tuples
    x_inc, y_inc = zip(*lists_lookup_inc) # unpack a list of pairs into two tuples
    
    lists_lookup_dec = lookup_dec.items() # sorted by key, return a list of tuples
    x_dec, y_dec = zip(*lists_lookup_dec) # unpack a list of pairs into two tuples
    
    # e    Visualize point diagram with [0-4095;LGTtN] including all taxel data points in X axis =[0-4095] over Y-axis =[fmin-fmax Newton]
    if args.plot:
        plt.plot(x_inc, y_inc, 'mx')
        plt.plot(x_inc, smooth(y_inc,100), 'r')
        plt.plot(x_dec, y_dec, 'c+')
        plt.plot(x_dec, smooth(y_dec,100), 'b')
        plt.show()

    # process only increasing
    xs = np.array(x_inc)
    ys = np.array(y_inc)
    #ys = smooth(y_inc,100)

    # solution with pwlf 1  https://pypi.org/project/pwlf/   # not available in package manager
    # Installing collected packages: numpy, scipy, pyDOE, pwlf
    # Successfully installed numpy-1.16.6 pwlf-2.0.3 pyDOE-0.3.8 scipy-1.2.3

    pwlf_result = pwlf.PiecewiseLinFit(xs, ys)
    # request a fit with n points
    pwlf_breaks = pwlf_result.fit(args.segments)
    
    #print(breaks)
    if args.plot:
        x_hat = np.linspace(xs.min(), xs.max(), 100)
        y_hat = pwlf_result.predict(x_hat)

        plt.figure()
        plt.plot(xs, ys, 'o')
        plt.plot(x_hat, y_hat, '-')
        plt.show()

    # with open('lookup_inc.csv', 'w') as csvfile:
        # csvwriter = csv.writer(csvfile, delimiter=',')
        # for key in lookup_inc:
            # csvwriter.writerow([key, lookup_inc[key]])
    # with open('lookup_dec.csv', 'w') as csvfile:
        # csvwriter = csv.writer(csvfile, delimiter=',')
        # for key in lookup_dec:
            # csvwriter.writerow([key, lookup_dec[key]])
    # extract the lookup_values
   
    mapping_dict_tmp = OrderedDict()
    mapping_dict = OrderedDict()
    for b in pwlf_breaks:
        mapping_dict_tmp[int(b)] =  round(float(pwlf_result.predict(round(float(b),2))[0]),3)

    if not args.no_extrapolation:
        # extract extremety point 0
        mapping_dict[0] = pwlf_result.beta[0] + (pwlf_result.beta[1])*(0.0-pwlf_breaks[0])
        # add other points
        for k, i in mapping_dict_tmp.items():
            mapping_dict[k] = i
        # extract extremety point input_range_max-1
        y0 = pwlf_result.predict(pwlf_breaks[args.segments-1])[0] # y value at last breakpoint
        mapping_dict[input_range_max-1] = round(float(y0 + (pwlf_result.beta[args.segments])*((input_range_max-1)-pwlf_breaks[args.segments-1])),3)
    else:
        mapping_dict = mapping_dict_tmp

    #print mapping_list
    if args.output_csv:
        with open('lookup.csv', 'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            for x in mapping_dict:
                csvwriter.writerow([x, mapping_dict[x]])
            
    # 6. Save
    # a    Save Lookuptable and-or Model in TaxelCalibrationMapping file.
    #yaml.add_representer(OrderedDict, lambda dumper, data: dumper.represent_mapping('tag:yaml.org,2002:map', data.items()))
    #yaml.add_representer(tuple, lambda dumper, data: dumper.represent_sequence('tag:yaml.org,2002:seq', data))
    represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
    yaml.add_representer(OrderedDict, represent_dict_order)  

    calib = {"calib": [{'sensor_name': sensor_name, 'type': 'PWL', 'idx_range': [args.data_channel], "values": mapping_dict}]}
    with open('mapping.yaml', 'w') as f:
        data = yaml.dump(calib, f)  #sort_keys=False in python3 only     

    


