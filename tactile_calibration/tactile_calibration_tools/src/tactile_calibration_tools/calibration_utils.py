import rosbag
from tactile_msgs.msg import TactileState
import numpy as np
from collections import OrderedDict
import matplotlib.pyplot as plt
import os
#pip install pwlf
import pwlf

def find_inflection_index(x, change_detection_threshold=4):
    direction = 0 # check for any change
    increasing_idx = []
    decreasing_idx = []
    sign = 1
    init_val = None
    # TODO add counting of repetition of push/release
    for i, val in enumerate(x):
        if direction == 0: # checking for changes at all
            if init_val is None:
                init_val = val
            if abs(val) > change_detection_threshold:
                if val-init_val > 0: # first change is increasing (push = increasing)
                    sign = 1
                    direction = 2
                else:  # first change is decreasing (push = decreasing)
                    sign = -1
                    direction = -2
        if direction == 1: # checking for increasing
            if val > change_detection_threshold: # if above threshold consider increasing
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
            if val < -change_detection_threshold: # if below threshold consider decreasing
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

def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='valid')
    return y_smooth

def is_raw(data):
    # check if there data are pure integers
    if np.sum(data.astype(int) - data) ==0 :
        return True
    return False

def detect_channel_press(raw_previous_vec, raw_vec, ref_channel, detect_threshold):
    detected_channel = None
    if len(raw_previous_vec) == len (raw_vec):
        absdiff = np.fabs(np.array(raw_vec)-np.array(raw_previous_vec))
        idx_above_thresh = np.where(absdiff > detect_threshold)
        if len(idx_above_thresh[0]):  # anything above threshold  
            if len(idx_above_thresh[0])==1 and not (ref_channel in idx_above_thresh[0]):  # one channel pressed that is not the ref
                detected_channel =  idx_above_thresh[0][0]  
            if len(idx_above_thresh[0])==2:  # 2 channels pressed
                if ref_channel in idx_above_thresh[0]:  # the reference was pressed too
                    for channel in idx_above_thresh[0]:  # find the one that is not the ref
                        if channel != ref_channel:
                            detected_channel = channel
                            break
                else:  # 2 pressed and without ref
                    print "More than one channel was pressed, retry"
            if len(idx_above_thresh[0])>2:
                print "More than one channel was pressed, retry"
        #else none pressed
    return detected_channel


def compute_tare(data, flatness_threshold=None):
    if flatness_threshold is not None: # search for the flat part in the vec
        cur_val = None
        end_flat_range = None
        for i, val in enumerate(data):
            if cur_val is None:
                cur_val = val
            if abs(cur_val -  val) > flatness_threshold:
                end_flat_range = i
                break
        if end_flat_range is not None:
            # we found a certain zone of flatness at the beginning of the data
            tare = np.mean(ref_cal[0:end_flat_range])
        else: 
            print "The ref is too flat, are you sure the correct channel was selected ?"
            tare = None
    else: # not extracting a flat, part, just using the whole vector
        tare = np.mean(data)
    return tare


def calibrate_affine(raw, a, b):
    # affine calibration 
    return a * raw + b

def calibrate_ref(ref_vec, ref_ratio, ref_offset, user_tare=None, flatness_threshold=0.3, user_is_raw=False):
    ref_raw = np.array(ref_vec)
    if user_is_raw:
        print "calibrating ref with ratio=", ref_ratio, " offset=", ref_offset
        # compute ref in newton 3.b    Calculate Ground Truth in Newton GTN: with Model GTN = -0,0154 * CtRAW+ 53,793    where   CrRAW = Calib-tool-RAW [12Bit] = "field.sensors0.values17" out of recorded rosbag file.
        # tare 3.c    Tare with each: GTtN = GTN - Mean(TareValues)
        ref_cal = calibrate_affine(ref_raw, ref_ratio, ref_offset)

        # extract tare
        if user_tare is not None:  # tare is forced by the user, use it.
            tare = user_tare
        else:
            # need to compute it
            ## try find a "flat" zone in the beginning of the data
            tare = compute_tare(ref_cal, flatness_threshold)
            if tare is None:
                tare = 0
            else:
                print "extracted tare =", round(tare,3)
        # tare
        ref_cal_tare = ref_cal - tare

    else:  # value is already calibrated to newton and tared
        ref_cal_tare = np.array(ref_vec)
        # warn if data is strangly integers everywhere (which means is raw)
        if is_raw(ref_cal_tare):
            print " # warning # ref values seem raw but option --ref_is_raw was not set to true, proceeding without calibration of ref"

    return ref_cal_tare

def get_push_release(raw, ref, change_detect_threshold, doplot=False):
    # TODO Guillaume : also look at the range of ref data to not get a decreasing force at the end of the increasing raw data
    ## smooth the data to avoid derivate sign change on noise
    smooth_raw = smooth(raw,19)
    ## compute derivative of the input data
    vel = np.gradient(smooth_raw)
    ## find inflections, inc_idx are increasing "pressure" index (which might be raw value decreasing or increasing)
    [inc_idx, dec_idx]=find_inflection_index(vel, change_detect_threshold);

    # check there were some push/release
    if len(inc_idx)==0 or len(dec_idx)==0:
        print "could not find push/release action in the data, verify the file or the channels"
        return [None,None]

    # 3.a View recorded data in graphplot in order to validate correctness (no spurious wrong data or high noise)
    if doplot:
        plt.figure(1)
        plt.clf()
       
        plt.subplot(211)             # the first subplot in the first figure
        plt.title('Detection of Push / Release')
        plt.plot(range(len(smooth_raw)),smooth_raw, 'g-', lw=1) # smooth raw values
        plt.plot(inc_idx, smooth_raw[inc_idx], 'rx', lw=2) # increasing pressure values
        plt.plot(dec_idx, smooth_raw[dec_idx], 'bx', lw=2) # decreasing pressure values
        plt.plot(range(len(vel)), vel*50, 'k-', lw=1) # velocity scaled up
        plt.ylabel('Raw / derivative of Raw (x50)')
        plt.legend(["Channel","Pushing", "Releasing", "Vel"])
        
        plt.subplot(212)             # the second subplot in the first figure
        plt.plot(range(len(ref)), ref, 'm-', lw=1)
        plt.xlabel('Samples')
        plt.ylabel('Ref')
        plt.legend(["Reference"])
        plt.show(block = False)
    
    return [inc_idx, dec_idx]

def get_push_release_from_msgs(msgs, change_detect_threshold, doplot=False):
    raw_vec = []
    ref_vec = []
    for msg in msgs:
        raw_vec.append(msg.sensors[0].values[0])
        ref_vec.append(msg.sensors[0].values[1])
    return get_push_release(np.array(raw_vec), np.array(ref_vec), change_detect_threshold, doplot)

def generate_lookup(raw, ref, inc_idx, dec_idx, input_range_max, doplot=False):

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
            lookup_inc[val] = round(np.mean(ref[inc_idx[idx_of_val_inc]]),3);
        else: # no element for that value, store it in an index
            missing_inc_input_values.append(val)

        # Get the index of decreasing elements with value val   3.d1        Seek Lookup(n=0) within RAW-Taxel-values
        idx_of_val_dec = np.where(raw[dec_idx] == val)
        if len(idx_of_val_dec[0]) > 0:
            # TODO: depending how many index were found, process or not
            # store mean val 3.d2-d5
            lookup_dec[val] = round(np.mean(ref[dec_idx[idx_of_val_dec]]),3);
        else: # no element for that value, store it in an index
            missing_dec_input_values.append(val)

    lists_lookup_inc = lookup_inc.items() # sorted by key, return a list of tuples
    x_inc, y_inc = zip(*lists_lookup_inc) # unpack a list of pairs into two tuples


    lists_lookup_dec = lookup_dec.items() # sorted by key, return a list of tuples
    x_dec, y_dec = zip(*lists_lookup_dec) # unpack a list of pairs into two tuples

    # e    Visualize point diagram with [0-4095;LGTtN] including all taxel data points in X axis =[0-4095] over Y-axis =[fmin-fmax Newton]
    if doplot:
        plt.figure(2)
        plt.clf()
        plt.plot(x_inc, y_inc, 'mx')
        plt.plot(x_dec, y_dec, 'c+')
        
        plt.title('Lookup data for Push and Release') 
        plt.ylabel('Ref')
        plt.xlabel('Raw')
        plt.legend(["Pushing","Releasing"])
        plt.show(block = False)

    return [[x_inc, y_inc], [x_dec, y_dec]]

def get_channels(user_data_channel, user_ref_channel, bagfilename):
    calib_channel = data_channel = ref_channel = None
    if user_data_channel and user_ref_channel:
        calib_channel = user_data_channel
        data_channel = calib_channel
        ref_channel = user_ref_channel
    else:
        if user_data_channel: # user has priority
            calib_channel = user_data_channel
            data_channel = 0
            ref_channel = 1
        else:
            # try extract data_channel from filename
            filename = os.path.basename(bagfilename)
            if "calib_" in filename:
                split_filename = filename.split('_')
                if split_filename[1].isdigit():
                    calib_channel = int(split_filename[1])
                    data_channel = 0
                    ref_channel = 1
    return [calib_channel, data_channel, ref_channel]

def read_calib(bagfilename, user_topic, data_channel, ref_channel, input_range_max=1024):

    bag = rosbag.Bag(bagfilename)
    ref_raw_vec = []
    raw_vec = []
    # TODO Guillaume : also handle the sensor name in case there are more than one sensor (like on iObject+)
    sensor_name = None
    print "Reading", bagfilename," looking for ", user_topic, "channel", data_channel, " and ref ", ref_channel
    warned_size_tactile_vec = False
    for topic, msg, t in bag.read_messages(topics=[user_topic]):
        # if there is data
        if len(msg.sensors):
            if sensor_name is None:
                sensor_name = msg.sensors[0].name
            # warn if no channel provided but data is larger than 2 values
            if not warned_size_tactile_vec:
                if len(msg.sensors[0].values) > 2 and not (data_channel and ref_channel):
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

    return [sensor_name, ref_raw_vec, raw_vec]

def generate_mapping_pwl(x, y, input_range_max, seg=4, no_extrapolation=False, doplot=False):

    xs = np.array(x)
    ys = np.array(y)
    #ys = smooth(y_inc,100)

    # solution with pwlf 1  https://pypi.org/project/pwlf/   # not available in package manager
    # Installing collected packages: numpy, scipy, pyDOE, pwlf
    # Successfully installed numpy-1.16.6 pwlf-2.0.3 pyDOE-0.3.8 scipy-1.2.3

    pwlf_result = pwlf.PiecewiseLinFit(xs, ys)
    # request a fit with n points
    pwlf_breaks = pwlf_result.fit(seg)

    #print(breaks)
    if doplot:
        x_hat = np.linspace(xs.min(), xs.max(), 100)
        y_hat = pwlf_result.predict(x_hat)

        plt.figure()
        plt.clf() 
        plt.plot(xs, ys, 'o')
        plt.plot(x_hat, y_hat, '-')
        plt.title('Piece-wise linear mapping') 
        plt.ylabel('Ref')
        plt.xlabel('Raw')
        plt.legend(["Pushing data", "PWL Mapping"])
        plt.show(block = False)

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

    if not no_extrapolation:
        # extract extremety point 0
        mapping_dict[0] = round(float(pwlf_result.beta[0] + (pwlf_result.beta[1])*(0.0-pwlf_breaks[0])),3)
        # add other points
        for k, i in mapping_dict_tmp.items():
            mapping_dict[k] = i
        # extract extremety point input_range_max-1
        y0 = pwlf_result.predict(pwlf_breaks[seg])[0] # y value at last breakpoint
        mapping_dict[input_range_max-1] = round(float(y0 + (pwlf_result.beta[seg])*((input_range_max-1)-pwlf_breaks[seg-1])),3)
    else:
        mapping_dict = mapping_dict_tmp
    return mapping_dict

