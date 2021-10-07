#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import argparse

from tactile_calibration_tools.calibration_utils import *
from tactile_calibration_tools.mapping_utils import *

REF_CALIB_RATIO = -0.0154
REF_CALIB_OFFSET = 53.793
REF_FLAT_THRESHOLD = 0.3 # in newton (30 gram) accepatable variation to find the flat part of the ref
DEFAULT_INPUT_RESOL = 12 # in bits
CHANGE_DETECT_THRESH = 4 # 5 unit of velocity positive or negative
DEFAULT_SEGMENTS = 5


def process(bagfilename, topic, calib_channel, data_channel, ref_channel, ref_ratio, ref_offset, ref_tare_val, ref_is_raw, input_range_max, segments, no_extrapolation, mapping_file, output_csv, plot):

    # read the bag file
    [sensor_name, ref_raw_vec, raw_vec] = read_calib(bagfilename, topic, data_channel, ref_channel, input_range_max)
    if (len (raw_vec)==0 or len(ref_raw_vec) ==0):
        print("no data retrieved, check topic name ")
        return -1

    # process the data
    print("Processing data...")
    # 3. Offline Lookuptable generation
    ## convert the data
    raw = np.array(raw_vec)
    ## process ref if needed
    ref_newton_tare = calibrate_ref(ref_raw_vec, ref_ratio, ref_offset, ref_tare_val, REF_FLAT_THRESHOLD , ref_is_raw)

    # find the sections in which pressure increases/decreases
    print(" Finding push/release")
    [inc_idx, dec_idx] = get_push_release(raw, ref_newton_tare, CHANGE_DETECT_THRESH, args.plot)
    if inc_idx is not None:
        [inc, dec] = generate_lookup(raw, ref_newton_tare, inc_idx, dec_idx, input_range_max, plot)
        if len(inc)==0:
            print(" failed to generate lookup")
            return -1
    else:
        print(" failed to extract push/release")
        return -1
    print(" Fitting the data and extracting a", segments, " segment piece-wise-linear calib")
    # process only increasing
    mapping_dict = generate_mapping_pwl(inc[0], inc[1], input_range_max, calib_channel, segments, no_extrapolation, plot)

    # 6. Save
    # a    Save Lookuptable and-or Model in TaxelCalibrationMapping file.
    print("Preparing mapping for cell ", calib_channel)
    save_mapping(mapping_dict, calib_channel, sensor_name, mapping_file, output_csv)

    return 0

if __name__ == "__main__":
    # execute only if run as a script
    parser = argparse.ArgumentParser()

    # args :
    parser.add_argument("bagfilename", type=str,
                      help="bag filename to open, use calib_# as filename to process all calib of the folder")
    parser.add_argument("topic", type=str,
                      help="topic to process")
    parser.add_argument("--mapping_file", type=str,
                      help="output mapping filename, default is mapping.yaml")
    parser.add_argument("--no_extrapolation", action="store_true",
                      help="deactivate extrapolation of the mapping at both ends (O and input_range-1)")
    parser.add_argument("--data_channel", type=int,
                      help="index of the data channel to calibrate")
    parser.add_argument("--ref_channel", type=int,
                      help="index of the ref channel to calibrate against")
    parser.add_argument("--ref_is_raw",  action="store_true",
                      help="activate if the ref channel should be calibrated, and provide ref_ratio & ref_offset")
    parser.add_argument("--ref_tare_val", type=float,
                      help="use provided reference tare value (given in newton) instead of using rest pose values")
    parser.add_argument("--ref_ratio", type=float,  nargs='?', const=REF_CALIB_RATIO,
                      help="reference ratio (indicated on the tool)")
    parser.add_argument("--ref_offset", type=float, nargs='?', const=REF_CALIB_OFFSET,
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
        print("ref_is_raw was activate but ref_ratio and/or ref_offset are missing")
        exit(-1)

    # select mode of operation either calib_xx with xx the calibrated cell idx, or provided calib cell idx, ref cell indx
    [calib_channels, data_channel, ref_channel] = get_channels(args.data_channel, args.ref_channel, args.bagfilename)
    if calib_channels is None:
        print("could not find a channel number in the filename (expected 'calib_NN_*.bag') and no data_channel provided")
        exit(-1)
    if type(calib_channels) == dict:
        for calib_channel in calib_channels:
            print("### Processing file", calib_channels[calib_channel], " channel", calib_channel, "###")
            process(calib_channels[calib_channel], args.topic, calib_channel, data_channel,
            ref_channel, args.ref_ratio, args.ref_offset, args.ref_tare_val, args.ref_is_raw,
            input_range_max, args.segments, args.no_extrapolation, args.mapping_file, args.output_csv, args.plot)
    else:  # single number
        calib_channel = calib_channels
        process(args.bagfilename,  args.topic, calib_channel, data_channel,
            ref_channel, args.ref_ratio, args.ref_offset, args.ref_tare_val, args.ref_is_raw,
            input_range_max, args.segments, args.no_extrapolation, args.mapping_file, args.output_csv, args.plot)

    if args.plot:
            print("close plot windows to quit")
            plt.show(block = True)
