# Disclaimer

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.

# Overview

This toolbox contains 2 command line applications, a recorder and a generator

## Recorder 

The recorder is a helper to record a raw and a reference signal in a specific calib bagfile to be later calibrated through the generator.
It is recommended to provide reference calibration information (see options in usage) to directly save a calibrated reference (for instance in Newton)

It can record one or more channel in a row.
A tare is of the reference is done at start of the application unless disabled.
The same tare is re-used for all the acquired channels.
If a same channel is recorded twice during a recording session, the previous data is moved to an "old" folder.

## Generator

The gerenator basically extracts a Piece Wise Linear (PWL) calibration mapping from a dataset composed of raw values of a data channel 
and corresponding ref values (of a calibration tool) while pressing/releasing the data channel to be calibrated.
The PWL mapping is computed by the library pwlf  https://github.com/cjekel/piecewise_linear_fit_py that must be installed in the system (see Requirements)

The default data expected is in form of a rosbag file with a name "calib_##_<datetime>.bag" containing raw values at channel 0 of a tactile_msgs and ref values at channel 1. 
The number in the filename will be extracted to serve as calibrated channel index in the mapping unless a different number is provided at command line.
However, the application can handle any bagfile containing a tactile_msgs, at any topic name and at any raw/ref channel indices provided correct options are given.
A special input permit to process all calib_## files in a folder.

The output is a mapping.yaml file that will be merged with any existing/provided filename, and will replace calibration for duplicate channel index with existing entries.
single calib mapping.yaml (previous format) will be converted to the multi calib calibration format before merging

# Requirements

* ros environment providing rosbag

works only in python2.7 due to ros in melodic not supporting python3 (rospkg)

* pwlf library 
 
pip install pwlf

pwlf is compatible with Python2.7 but requires some more recent numpy than the system in Ubuntu xenial, ensure you have more recent one.

# Usage of the recorder 

rosrun tactile_calibration_tools record_calib.py <topic> <ref_channel>

usage: record_calib.py [-h] [--ref_topic REF_TOPIC]
                       [--data_channel DATA_CHANNEL [DATA_CHANNEL ...]]
                       [--detect_threshold DETECT_THRESHOLD]
                       [--num_channels NUM_CHANNELS] [--ref_tare REF_TARE]
                       [--ref_ratio [REF_RATIO]] [--ref_offset [REF_OFFSET]]
                       [--no_tare] [--plot]
                       [--input_resolution INPUT_RESOLUTION]
                       [--repetition REPETITION]
                       raw_topic ref_channel

## required arguments:
 
* raw_topic           topic where the data channel and optionally reference channel is
* ref_channel         index where the reference channel is in the raw_topic, or in the ref_topic if provided.

## optional arguments:

* --data_channel <DATA_CHANNEL> [<DATA_CHANNEL2> [...]] : index (or space-separated indices) of the data channel to calibrate 
    (if not provided in the bagfile name, or if one wants to change the default channel 0)
* --detect_threshold <DETECT_THRESHOLD> : change the default detection threshold for selecting a cell
* --ref_topic <REF_TOPIC> : topic where the reference channel is if different from raw_topic (data of raw and ref topic will be synchronized)
* --num_channels <NUM_CHANNELS> : number of channels to calibrate. Will prepare a list of 0 to num_channel-1 to be calibrated.
* --ref_tare <REF_TARE> : use provided reference tare value (given in newton) instead of running a tare procedure
* --no_tare : deactivates using tare and deactivates tare procedure
* --ref_ratio [<REF_RATIO>] (used in combination with --ref_offset) : reference ratio for ref_raw calibration (indicated on the tool), if no value given, default will be used
* --ref_offset [<REF_OFFSET>] (used in combination with --ref_ratio) : reference offset for ref_raw calibration (indicated on the tool), if no value given, default will be used
* --plot : show the plots
* --input_resolution <INPUT_RESOLUTION> : input resolution in bits (default is 12)
* --repetition <REPETITION> : number of repetition of push/release actions awaited (NOT USED YET)


# Usage of the generator

rosrun tactile_calibration_tools generate_calib.py <bagfilename> <topic> 

usage: generate_calib.py [-h] [--mapping_file MAPPING_FILE]
                         [--no_extrapolation] [--data_channel DATA_CHANNEL]
                         [--ref_channel REF_CHANNEL] [--ref_is_raw]
                         [--ref_tare_val REF_TARE_VAL]
                         [--ref_ratio [REF_RATIO]] [--ref_offset [REF_OFFSET]]
                         [--plot] [--output_csv]
                         [--input_resolution INPUT_RESOLUTION]
                         [--segments SEGMENTS]
                         bagfilename topic

## required arguments:
 
* bagfilename           bag filename to open. 
                        If filename is ofr the form "calib_##_*.bag", ## being a number, the number will be extracted to become the calibrated cell index in the output mapping
                        If filename ends with calib_# (with the sharp), all calib_##_*.bag files in the given folder will be processed
                           if several files are available for a same channel, the latest in recording date will be used.
* topic                 topic to process, matches previously used data topic in the recorder

## optional arguments:

* --mapping_file <MAPPING_FILE> : output mapping filename, default is mapping.yaml
* --no_extrapolation : deactivate extrapolation of the mapping at both ends (O and input_range-1)
* --data_channel <DATA_CHANNEL> : index of the data channel to calibrate (if not provided in bagfile name, or if want to change the default channel 0)
* --ref_channel <REF_CHANNEL> : index of the ref channel to calibrate against (default channel is 1)
* --ref_is_raw : activate if the ref channel should be calibrated, should be used in combination with ref_ratio & ref_offset
* --ref_tare_val <REF_TARE_VAL> : use provided reference tare value (given in newton) instead of using rest pose values at beginning of the ref data
* --ref_ratio [<REF_RATIO>] (used only if --ref_is_raw is active) : reference ratio for ref_raw calibration (indicated on the tool), if no value given, default will be used
* --ref_offset [<REF_OFFSET>] (used only if --ref_is_raw is active) : reference offset for ref_raw calibration (indicated on the tool), if no value given, default will be used
* --plot : show the plots
* --output_csv : enable output of a lookup.csv with the resulting mapping (for debugging)
* --input_resolution <INPUT_RESOLUTION> : input resolution in bits (default is 12)
* --segments <SEGMENTS> : number of segments for piece-wise-linear mapping (default is 2)

