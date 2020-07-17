# Overview

This application extract a Piece Wise Linear (PWL) calibration mapping from a dataset composed of raw values of a cell and corresponding ref values of a calibration tool while pressing/releasing the calibrated cell.
The PWL mapping is computed with a library pwlf  https://github.com/cjekel/piecewise_linear_fit_py that must be installed in the system (see Requirements)

The default data expected is in form of a rosbag file with a name "calib_##_*.bag" containing raw values at channel 0 of a tactile_msgs and ref values at a channel 1. 
the number in the filename will be extracted to serve as calibrated cell index in the mapping unless a different number is provided at command line.
However, the application can handle any bagfile containing a tactile_msgs, at any topic name and at any raw/ref channel indices provided correct options are given.

The output is a mapping.yaml file that will be merged with any existing/provided filename, and will replace calibration for duplicate cell index with existing entries
single cell mapping.yaml will be converted to the multicell calibration format before merging

# Requirements

* ros environment providing rosbag

works only in python2.7 due to ros in melodic not supporting python3 (rospkg)

* pwlf library 
 
pip install pwlf

pwlf is compatible with Python2.7 but requires some more recent numpy than the system in Ubuntu xenial, ensure you have more recent one.

# Usage

rosrun tactile_calibration_generator generate_calib.py <bagfilename> <topic> 

usage: generate_calib.py [-h] [--mapping_file MAPPING_FILE]
                         [--no_extrapolation] [--data_channel DATA_CHANNEL]
                         [--ref_channel REF_CHANNEL] [--ref_is_raw]
                         [--ref_tare_val REF_TARE_VAL] [--ref_ratio REF_RATIO]
                         [--ref_offset REF_OFFSET] [--plot] [--output_csv]
                         [--input_resolution INPUT_RESOLUTION]
                         [--segments SEGMENTS]
                         bagfilename topic

## required arguments:
 
* bagfilename           bag filename to open. If filename is "calib_##_*.bag" the number will be extracted to become the calibrated cell index in the output mapping
* topic                 topic to process

## optional arguments:

* --mapping_file <MAPPING_FILE> : output mapping filename, default is mapping.yaml
* --no_extrapolation : deactivate extrapolation of the mapping at both ends (O and input_range-1)
* --data_channel <DATA_CHANNEL> : index of the data channel to calibrate (if not provided in bagfile name, or if want to change the default channel 0)
* --ref_channel <REF_CHANNEL> : index of the ref channel to calibrate against (default channel is 1)
* --ref_is_raw : activate if the ref channel should be calibrated, should be used in combination with ref_ratio & ref_offset
* --ref_tare_val <REF_TARE_VAL> : use provided reference tare value (given in newton) instead of using rest pose values at beginning of the ref data
* --ref_ratio <REF_RATIO> (used only if --ref_is_raw is active) : reference ratio for ref_raw calibration (indicated on the tool)
* --ref_offset <REF_OFFSET> (used only if --ref_is_raw is active) : reference offset for ref_raw calibration (indicated on the tool)
* --plot : show the plots (requires to close plot windows for processing to continue)
* --output_csv : enable output of a lookup.csv with the resulting mapping (for debugging)
* --input_resolution <INPUT_RESOLUTION> : input resolution in bits (default is 12)
* --segments <SEGMENTS> : number of segments for piece-wise-linear mapping (default is 5)

