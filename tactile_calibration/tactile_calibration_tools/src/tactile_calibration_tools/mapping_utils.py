from __future__ import print_function
import os
import yaml
import csv
from collections import OrderedDict

DEFAULT_MAPPING_FILENAME = "mapping.yaml"
DEFAULT_LOOKUP_FILENAME = "lookup.csv"


def check_idx_range_duplicate(idx_ranges, data_channel):
    new_idx_ranges = []
    found_duplicate = False
    for idx_range in idx_ranges:
        # print "  found range", idx_range
        if not found_duplicate:
            if type(idx_range) == list:
                if data_channel > idx_range[0] and data_channel < idx_range[1]:  # stricly within the range
                    if data_channel-1 == idx_range[0]:  # touches one extremity, this extremity is alone
                        new_idx_ranges.append(idx_range[0])
                    if data_channel+1 == idx_range[1]:  # touches other extremity, this extremity is alone
                        # no gap, only 2 extremeties
                        new_idx_ranges.append(idx_range[1])
                    if data_channel-1 > idx_range[0]:
                        new_idx_ranges.append([idx_range[0], data_channel-1])
                    if data_channel+1 < idx_range[1]:
                        new_idx_ranges.append([data_channel+1, idx_range[1]])
                    found_duplicate = True
                else:  # within the range but on the extremities or out of the range
                    if data_channel == idx_range[0] or data_channel == idx_range[1]:  # is on one extremity
                        if data_channel == idx_range[0]:  # one extremity
                            if data_channel+1 == idx_range[1]:  # end is single
                                new_idx_ranges.append(idx_range[1])
                            else:  # end is not single
                                new_idx_ranges.append([data_channel+1, idx_range[1]])
                        if data_channel == idx_range[1]:  # other extremity
                            if data_channel-1 == idx_range[0]:  # end is single
                                new_idx_ranges.append(idx_range[0])
                            else:
                                new_idx_ranges.append([idx_range[0], data_channel-1])
                        found_duplicate = True
                    # else:  out of the range
                # continue
            else:  # int
                if idx_range == data_channel:
                    found_duplicate = True
                    # don't append, since this data_channel will have its own calib
                else:
                    new_idx_ranges.append(idx_range)
        else:  # concatenate the remaining ranges
            new_idx_ranges.append(idx_range)
    if found_duplicate:
        # print "found duplicate "
        return new_idx_ranges
    else:
        return None


# handle yaml export

class flowmap(list):
    pass


def flowmap_rep(dumper, data):
    return dumper.represent_sequence(u'tag:yaml.org,2002:seq', data, flow_style=True)


def represent_dict_order(self, data): return self.represent_mapping('tag:yaml.org,2002:map', data.items())


yaml.add_representer(flowmap, flowmap_rep)
yaml.add_representer(OrderedDict, represent_dict_order)


def save_mapping(mapping_dict, calib_channel, sensor_name, mapping_file=None, output_csv=False):
    if mapping_file is not None:
        mapping_filename = mapping_file
    else:
        mapping_filename = DEFAULT_MAPPING_FILENAME

    if output_csv:
        with open(DEFAULT_LOOKUP_FILENAME, 'w') as csvfile:
            csvwriter = csv.writer(csvfile, delimiter=',')
            for x in mapping_dict:
                csvwriter.writerow([x, mapping_dict[x]])

    rename_oldcalib = False
    calib = None
    newcalib = {"calib": []}
    # check if mapping file exists
    try:
        # read previous data if exists
        with open(mapping_filename, 'r') as f:
            # parse yaml
            previous_calib = yaml.load(f)
            if previous_calib is not None:
                # check if previous calib is single calib format or not
                if "calib" in previous_calib:
                    # multi calib file
                    calib = previous_calib
                else:
                    # old calib file
                    # double check there are only digits as keys
                    if previous_calib:
                        for k in previous_calib.keys():
                            if type(k) != int:
                                print("file is not a correct mapping, and cannot be merged")
                                rename_oldcalib = True
                        if not rename_oldcalib:
                            print("Old calib file detected, converting to new format and merging")
                            calib = {"calib": [{'sensor_name': sensor_name, 'type': 'PWL',
                                                'idx_range': flowmap([-1]), "values": previous_calib}]}
            # else: # file is empty

        if rename_oldcalib is True:
            # TODO rename
            os.rename(mapping_filename, mapping_filename + ".bak")
        # merge calib if needed
        if calib is None:  # no merge possible, or file empty then just write
            newcalib["calib"] = [{'sensor_name': sensor_name, 'type': 'PWL',
                                  'idx_range':  flowmap([calib_channel]), "values": mapping_dict}]
        else:  # merge needed
            # TODO insert the new channel at the correct place
            # TODO maybe allow to insert this calib for more than calib_channel
            # check if duplicate data_channel
            for cal in calib["calib"]:
                if 'values' not in cal:
                    cal['values'] = []
                new_idx_range = check_idx_range_duplicate(cal["idx_range"], calib_channel)
                if new_idx_range is not None:  # data must be split
                    print(" found a calibration for channel", calib_channel, " replacing with new calib")
                    if len(new_idx_range):
                        # keep the others
                        if type(new_idx_range) == list:
                            print(new_idx_range)
                            newcalib["calib"].append(
                                {'sensor_name': cal["sensor_name"], 'type': cal["type"], 'idx_range':  flowmap(new_idx_range), 'values': cal["values"]})
                        else:
                            newcalib["calib"].append({'sensor_name': cal["sensor_name"], 'type': cal["type"], 'idx_range': flowmap(
                                [new_idx_range]), 'values': cal["values"]})
                else:  # just append
                    newcalib["calib"].append({'sensor_name': cal["sensor_name"], 'type': cal["type"],
                                             'idx_range':  flowmap(cal["idx_range"]), 'values': cal["values"]})
            # append new calib
            newcalib["calib"].append({'sensor_name': sensor_name, 'type': 'PWL',
                                     'idx_range': flowmap([calib_channel]), 'values': mapping_dict})

    except IOError:  # file does not exist, create it
        newcalib["calib"] = [{'sensor_name': sensor_name, 'type': 'PWL',
                              'idx_range': flowmap([calib_channel]), 'values': mapping_dict}]
        pass
    print("Saving mapping to ", mapping_filename)
    with open(mapping_filename, 'w') as f:
        data = yaml.dump(newcalib, f, default_flow_style=False)  # sort_keys=False in python3 only
