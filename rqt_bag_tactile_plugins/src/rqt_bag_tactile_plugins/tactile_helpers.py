from xml.dom.minidom import parse, parseString
from geometry_msgs.msg import Pose, Point

from python_qt_binding.QtGui import QColor

# ============================================================
# ColorMap is based on color_map.cpp of tactile_filters
# Copyright (C) 2015 by Robert Haschke <rhaschke at techfak dot uni-bielefeld dot de>
# LGPLv3
# ============================================================

class ColorMap(object):
    def __init__(self, fMin, fMax):
        self.colors = []
        self.fMin=fMin
        self.fMax=fMax

    def append(self, c):
        if type(c)==QColor:
            self.colors.append(c)
        if type(c)==list:
            for name in c:
                self.colors.append(QColor(name))

    def map(self, value):

        errColor = QColor("cyan")
        if len(self.colors) > 1 : #and not np.isnan(value):
            ratio = float(value-self.fMin) / float(self.fMax-self.fMin) * (len(self.colors)-1)
            if (ratio < 0):
                return self.colors[0]
            idx = int(ratio)
            if (idx >= len(self.colors)-1):
                return self.colors[-1]
            b = ratio - idx  # in [0..1)
            a = 1.0 - b;
            lo = self.colors[idx]
            hi = self.colors[idx+1]
            return QColor(a*lo.red()+b*hi.red(), a*lo.green()+b*hi.green(), a*lo.blue()+b*hi.blue(), a*lo.alpha()+b*hi.alpha());
        return errColor


class TactileTaxel(object):
    def __init__(self, idx=0, origin=Point()):
        self.idx = idx
        self.origin = origin


class TactileArray(object):
    ROWMAJOR=0
    COLUMNMAJOR=1
    def __init__(self, rows=1, cols=0, order=None, size=[0, 0], spacing=[0, 0], offset=[0, 0]):
        self.rows = rows
        self.cols = cols
        self.order = order
        self.size = size
        self.spacing = spacing
        self.offset = offset


class TactileSensor(object):
    TAXEL_LIST = 1
    TAXEL_ARRAY = 2
    def __init__(self, name="", channel=""):
        self.name = name
        self.channel = channel
        self.taxels = {}
        self.array = None
        self.type = None


def parse_sensors(robot_description_xml):
    robot = parseString(robot_description_xml)
    sensors = {}
    for sensor_element in robot.getElementsByTagName("sensor"):
        if sensor_element.firstChild:
            sensor_name = sensor_element.getAttribute("name")
            for tactile_element in sensor_element.getElementsByTagName("tactile"):
                tactile_channel = tactile_element.getAttribute("channel")
                sensor = TactileSensor(sensor_name, tactile_channel)
                count_taxels = 0
                count_arrays = 0
                # find taxels
                for taxel_element in tactile_element.getElementsByTagName("taxel"):
                    count_taxels += 1
                    taxel = TactileTaxel()
                    sensor.taxels.append(taxel)
                # find arrays
                if count_taxels == 0:
                    
                    for array_element in tactile_element.getElementsByTagName("array"):
                        if count_arrays > 0:
                            print "more than one array is not supported"
                            break
                        rows = array_element.getAttribute("rows")
                        cols = array_element.getAttribute("cols")
                        order_str = array_element.getAttribute("order")
                        size = array_element.getAttribute("size").split(' ')
                        offset = array_element.getAttribute("size").split(' ')
                        spacing = array_element.getAttribute("size").split(' ')
                        sensor.array = TactileArray(int(rows), int(cols), TactileArray.ROWMAJOR if order_str == "row-major" else TactileArray.COLUMNMAJOR, size, spacing, offset)
                        count_arrays += 1
                    sensor.type = TactileSensor.TAXEL_ARRAY
                else:
                    sensor.type = TactileSensor.TAXEL_LIST
                if (count_taxels > 0 or count_arrays > 0):
                    sensors[tactile_channel] = sensor
    return sensors
