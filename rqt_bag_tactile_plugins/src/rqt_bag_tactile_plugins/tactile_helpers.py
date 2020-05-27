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
