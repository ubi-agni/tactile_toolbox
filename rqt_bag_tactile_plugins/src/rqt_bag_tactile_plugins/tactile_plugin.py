from rqt_bag.plugins.plugin import Plugin
from rqt_bag import TopicMessageView, TimelineRenderer
from python_qt_binding.QtCore import Qt, QPoint, QRectF
from python_qt_binding.QtWidgets import QWidget, QPushButton
from python_qt_binding.QtGui import QPainter, QBrush, QTextOption

from tactile_msgs.msg import TactileState
import numpy as np


def get_color(state):
    if len(state.sensors) == 1:
        return Qt.green
    elif len(state.sensors) > 1 :
        return Qt.yellow
    else:  # 0
        return Qt.red
        
def get_average_taxel_color(sensor):
    if len(sensor.values) > 0:
        mean = np.array(sensor.values).mean()
        if mean >= 4095:
            return Qt.black
        elif mean >= 4000 and mean < 4095:
            return Qt.green
        elif mean >= 3000 and mean < 4000:
            return Qt.yellow
        elif mean >= 1000 and mean < 3000:
            return Qt.orange
        else:
            return Qt.red
    else:  # 0
        return Qt.black

class TactileArrayPanel(TopicMessageView):
    name = 'Tactile Array'
    MAX_COLUMNS = 4
    DEFAULT_HEIGHT =  20
    def __init__(self, timeline, parent, topic):
        super(TactileArrayPanel, self).__init__(timeline, parent, topic)
        self.widget = QWidget()
        parent.layout().addWidget(self.widget)
        self.msg = None
        self.widget.paintEvent = self.paintEvent
        self.text_option = QTextOption()
        self.text_option.setAlignment(Qt.AlignCenter)

    def message_viewed(self, bag, msg_details):
        super(TactileArrayPanel, self).message_viewed(bag, msg_details)
        _, self.msg, _ = msg_details
        self.widget.update()


    def paintEvent(self, event):
        self.qp = QPainter()
        self.qp.begin(self.widget)

        rect = event.rect()

        if self.msg is None:
            self.qp.fillRect(0, 0, rect.width(), rect.height(), Qt.white)
        else:
            num_sensors = len(self.msg.sensors)
            if num_sensors:
                if num_sensors > self.MAX_COLUMNS:
                    sensor_width = rect.width() / self.MAX_COLUMNS;
                else:
                    sensor_width = rect.width() / num_sensors
                i = 0
                j = 0
                for idx, sensor in enumerate(self.msg.sensors):
                    
                    if (idx % self.MAX_COLUMNS == 0):
                        i = 0
                        j +=1
                    
                    color = get_average_taxel_color(sensor)
                    self.qp.setBrush(QBrush(color))
                    self.qp.drawEllipse(i * sensor_width, j * self.DEFAULT_HEIGHT, sensor_width, self.DEFAULT_HEIGHT)
                    self.qp.drawText(QRectF(i * sensor_width, j * self.DEFAULT_HEIGHT,sensor_width,self.DEFAULT_HEIGHT),sensor.name,self.text_option)
                    i += 1
        self.qp.end()

class TactileArrayTimeline(TimelineRenderer):
    def __init__(self, timeline, height=80):
        TimelineRenderer.__init__(self, timeline, msg_combine_px=height)

    def draw_timeline_segment(self, painter, topic, start, end, x, y, width, height):
        bag_timeline = self.timeline.scene()
        for bag, entry in bag_timeline.get_entries_with_bags([topic], rospy.Time(start), rospy.Time(end)):
            topic, msg, t = bag_timeline.read_message(bag, entry.position)
            color = get_color(msg)
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color, 5))

            p_x = self.timeline.map_stamp_to_x(t.to_sec())
            painter.drawLine(p_x, y, p_x, y+height)

class TactileArrayBagPlugin(Plugin):
    def __init__(self):
        pass

    def get_view_class(self):
        return TactileArrayPanel

    def get_renderer_class(self):
        return TactileArrayTimeline

    def get_message_types(self):
        return ['tactile_msgs/TactileState']


