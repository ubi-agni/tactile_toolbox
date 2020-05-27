from rqt_bag.plugins.plugin import Plugin
from rqt_bag import TopicMessageView, TimelineRenderer
from python_qt_binding.QtCore import Qt, QPoint, QRectF
from python_qt_binding.QtWidgets import QWidget, QPushButton, QLabel, QLineEdit, \
    QVBoxLayout, QHBoxLayout, QSizePolicy, QFileDialog
from python_qt_binding.QtGui import QPainter, QBrush, QTextOption


from tactile_msgs.msg import TactileState
from urdf_parser_py.urdf import URDF

from tactile_helpers import ColorMap

from urdf_parser_py import urdf

import numpy as np
import rospy
import xacro


abs_color_map = ColorMap(4095,0)
colorNames = ["black" ,"lime" ,"yellow" ,"red"]
abs_color_map.append(colorNames)

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
        return abs_color_map.map(mean)
    else:  # 0
        return Qt.black

def paint_default_sensor(qp, sensor, i, j, w, h, text_option):
    color = get_average_taxel_color(sensor)
    qp.setBrush(QBrush(color))
    qp.drawEllipse(i * w, j * h, w, h)
    qp.drawText(QRectF(i * w, j * h,w,h),sensor.name, text_option)

def paint_sensor(qp, sensor, values, i, j, w, h, text_option):
    if isinstance(sensor, urdf.TactileArray):
        array = sensor.array
        # hack to re-arrange the values
        #reorder = [8, 4, 7, 9, 0, 13, 10 , 12, 5, 14, 15, 11, 1, 2, 3, 6]
        # bottom to top reorder = [4, 12, 13, 14, 1, 8, 15, 2, 0, 3, 6, 11, 7, 5, 9, 10]
        # top to bottom by block reorder = [10, 9, 5, 7, 11, 6, 3, 0, 2, 15, 8, 1, 14, 13, 12, 4]
        reorder = [10, 9, 5, 7, 11, 6, 3, 0, 2, 15, 8, 1, 14, 13, 12, 4]
        reordered_values = []
        for block_idx in range(len(values)/16):
            for k in range(16):
                reordered_values.append(values[block_idx*16 + reorder[k]])

        if array.rows > 0 and array.cols > 0:
            tw = w / array.cols
            th = h / array.rows
            if tw>th:
              tw = th
            for ti in range(array.cols):
                for tj in range(array.rows):
                    idx = tj * array.cols + ti
                    if idx < len(reordered_values):
                        color = abs_color_map.map(reordered_values[idx])
                        qp.setBrush(QBrush(color))
                        qp.drawRect(QRectF(i * w + ti*tw, j * h + tj* th ,tw,th))

            #p.drawText(QRectF(i * w, j * h,w,h), sensor.name, text_option)



class TactileArrayPanel(TopicMessageView):
    name = 'Tactile Array'
    MAX_COLUMNS = 10
    MIN_HEIGHT =  120
    def __init__(self, timeline, parent, topic):
        super(TactileArrayPanel, self).__init__(timeline, parent, topic)
        self.widget = QWidget()
        parent.layout().addWidget(self.widget)
        self.msg = None
        self.initWindow()
        self.paint_widget.paintEvent = self.paintEvent
        self.text_option = QTextOption()
        self.text_option.setAlignment(Qt.AlignCenter)
        self.loaded_filename = ""
        self.robot_description = ""
        self.sensors_initialized = False
        self.sensors = {}

    def initWindow(self):
        vlayout = QVBoxLayout()
        hparamlayout = QHBoxLayout()
        hpainterlayout = QHBoxLayout()
        self.rdparam_label = QLabel(self.widget)
        self.rdparam_label.setText('robot_description:')
        self.rdparam = QLineEdit(self.widget)
        self.rdparam.setText("/robot_description")
        self.rdparam_button = QPushButton("load from param", self.widget)
        self.rdparam_button.clicked.connect(self.param_load_robot_description_cb)
        self.rdfile_button = QPushButton("load from file", self.widget)
        self.rdfile_button.clicked.connect(self.file_load_robot_description_cb)
        self.rdparam.resize(200, 32)
        hparamlayout.addWidget(self.rdparam_label)
        hparamlayout.addWidget(self.rdparam)
        hparamlayout.addWidget(self.rdparam_button)
        self.paint_widget = QWidget()
        policy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        policy.setVerticalStretch(1)
        self.paint_widget.setSizePolicy(policy)
        hpainterlayout.addWidget(self.paint_widget)
        vlayout.addLayout(hparamlayout)
        vlayout.addWidget(self.rdfile_button)
        vlayout.addLayout(hpainterlayout)
        self.widget.setLayout(vlayout)

    def message_viewed(self, bag, msg_details):
        super(TactileArrayPanel, self).message_viewed(bag, msg_details)
        _, self.msg, _ = msg_details
        self.widget.update()

    def param_load_robot_description_cb(self):
        if rospy.has_param(self.rdparam.text()):
            self.robot_description = rospy.get_param(self.rdparam.text())
            self.load_robot_description()
        else:
            print (self.rdparam.text() , " not found on the param server")

    def file_load_robot_description_cb(self):
        dlg = QFileDialog()
        filename, filter_used = dlg.getOpenFileName(parent=self.widget,
            caption='Open xacro file', directory=self.loaded_filename,
            filter="xacro (*.xacro);;urdf (*.urdf)")
        if filename:
            xacro_file = open(filename, 'r')
            with xacro_file:
                robot_xacro = xacro.parse(xacro_file)
                xacro.process_doc(robot_xacro,{})
                self.robot_description = robot_xacro.toprettyxml(encoding="utf-8")
                self.loaded_filename = filename
                self.load_robot_description()
                

    def load_robot_description(self):
        try:
            robot = URDF.from_xml_string(self.robot_description)
            
            if len(robot.sensors):
                self.sensors = {}
                # remap with channel names // works only for arrays right now
                for sensor in robot.sensors:
                    self.sensors[sensor.tactile.channel] = sensor.tactile
                self.sensors_initialized = True
                self.widget.update()
            else:
                print (self.loaded_filename , " does not contain sensors")
        except Exception as e:
            print self.robot_description
            print("URDF parsing error: %s" % unicode(e))

    def paintEvent(self, event):

        self.qp = QPainter()
        self.qp.begin(self.paint_widget)
        rect = event.rect()

        if self.msg is None:
            self.qp.fillRect(0, 0, rect.width(), rect.height(), Qt.white)
        else:
            num_sensors = len(self.msg.sensors)
            if num_sensors:
                if num_sensors > self.MAX_COLUMNS:
                    sensor_width = rect.width() / self.MAX_COLUMNS;
                    sensor_height = max(self.MIN_HEIGHT, rect.height()/int(num_sensors/self.MAX_COLUMNS))
                else:
                    sensor_width = rect.width() / num_sensors
                    sensor_height = max(self.MIN_HEIGHT, rect.height())
                i = 0
                j = 0
                #print sensor_width, sensor_height

                for idx, sensor in enumerate(self.msg.sensors):

                    if (idx % self.MAX_COLUMNS == 0 and idx != 0):
                        i = 0
                        j +=1
                    if self.sensors_initialized:
                        if sensor.name in self.sensors:
                            paint_sensor(self.qp, self.sensors[sensor.name], sensor.values , i, j, sensor_width, sensor_height, self.text_option)
                        else:
                            print sensor.name, " not in sensors"
                            self.sensors_initialized = False
                            paint_default_sensor(self.qp, sensor, i, j, sensor_width, sensor_height, self.text_option)
                    else:
                        paint_default_sensor(self.qp, sensor, i, j, sensor_width, sensor_height, self.text_option)

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


