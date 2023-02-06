from PyQt5.QtWidgets import QMainWindow, QHBoxLayout
from .fileManager import *
from .paintWidget import *
from .settingsWindow import SettingsWidget
from ..controller import utils as util 
from .splineWidget import SplineWidget
from .rosWidget import RosWidget
from ..controller import ControlPanel

class MainWindow(QMainWindow):

    def __init__(self, settings):
        super(MainWindow, self).__init__()

        self.setWindowTitle("Crazydraw")
        self.setWindowIcon(QtGui.QIcon(QDir.currentPath() + '/gui/res/icons/drone.png'))

        tabs = QTabWidget()
        tabs.setTabPosition(QTabWidget.West)

        draw_window = DrawWindow(settings)
        spline_window = SplineWindow(settings)
        ros_window = RosWindow(settings)
        settings_window = SettingsWidget(settings)

        tabs.addTab(draw_window, "Draw")
        tabs.addTab(spline_window, "Spline")
        tabs.addTab(ros_window, "ROS")
        tabs.addTab(settings_window, "Settings")

        self.setCentralWidget(tabs)


class DrawWindow(QWidget):
    def __init__(self, settings):
        super(DrawWindow, self).__init__()

        layout = QHBoxLayout()

        paintWidget = paintMainWidget(settings)

        fileManager = FileManagerDraw(settings.get_trajectory_path())

        layout.addWidget(paintWidget)
        layout.addWidget(fileManager)

        self.setLayout(layout)


class SplineWindow(QWidget):

    def __init__(self, settings):
        super(SplineWindow, self).__init__()

        layout = QHBoxLayout()
        self.settings = settings

        self.file_manager = FileManagerSpline(settings)
        self.spline_widget = SplineWidget(settings)

        self.file_manager.bttn_spline.clicked.connect(self.plot_spline)
        self.file_manager.bttn_save.clicked.connect(self.save_spline)
        self.file_manager.bttn_compare.clicked.connect(self.plot_comparison)

        layout.addWidget(self.file_manager)
        layout.addWidget(self.spline_widget)

        self.setLayout(layout)

    def plot_spline(self):
        self.spline_widget.plot_spline(self.file_manager.selected_file, self.file_manager.time_scale, self.file_manager.interpolation_scale)

    def save_spline(self):
        self.spline_widget.save_spline(self.file_manager.selected_file,
                                       self.settings.get_polynomials_path(),
                                       30)
    def plot_comparison(self):
        self.spline_widget.plot_comparison(self.file_manager.selected_file, self.file_manager.time_scale, self.file_manager.interpolation_scale)

class RosWindow(QWidget):

    def __init__(self, settings):
        super(RosWindow, self).__init__()

        layout = QHBoxLayout()
        self.file_manager = FileManagerRos(settings.get_trajectory_path())
        self.ros_widget = RosWidget()

        self.settings = settings

        layout.addWidget(self.file_manager)
        layout.addWidget(self.ros_widget)

        self.setLayout(layout)

        self.cp = ControlPanel()        
        self.ros_widget.bttn_start.clicked.connect(self.start_experiment)
        self.ros_widget.bttn_take_off.clicked.connect(self.takeoff)
        self.ros_widget.bttn_land.clicked.connect(self.land)
        self.ros_widget.bttn_stop.clicked.connect(self.stop)
        self.file_manager.bttn_fly.clicked.connect(self.fly)
        self.ros_widget.bttn_direct.clicked.connect(self.send_direct)
        self.ros_widget.bttn_plot.clicked.connect(self.plot)

    def fly(self):

        id = self.file_manager.id_value.text()
        if id is None:
            id_list = [0]
        else:
            id_list = id.split(",")

        # height = self.file_manager.take_off_height.text()
        # if height is None:
        #     height = 1
        # else:
        #     height = int(height)
        height = 1 # Placeholder, reference height computed later as current height.
        for id in id_list:
            self.cp.send_full_trajectory(file_name=self.file_manager.selected_file, id=int(id), time_scale=self.settings.get_time_scale(), interpolation_scale=self.settings.get_interpolation_scale(),height=height)

    def start_experiment(self):
        self.cp.start_experiment()

    def takeoff(self):
        self.cp.takeoff()

    def land(self):
        self.cp.land()

    def stop(self):
        self.cp.stop()

    def send_direct(self):
        self.cp.goto(
            int(self.ros_widget.direct_id.text()),
            float(self.ros_widget.direct_t.text()),
            float(self.ros_widget.direct_x.text()),
            float(self.ros_widget.direct_y.text()),
            float(self.ros_widget.direct_z.text()))

    def plot(self):
        self.cp.plot()