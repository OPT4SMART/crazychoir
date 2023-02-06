from PyQt5.QtWidgets import QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from ..controller.spline import DrawSpline
from matplotlib.widgets import CheckButtons
from ..controller import utils 
from .custom_dialogs import SaveDialog
from datetime import datetime


class SplineWidget(QWidget):

    def __init__(self,settings):
        super(SplineWidget, self).__init__()
        self.ax = None
        self.check = None
        self.toolbar = None
        self.canvas = None
        self.file_name = None
        self.settings = settings

        self.setMinimumWidth(int(settings.get_screen_res().width()*0.15))
        self.setMaximumWidth(int(settings.get_screen_res().width() * 0.7))
        self.setContentsMargins(5,5,5,5)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

    def plot_spline(self, file_name, time_scale, interpolation_scale):

        if file_name is not None:
            self.layout.removeWidget(self.canvas)
            self.layout.removeWidget(self.toolbar)
            self.canvas = MplCanvasSpline()

            self.toolbar = NavigationToolbar(self.canvas)

            self.ax = []
            x, y = DrawSpline.plot_cubic_spline(file_name, self.canvas.fig, self.canvas.axes, time_scale, interpolation_scale)
            self.ax.append(x)
            self.ax.append(y)

            self.__make_checkBox__()
            self.check[0].on_clicked(self.__on_check_1__)
            self.check[1].on_clicked(self.__on_check_2__)

            self.layout.addWidget(self.toolbar)
            self.layout.addWidget(self.canvas)
            self.setLayout(self.layout)

    def plot_comparison(self,file_name, time_scale, interpolation_scale):
        DrawSpline.spline_check(file_name,time_scale, interpolation_scale)

    def __make_checkBox__(self):

        self.check = []
        i = 0
        ax = []
        ax.append(self.canvas.fig.add_axes([0.9, 0.65, 0.1, 0.15]))
        ax.append(self.canvas.fig.add_axes([0.9, 0.25, 0.1, 0.15]))

        for plot in self.ax:
            labels = [str(line.get_label()) for line in plot]
            visibility = [line.get_visible() for line in plot]
            self.check.append(CheckButtons(ax[i], labels, visibility))
            i += 1

    def __on_check_1__(self, label):

        ax = self.ax[0]
        labels = [str(line.get_label()) for line in ax]

        index = labels.index(label)
        ax[index].set_visible(not ax[index].get_visible())

        tmp = ax.copy()
        # pop not visible
        i = 0
        for line in ax:
            if not line.get_visible():
                tmp.pop(i)
            else:
                i += 1

        if len(tmp) > 0:
            max_value = utils.get_max_value(tmp)
            self.canvas.axes[0].set_ylim(-max_value, max_value)

        self.canvas.draw()

    def __on_check_2__(self, label):

        ax = self.ax[1]
        labels = [str(line.get_label()) for line in ax]

        index = labels.index(label)
        ax[index].set_visible(not ax[index].get_visible())

        tmp = ax.copy()
        # pop not visible
        i = 0
        for line in ax:
            if not line.get_visible():
                tmp.pop(i)
            else:
                i += 1

        if len(tmp) > 0:
            max_value = utils.get_max_value(tmp)
            self.canvas.axes[1].set_ylim(-max_value, max_value)

        self.canvas.draw()

    def save_spline(self, source_file, save_path, interpolation_distance):

        dlg = SaveDialog()
        result = dlg.exec()

        if result == 1:
            filename = dlg.filename.text()
            if filename == "":
                filename = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
            filename = "/"+ filename + "_poly.csv"
            utils.DrawSpline.print_poly_to_file(source_file, save_path+filename, interpolation_distance)


class MplCanvasSpline(FigureCanvasQTAgg):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = []
        self.axes.append(self.fig.add_subplot(2, 1, 1, autoscale_on=True))
        self.axes.append(self.fig.add_subplot(2, 1, 2, autoscale_on=True))
        super(MplCanvasSpline, self).__init__(self.fig)
