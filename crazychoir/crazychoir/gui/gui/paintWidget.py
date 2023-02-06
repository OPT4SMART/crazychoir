import os.path
import time
from PyQt5 import QtGui, QtWidgets
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QWidget, QAction, QVBoxLayout
from .custom_dialogs import *
from datetime import datetime
from ..controller import *


class paintMainWidget(QWidget):
    def __init__(self, settings):
        super().__init__()
        pagelayout = QVBoxLayout()
        toolbar = QtWidgets.QToolBar()
        toolbar.setStyleSheet("QToolBar{spacing:30px;}");
        paint_widget = paint(settings)

        pagelayout.addWidget(toolbar)
        pagelayout.addWidget(paint_widget)

        bttn_clear = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/icons8-clear-58.png"), "Clear", self)
        bttn_clear.clicked.connect(paint_widget.clear_all)

        bttn_save = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/icons8-save-64.png"), "Save", self)
        bttn_save.clicked.connect(paint_widget.save_data)

        toolbar.addWidget(bttn_clear)
        toolbar.addWidget(bttn_save)

        self.setLayout(pagelayout)


class paint(QtWidgets.QLabel):
    def __init__(self, settings):
        super().__init__()

        self.scale_size = settings.get_scale_factor()
        self.csv_file = None
        self.saves_dir = settings.get_trajectory_path()
        self.settings = settings

        self.setFixedSize(settings.get_paint_size_scaled()[0], settings.get_paint_size_scaled()[1])

        qsize = QSize(settings.get_paint_size_scaled()[0], settings.get_paint_size_scaled()[1])
        self.canvas = QtGui.QPixmap(qsize)

        self.clear_all()
        self.paint_grid()

        self.last_x, self.last_y, self.start_time, self.start_x, self.start_y = None, None, None, None, None

    def paint_grid(self):

        x_widget, y_widget = self.settings.get_paint_size_scaled()

        painter = QtGui.QPainter(self.pixmap())
        pen = painter.pen()
        pen.setWidth(2)
        painter.setPen(pen)

        # vertical lines
        index = 0
        while index < x_widget:
            index += self.scale_size
            painter.drawLine(index, 0, index, y_widget)

        # vertical lines
        index = 0
        while index < y_widget:
            index += self.scale_size
            painter.drawLine(0, index, x_widget, index)

        # half - meter lines
        # vertical lines
        pen.setColor(QtGui.QColor("#6d737d"))
        pen.setWidth(1)
        painter.setPen(pen)
        index = int(self.scale_size/-2)
        while index < x_widget:
            index += self.scale_size
            painter.drawLine(index, 0, index, y_widget)

        # vertical lines
        index = int(self.scale_size/-2)
        while index < y_widget:
            index += self.scale_size
            painter.drawLine(0, index, x_widget, index)

        # Draw central point
        pen.setColor(QtGui.QColor("#ff9d00"))
        pen.setWidth(10)
        painter.setPen(pen)
        painter.drawPoint(int(x_widget/2), int(y_widget/2))

        painter.end()


    def clear_all(self):
        self.canvas.fill(QtGui.QColor("white"))
        self.setPixmap(self.canvas)
        self.paint_grid()

        # Init file save
        if not os.path.isdir(self.saves_dir):
            os.mkdir(self.saves_dir)

        if self.csv_file is not None:
            self.csv_file.close()

        self.csv_file = open(self.saves_dir + "/" + ".tmp.csv", mode='w')
        self.csv_file.write("x, y, time\n")

        self.csv_file.flush()

        self.last_x, self.last_y, self.start_time = None, None, None
        self.csv_file.write(f'{str(0)}, {str(0)}, {str(0)}\n')

    def save_data(self, s):

        dlg = SaveDialog()
        result = dlg.exec()

        if result == 1:

            filename = dlg.filename.text()
            if filename == "":
                filename = datetime.now().strftime('%d-%m-%Y_%H-%M-%S')
            filename = filename + ".csv"

            self.csv_file.close()
            old_file = self.saves_dir + "/.tmp.csv"
            new_file = self.saves_dir + "/" + filename

            if os.path.isfile(new_file):
                os.remove(new_file)
            os.rename(old_file, new_file)

            self.csv_file = open(self.saves_dir + "/" + ".tmp.csv", mode='w')
            self.csv_file.write("x, y, time\n")

    def mouseMoveEvent(self, e):

        if self.last_x is None:  # First event.
            self.last_x = e.x()
            self.last_y = e.y()
            self.start_x = self.last_x
            self.start_y = self.last_y
            self.start_time = time.time()
            return  # Ignore the first time.

        painter = QtGui.QPainter(self.pixmap())

        pen = painter.pen()
        pen.setWidth(3)
        pen.setColor(QtGui.QColor("#e82727"))
        painter.setPen(pen)

        painter.drawLine(self.last_x, self.last_y, e.x(), e.y())
        painter.end()
        self.update()

        # Update the origin for next time.
        self.last_x = e.x()
        self.last_y = e.y()
        #self.csv_file.write(f'{str((self.last_x -self.settings.get_paint_widget_middle()[0] - self.start_x)/ self.scale_size )}, {str((self.settings.get_screen_res().height()*2/3 - (self.last_y + self.settings.get_paint_widget_middle()[1] + self.start_y)) / self.scale_size)}, {str((time.time() - self.start_time)*2.5)}\n')
        self.csv_file.write(f'{str((self.last_x - self.start_x)/ self.scale_size )}, {str((self.last_y - self.start_y) / -self.scale_size)}, {str((time.time() - self.start_time)*2.5)}\n')
