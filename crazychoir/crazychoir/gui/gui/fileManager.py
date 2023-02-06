import os
from PyQt5 import QtWidgets
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from ..controller.spline import DrawSpline
from .custom_dialogs import DeleteDialog
from ..controller.utils import *
# from .splineWidget import SplineWidget


class FileManager(QWidget):
    def __init__(self, rel_path):
        super().__init__()
        hlay = QHBoxLayout(self)
        self.treeview = QListView()
        hlay.addWidget(self.treeview)

        self.dirModel = QFileSystemModel()
        self.dirModel.setRootPath(QDir.rootPath())
        self.dirModel.setFilter(QDir.NoDotAndDotDot | QDir.Files |QDir.Writable | QDir.Readable)

        self.treeview.setModel(self.dirModel)
        self.treeview.setRootIndex(self.dirModel.index(rel_path))

        self.treeview.setMinimumWidth(150)
        self.treeview.setMaximumWidth(500)


class FileManagerMain(QWidget):

    def __init__(self, rel_path):
        super().__init__()

        self.selected_file = None
        self.tmp_file = rel_path + "/.tmp.csv"
        self.pagelayout = QVBoxLayout()
        self.toolbar = QtWidgets.QToolBar()
        self.toolbar.setStyleSheet("QToolBar{spacing:5px;}")
        self.file_manager_widget = FileManager(rel_path)

        self.pagelayout.addWidget(self.toolbar)
        self.pagelayout.addWidget(self.file_manager_widget)

        self.bttn_delete = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/icons8-rimuovere-100.png"), "DELETE", self)
        self.bttn_delete.setDisabled(True)
        self.bttn_delete.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_delete)

        self.setLayout(self.pagelayout)


class FileManagerDraw(FileManagerMain):

    def __init__(self, rel_path):
        super().__init__(rel_path)

        self.bttn_preview = QPushButton("PREVIEW", self)
        self.bttn_preview.clicked.connect(self.preview_trajectory)
        self.bttn_preview.setDisabled(True)
        self.bttn_preview.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_preview)

        self.bttn_delete.clicked.connect(self.delete_file)
        self.file_manager_widget.treeview.clicked.connect(self.on_clicked)

    def delete_file(self):

        dlg = DeleteDialog(self.selected_file)

        if dlg.exec() == QMessageBox.Yes:
            os.remove(self.selected_file)
            self.bttn_delete.setDisabled(True)
            self.bttn_preview.setDisabled(True)

    def on_clicked(self, index):
        self.selected_file = self.file_manager_widget.dirModel.fileInfo(index).absoluteFilePath()

        if self.selected_file == self.tmp_file:
            self.bttn_delete.setDisabled(True)
            self.bttn_preview.setDisabled(True)
        else:
            self.bttn_delete.setDisabled(False)
            self.bttn_preview.setDisabled(False)

    def preview_trajectory(self):
        dlg = QDialog(self)
        preview = GraphPreview(self.selected_file)
        layout = QVBoxLayout()
        layout.addWidget(preview)
        dlg.setWindowTitle("Preview!")
        dlg.setLayout(layout)
        dlg.exec()


class FileManagerSpline(FileManagerMain):

    def __init__(self, settings):
        super().__init__(settings.get_trajectory_path())

        self.file_manager_widget.treeview.setMinimumWidth(450)

        self.bttn_spline = QPushButton("SPLINE", self)
        self.bttn_spline.setDisabled(True)
        self.bttn_spline.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_spline)

        self.bttn_save = QPushButton("SAVE", self)
        self.bttn_save.setDisabled(True)
        self.bttn_save.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_save)

        self.bttn_compare = QPushButton("COMPARE", self)
        self.bttn_compare.setDisabled(True)
        self.bttn_compare.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_compare)

        self.bttn_delete.clicked.connect(self.delete_file)
        self.file_manager_widget.treeview.clicked.connect(self.on_clicked)

        self.settings = settings


        #slider interpolation
        self.slider_interpolation = DoubleSlider(orientation=Qt.Horizontal)
        self.slider_interpolation.setMaximum(1.0)
        self.slider_interpolation.setMinimum(0.01)
        self.slider_interpolation.setSingleStep(0.01)
        self.slider_interpolation.setValue(0.2)
        self.slider_interpolation.valueChanged.connect(self.scale_interpolation)
        self.interpolation_scale = float(self.slider_interpolation.value())

        self.label_interpolation = QLabel()
        self.label_interpolation.setText("Interpolation distance = " + str(self.slider_interpolation.value()) + "m")
        self.label_interpolation.setAlignment(Qt.AlignCenter)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.label_interpolation)
        layout.addWidget(self.slider_interpolation)
        self.pagelayout.addLayout(layout)

        #slider time
        self.slider = DoubleSlider(orientation=Qt.Horizontal)
        self.slider.setMaximum(10)
        self.slider.setMinimum(0.5)
        self.slider.setSingleStep(0.5)
        self.slider.setValue(5)
        self.slider.valueChanged.connect(self.scale_time)
        self.time_scale = float(self.slider.value())

        self.label = QLabel()
        self.label.setText("Time scale = " + str(self.slider.value()) + "x")
        self.label.setAlignment(Qt.AlignCenter)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.slider)

        self.pagelayout.addLayout(layout)

    def scale_interpolation(self):
        self.label_interpolation.setText("Interpolation distance = " + str(self.slider_interpolation.value()) + "m")
        self.interpolation_scale = float(self.slider_interpolation.value())
        self.settings.set_interpolation_scale(self.interpolation_scale)

    def scale_time(self):
        self.label.setText("Time scale = " + str(self.slider.value()) + "x")
        self.time_scale = float(self.slider.value())
        self.settings.set_time_scale(self.time_scale)

    def delete_file(self):

        dlg = DeleteDialog(self.selected_file)

        if dlg.exec() == QMessageBox.Yes:
            os.remove(self.selected_file)
            self.bttn_delete.setDisabled(True)
            self.bttn_spline.setDisabled(True)
            self.bttn_save.setDisabled(True)
            self.bttn_compare.setDisabled(True)


    def on_clicked(self, index):
        self.selected_file = self.file_manager_widget.dirModel.fileInfo(index).absoluteFilePath()

        if self.selected_file == self.tmp_file:
            self.bttn_delete.setDisabled(True)
            self.bttn_spline.setDisabled(True)
            self.bttn_save.setDisabled(True)
            self.bttn_compare.setDisabled(True)
        else:
            self.bttn_delete.setDisabled(False)
            self.bttn_spline.setDisabled(False)
            self.bttn_save.setDisabled(False)
            self.bttn_compare.setDisabled(False)


class FileManagerRos(FileManagerMain):

    def __init__(self, rel_path):
        super().__init__(rel_path)

        self.bttn_fly = QPushButton("FLY", self)
        self.bttn_fly.setDisabled(True)
        self.bttn_fly.setFixedHeight(70)
        self.toolbar.addWidget(self.bttn_fly)

        self.toolbar.addWidget(QLabel("ID:"))
        self.id_value = QLineEdit()
        self.id_value.setText(str(0))
        self.id_value.setFixedWidth(50)
        self.toolbar.addWidget(self.id_value)

        # self.toolbar.addWidget(QLabel("Height:"))
        # self.take_off_height = QLineEdit()
        # self.take_off_height.setFixedWidth(50)
        # self.take_off_height.setText(str(1))
        # self.toolbar.addWidget(self.take_off_height)

        self.bttn_delete.clicked.connect(self.delete_file)
        self.file_manager_widget.treeview.clicked.connect(self.on_clicked)

    def delete_file(self):

        dlg = DeleteDialog(self.selected_file)

        if dlg.exec() == QMessageBox.Yes:
            os.remove(self.selected_file)
            self.bttn_delete.setDisabled(True)
            self.bttn_fly.setDisabled(True)

    def on_clicked(self, index):
        self.selected_file = self.file_manager_widget.dirModel.fileInfo(index).absoluteFilePath()

        if self.selected_file == self.tmp_file:
            self.bttn_delete.setDisabled(True)
            self.bttn_fly.setDisabled(True)
        else:
            self.bttn_delete.setDisabled(False)
            self.bttn_fly.setDisabled(False)