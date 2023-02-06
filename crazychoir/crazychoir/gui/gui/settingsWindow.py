from PyQt5.QtCore import QDir, Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QFileDialog, QSlider, \
    QMessageBox, QCheckBox


class SettingsWidget(QWidget):
    def __init__(self, settings):
        super().__init__()

        self.ros_checkbox = None
        self.polynomials_path_new = None
        self.trajectory_path_new = None
        self.size_info = None
        self.size_value = None
        self.trajectory_file = None
        self.polynomials_label = None
        self.polynomials_value = None
        self.trajectory_label = None
        self.trajectory_value = None

        self.settings = settings

        self.layout = QVBoxLayout()

        self.layout.addWidget(self.get_trajectory_saves())
        self.layout.addWidget(self.get_polynomial_saves())
        self.layout.addWidget(self.get_size_meters())
        self.layout.addWidget(self.get_ros_checkbox())

        self.bttn_save = QPushButton("SAVE SETTINGS", self)
        self.bttn_save.setFixedHeight(70)
        self.bttn_save.clicked.connect(self.save_settings)
        self.layout.addWidget(self.bttn_save)

        self.setLayout(self.layout)

    def get_trajectory_saves(self):
        row = QWidget()
        row_layout = QHBoxLayout()

        self.trajectory_label = QLabel("Trajectory files location:")
        self.trajectory_value = QLineEdit()
        self.trajectory_value.setPlaceholderText(self.settings.get_trajectory_path())

        self.trajectory_file = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/icons8-directory-100.png"),
                                           "SELECT FILE", self)
        self.trajectory_file.clicked.connect(self.get_trajectory_file_dialog)

        row_layout.addWidget(self.trajectory_label)
        row_layout.addWidget(self.trajectory_value)
        row_layout.addWidget(self.trajectory_file)

        row.setLayout(row_layout)

        return row

    def get_polynomial_saves(self):
        row = QWidget()
        row_layout = QHBoxLayout()

        self.polynomials_label = QLabel("Polynomials files location:")
        self.polynomials_value = QLineEdit()
        self.polynomials_value.setPlaceholderText(self.settings.get_polynomials_path())

        self.polynomials_file = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/icons8-directory-100.png"),
                                            "SELECT FILE", self)
        self.polynomials_file.clicked.connect(self.get_polynomial_file_dialog)

        row_layout.addWidget(self.polynomials_label)
        row_layout.addWidget(self.polynomials_value)
        row_layout.addWidget(self.polynomials_file)

        row.setLayout(row_layout)

        return row

    def get_size_meters(self):
        box = QWidget()
        box_layout = QVBoxLayout()
        box.setLayout(box_layout)

        self.size_value = []
        self.size_info = []

        # Width
        row = QWidget()
        row_layout = QHBoxLayout()
        size_label = QLabel("Width size (meters):")
        self.size_value.append(QSlider(Qt.Horizontal))
        self.size_info.append(QLabel())

        row_layout.addWidget(size_label)
        row_layout.addWidget(self.size_value[0])
        row_layout.addWidget(self.size_info[0])

        row.setLayout(row_layout)
        box_layout.addWidget(row)

        # Height
        row = QWidget()
        row_layout = QHBoxLayout()
        size_label = QLabel("Height size (meters):")
        self.size_value.append(QSlider(Qt.Horizontal))
        self.size_info.append(QLabel())

        row_layout.addWidget(size_label)
        row_layout.addWidget(self.size_value[1])
        row_layout.addWidget(self.size_info[1])

        row.setLayout(row_layout)
        box_layout.addWidget(row)

        for slider in self.size_value:
            slider.setMinimum(1)
            slider.setMaximum(6)
            slider.setSingleStep(1)
            slider.valueChanged.connect(self.slider_value_changed)

        size = self.settings.get_area_size_meters()
        self.size_value[0].setValue(size[0])
        self.size_value[1].setValue(size[1])

        self.slider_value_changed()

        return box

    def get_ros_checkbox(self):
        row = QWidget()
        row_layout = QHBoxLayout()

        label = QLabel("Enable ROS:")
        self.ros_checkbox = QCheckBox()

        if self.settings.enable_ros:
            self.ros_checkbox.setChecked(True)
            self.ros_checkbox.setText("Enabled")
        else:
            self.ros_checkbox.setChecked(False)
            self.ros_checkbox.setText("Disabled")

        self.ros_checkbox.stateChanged.connect(self.check_value_changed)

        row_layout.addWidget(label)
        row_layout.addWidget(self.ros_checkbox)

        row.setLayout(row_layout)

        return row

    def slider_value_changed(self):
        self.size_info[0].setText(str(self.size_value[0].value()))
        self.size_info[1].setText(str(self.size_value[1].value()))

    def check_value_changed(self):
        if self.ros_checkbox.isChecked():
            self.ros_checkbox.setText("Enabled")
        else:
            self.ros_checkbox.setText("Disabled")

    def get_trajectory_file_dialog(self):
        self.trajectory_path_new = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
        self.trajectory_value.setPlaceholderText(self.trajectory_path_new)

    def get_polynomial_file_dialog(self):
        self.polynomials_path_new = str(QFileDialog.getExistingDirectory(self, "Select Directory"))
        self.polynomials_value.setPlaceholderText(self.polynomials_path_new)

    def save_settings(self):
        self.settings.paint_size_meters_new = []
        self.settings.paint_size_meters_new.append(self.size_value[0].value())
        self.settings.paint_size_meters_new.append(self.size_value[1].value())

        if self.trajectory_path_new is None:
            self.settings.trajectory_path_new = self.settings.get_trajectory_path()
        else:
            self.settings.trajectory_path_new = self.trajectory_path_new

        if self.polynomials_path_new is None:
            self.settings.polynomials_path_new = self.settings.get_polynomials_path()
        else:
            self.settings.polynomials_path_new = self.polynomials_path_new

        self.settings.enable_ros_new = self.ros_checkbox.isChecked()

        self.settings.save_data()
