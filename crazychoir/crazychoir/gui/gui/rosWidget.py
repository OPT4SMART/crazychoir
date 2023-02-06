from PyQt5.QtCore import QDir, Qt
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QHBoxLayout, QLabel, QLineEdit


class RosWidget(QWidget):

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(10)

        main_toolbar = QHBoxLayout()
        second_toolbar = QHBoxLayout()

        self.bttn_start = QPushButton("START", self)
        self.bttn_take_off = QPushButton(QIcon(QDir.currentPath() + "/gui/res/icons/drone.png"), "TAKE-OFF", self)
        self.bttn_land = QPushButton("LAND", self)
        self.bttn_stop = QPushButton("STOP", self)
        self.bttn_plot = QPushButton("PLOT", self)

        main_toolbar.addWidget(self.bttn_take_off)       
        main_toolbar.addWidget(self.bttn_land)
        main_toolbar.addWidget(self.bttn_stop)
        second_toolbar.addWidget(self.bttn_start)       
        second_toolbar.addWidget(self.bttn_plot)

        layout.addLayout(main_toolbar)
        layout.addLayout(second_toolbar)
        layout.addSpacing(50)

        # Direct command tool
        layout.addWidget(QLabel("Send direct command"))

        row = QHBoxLayout()
        row.addWidget(QLabel("ID:"))
        self.direct_id = QLineEdit()
        row.addWidget(self.direct_id)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("x:"))
        self.direct_x = QLineEdit()
        row.addWidget(self.direct_x)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("y:"))
        self.direct_y = QLineEdit()
        row.addWidget(self.direct_y)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("z:"))
        self.direct_z = QLineEdit()
        row.addWidget(self.direct_z)
        layout.addLayout(row)

        row = QHBoxLayout()
        row.addWidget(QLabel("time (seconds):"))
        self.direct_t = QLineEdit()
        row.addWidget(self.direct_t)
        layout.addLayout(row)
        
        self.bttn_direct = QPushButton("SEND DIRECT COMMAND")
        layout.addWidget(self.bttn_direct)

        self.setLayout(layout)