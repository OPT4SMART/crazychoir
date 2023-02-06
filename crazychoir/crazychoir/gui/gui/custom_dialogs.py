from PyQt5.QtWidgets import QDialog, QDialogButtonBox, QLabel, QVBoxLayout, QLineEdit, QMessageBox


class SaveDialog(QDialog):
    def __init__(self):
        super().__init__()

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        message = QLabel("Nome file")
        self.filename = QLineEdit()
        #self.filename.setPlaceholderText("trajectory")

        self.layout.addWidget(message)
        self.layout.addWidget(self.filename)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)
        self.setFixedSize(150,150)
        #self.setWindowTitle("Salva la traiettoria")

class DeleteDialog(QMessageBox):
    def __init__(self,filename):
        super().__init__(None)

        tmp = filename.split('/')
        file_name = tmp[len(tmp) - 1]
        message = f'Cancellare {file_name}?'

        self.setWindowTitle(" ")
        self.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        self.setIcon(QMessageBox.Question)
        self.setText(message)