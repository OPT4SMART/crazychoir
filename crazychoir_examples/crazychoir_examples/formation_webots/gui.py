import sys
from crazychoir.gui.gui import MainWindow
from crazychoir.gui.controller import SettingsParser
from qt_material import apply_stylesheet
from PyQt5 import QtWidgets
import rclpy

def main(args=None):
    
    rclpy.init(args=args)

    # Extra settings for application theme
    extra = {
        'density_scale': '2',
    }

    app = QtWidgets.QApplication(sys.argv)
    screen_resolution = app.desktop().screenGeometry()

    exit_code = 1

    while exit_code == 1:
        settings = SettingsParser(app)
        settings.set_screen_res(screen_resolution)

        window = MainWindow(settings)
        apply_stylesheet(app, theme='dark_teal.xml', extra=extra)

        window.show()
        exit_code = app.exec_()
        window.close()

if __name__ == '__main__':
    main()