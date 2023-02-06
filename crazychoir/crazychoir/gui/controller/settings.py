import yaml
import io
import os

from PyQt5.QtCore import QDir
from PyQt5.QtWidgets import QMessageBox


class SettingsParser:
    def __init__(self, app):

        self.polynomials_path = None
        self.trajectory_path = None
        self.paint_widget_size = None
        self.screen_res = None
        self.enable_ros = None
        self.time_scale = 5.0
        self.interpolation_scale = 0.2
        self.data = {}

        self.polynomials_path_new = None
        self.trajectory_path_new = None
        self.paint_size_meters_new = None
        self.enable_ros_new = None

        self.app = app

        self.check_settings_file()

        self.check_dirs()

        self.enable_ros = bool(self.data["enable_ros"])

    def check_dirs(self):

        if os.path.isabs(self.data["data_storage"]["trajectory_directory"]):
            self.trajectory_path = self.data["data_storage"]["trajectory_directory"]
        else:
            self.trajectory_path = QDir.currentPath() + "/" + self.data["data_storage"]["trajectory_directory"]

        if os.path.isabs(self.data["data_storage"]["polynomials_directory"]):
            self.polynomials_path = self.data["data_storage"]["polynomials_directory"]
        else:
            self.polynomials_path = QDir.currentPath() + "/" + self.data["data_storage"]["polynomials_directory"]

        if not os.path.isdir(self.trajectory_path):
            os.makedirs(self.trajectory_path)

        if not os.path.isdir(self.polynomials_path):
            os.makedirs(self.polynomials_path)

    def check_settings_file(self):

        if not os.path.isfile(QDir.currentPath() + "/settings.yaml"):
            try:
                data = {
                    'area_settings': {
                        'paint_square_size': [
                            4,
                            4
                        ]
                    },
                    'data_storage': {
                        'trajectory_directory': 'saves/trajectory',
                        'polynomials_directory': 'saves/poly'
                    },
                    'enable_ros': False
                }

                # Write YAML file
                with io.open('settings.yaml', 'w', encoding='utf8') as outfile:
                    yaml.dump(data, outfile, default_flow_style=False, allow_unicode=True)
            except:
                msg = QMessageBox()
                msg.setIcon(QMessageBox.Critical)
                msg.setText("Mannagg")
                msg.setInformativeText('Errore scrittura file settings.yaml')
                msg.setWindowTitle("Aiuto")
                msg.exec_()

        with open(QDir.currentPath() + "/settings.yaml") as stream:
            try:
                self.data = yaml.safe_load(stream)
                print(self.data)
            except yaml.YAMLError as exc:
                print(exc)
                stream.close()
                os.remove(QDir.currentPath() + "/settings.yaml")
                self.check_settings_file()

    def get_screen_res(self):
        return self.screen_res

    def set_screen_res(self, screen_res):
        self.screen_res = screen_res

        size = self.data["area_settings"]["paint_square_size"]

        self.paint_widget_size = []
        self.paint_widget_size.append(int(size[0] * screen_res.height() / 3 / size[1]))
        self.paint_widget_size.append(int(screen_res.height() / 3))

    def get_paint_size_scaled(self):

        return self.paint_widget_size

    def get_area_size_meters(self):

        return self.data["area_settings"]["paint_square_size"]

    def get_trajectory_path(self):
        return self.trajectory_path

    def get_polynomials_path(self):
        return self.polynomials_path

    def get_scale_factor(self):

        x_size, y_size = self.get_area_size_meters()
        x_widget, y_widget = self.get_paint_size_scaled()

        y_dist = int(y_widget / y_size)
        x_dist = int(x_widget / x_size)

        if x_dist == y_dist:
            return x_dist
        else:
            return 0

    def get_paint_widget_middle(self):
        tmp = []
        tmp.append(int(self.get_paint_size_scaled()[0]/2))
        tmp.append(int(self.get_paint_size_scaled()[1]/2))
        return tmp

    def save_data(self):

        try:
            data = {
                'area_settings': {
                    'paint_square_size': [
                        self.paint_size_meters_new[0],
                        self.paint_size_meters_new[1]
                    ]
                },
                'data_storage': {
                    'trajectory_directory': self.trajectory_path_new,
                    'polynomials_directory': self.polynomials_path_new
                },
                'enable_ros': self.enable_ros_new
            }

            os.remove(QDir.currentPath() + "/settings.yaml")

            # Write YAML file
            with io.open('settings.yaml', 'w', encoding='utf8') as outfile:
                yaml.dump(data, outfile, default_flow_style=False, allow_unicode=True)

            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("Settings Saved!")
            msg.setInformativeText('Application is going to restart!')
            msg.setWindowTitle("O w o")
            msg.exec_()

            self.app.exit(1)
        except:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Mannagg")
            msg.setInformativeText('Errore scrittura file settings.yaml')
            msg.setWindowTitle("Aiuto")
            msg.exec_()

    def set_interpolation_scale(self, value):
        self.interpolation_scale = value

    def set_time_scale(self, value):
        self.time_scale = value

    def get_interpolation_scale(self):
        return self.interpolation_scale

    def get_time_scale(self):
        return self.time_scale