# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDoubleSpinBox, QGridLayout,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit,
    QMainWindow, QMenuBar, QPushButton, QSizePolicy,
    QSpinBox, QStatusBar, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1127, 900)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.horizontalLayout_3 = QHBoxLayout(self.tab_3)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.tabWidget_3 = QTabWidget(self.tab_3)
        self.tabWidget_3.setObjectName(u"tabWidget_3")
        self.tab_6 = QWidget()
        self.tab_6.setObjectName(u"tab_6")
        self.horizontalLayout_20 = QHBoxLayout(self.tab_6)
        self.horizontalLayout_20.setObjectName(u"horizontalLayout_20")
        self.horizontalLayout_19 = QHBoxLayout()
        self.horizontalLayout_19.setObjectName(u"horizontalLayout_19")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.socket_connection_pushButton = QPushButton(self.tab_6)
        self.socket_connection_pushButton.setObjectName(u"socket_connection_pushButton")

        self.horizontalLayout_4.addWidget(self.socket_connection_pushButton)

        self.socket_connection = QLineEdit(self.tab_6)
        self.socket_connection.setObjectName(u"socket_connection")

        self.horizontalLayout_4.addWidget(self.socket_connection)

        self.socket_disconnection_pushButton = QPushButton(self.tab_6)
        self.socket_disconnection_pushButton.setObjectName(u"socket_disconnection_pushButton")

        self.horizontalLayout_4.addWidget(self.socket_disconnection_pushButton)


        self.verticalLayout_3.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_16 = QHBoxLayout()
        self.horizontalLayout_16.setObjectName(u"horizontalLayout_16")
        self.add_motors = QPushButton(self.tab_6)
        self.add_motors.setObjectName(u"add_motors")

        self.horizontalLayout_16.addWidget(self.add_motors)

        self.open_xy = QPushButton(self.tab_6)
        self.open_xy.setObjectName(u"open_xy")

        self.horizontalLayout_16.addWidget(self.open_xy)

        self.close_xy = QPushButton(self.tab_6)
        self.close_xy.setObjectName(u"close_xy")

        self.horizontalLayout_16.addWidget(self.close_xy)

        self.send_motor_message = QPushButton(self.tab_6)
        self.send_motor_message.setObjectName(u"send_motor_message")

        self.horizontalLayout_16.addWidget(self.send_motor_message)


        self.verticalLayout_3.addLayout(self.horizontalLayout_16)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.motor_position_mode = QPushButton(self.tab_6)
        self.motor_position_mode.setObjectName(u"motor_position_mode")

        self.horizontalLayout_5.addWidget(self.motor_position_mode)

        self.set_zero_position = QPushButton(self.tab_6)
        self.set_zero_position.setObjectName(u"set_zero_position")

        self.horizontalLayout_5.addWidget(self.set_zero_position)

        self.motor_velocity_mode = QPushButton(self.tab_6)
        self.motor_velocity_mode.setObjectName(u"motor_velocity_mode")

        self.horizontalLayout_5.addWidget(self.motor_velocity_mode)

        self.enable_motor = QPushButton(self.tab_6)
        self.enable_motor.setObjectName(u"enable_motor")

        self.horizontalLayout_5.addWidget(self.enable_motor)

        self.disable_motor = QPushButton(self.tab_6)
        self.disable_motor.setObjectName(u"disable_motor")

        self.horizontalLayout_5.addWidget(self.disable_motor)


        self.verticalLayout_3.addLayout(self.horizontalLayout_5)

        self.horizontalLayout_14 = QHBoxLayout()
        self.horizontalLayout_14.setObjectName(u"horizontalLayout_14")
        self.label_2 = QLabel(self.tab_6)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_14.addWidget(self.label_2)

        self.pitch_angle = QDoubleSpinBox(self.tab_6)
        self.pitch_angle.setObjectName(u"pitch_angle")
        self.pitch_angle.setDecimals(4)
        self.pitch_angle.setMinimum(-360.000000000000000)
        self.pitch_angle.setMaximum(360.000000000000000)
        self.pitch_angle.setValue(0.000000000000000)

        self.horizontalLayout_14.addWidget(self.pitch_angle)

        self.actual_pitch_angle = QLineEdit(self.tab_6)
        self.actual_pitch_angle.setObjectName(u"actual_pitch_angle")

        self.horizontalLayout_14.addWidget(self.actual_pitch_angle)


        self.verticalLayout_3.addLayout(self.horizontalLayout_14)

        self.horizontalLayout_15 = QHBoxLayout()
        self.horizontalLayout_15.setObjectName(u"horizontalLayout_15")
        self.label_3 = QLabel(self.tab_6)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_15.addWidget(self.label_3)

        self.yaw_angle = QDoubleSpinBox(self.tab_6)
        self.yaw_angle.setObjectName(u"yaw_angle")
        self.yaw_angle.setDecimals(4)
        self.yaw_angle.setMinimum(-360.000000000000000)
        self.yaw_angle.setMaximum(360.000000000000000)
        self.yaw_angle.setValue(0.000000000000000)

        self.horizontalLayout_15.addWidget(self.yaw_angle)

        self.actual_yaw_angle = QLineEdit(self.tab_6)
        self.actual_yaw_angle.setObjectName(u"actual_yaw_angle")

        self.horizontalLayout_15.addWidget(self.actual_yaw_angle)


        self.verticalLayout_3.addLayout(self.horizontalLayout_15)

        self.start_tracking = QPushButton(self.tab_6)
        self.start_tracking.setObjectName(u"start_tracking")

        self.verticalLayout_3.addWidget(self.start_tracking)

        self.stop_tracking = QPushButton(self.tab_6)
        self.stop_tracking.setObjectName(u"stop_tracking")

        self.verticalLayout_3.addWidget(self.stop_tracking)

        self.read_currunt_position = QPushButton(self.tab_6)
        self.read_currunt_position.setObjectName(u"read_currunt_position")

        self.verticalLayout_3.addWidget(self.read_currunt_position)

        self.delete_last_line = QPushButton(self.tab_6)
        self.delete_last_line.setObjectName(u"delete_last_line")

        self.verticalLayout_3.addWidget(self.delete_last_line)


        self.horizontalLayout_19.addLayout(self.verticalLayout_3)

        self.horizontalLayout_18 = QHBoxLayout()
        self.horizontalLayout_18.setObjectName(u"horizontalLayout_18")
        self.left____1 = QPushButton(self.tab_6)
        self.left____1.setObjectName(u"left____1")

        self.horizontalLayout_18.addWidget(self.left____1)

        self.verticalLayout_6 = QVBoxLayout()
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.up____1 = QPushButton(self.tab_6)
        self.up____1.setObjectName(u"up____1")

        self.verticalLayout_4.addWidget(self.up____1)

        self.up___1 = QPushButton(self.tab_6)
        self.up___1.setObjectName(u"up___1")

        self.verticalLayout_4.addWidget(self.up___1)

        self.up__1 = QPushButton(self.tab_6)
        self.up__1.setObjectName(u"up__1")

        self.verticalLayout_4.addWidget(self.up__1)

        self.up_1 = QPushButton(self.tab_6)
        self.up_1.setObjectName(u"up_1")

        self.verticalLayout_4.addWidget(self.up_1)


        self.verticalLayout_6.addLayout(self.verticalLayout_4)

        self.horizontalLayout_17 = QHBoxLayout()
        self.horizontalLayout_17.setObjectName(u"horizontalLayout_17")
        self.left___1 = QPushButton(self.tab_6)
        self.left___1.setObjectName(u"left___1")

        self.horizontalLayout_17.addWidget(self.left___1)

        self.left__1 = QPushButton(self.tab_6)
        self.left__1.setObjectName(u"left__1")

        self.horizontalLayout_17.addWidget(self.left__1)

        self.left_1 = QPushButton(self.tab_6)
        self.left_1.setObjectName(u"left_1")

        self.horizontalLayout_17.addWidget(self.left_1)

        self.right_1 = QPushButton(self.tab_6)
        self.right_1.setObjectName(u"right_1")

        self.horizontalLayout_17.addWidget(self.right_1)

        self.right__1 = QPushButton(self.tab_6)
        self.right__1.setObjectName(u"right__1")

        self.horizontalLayout_17.addWidget(self.right__1)

        self.right___1 = QPushButton(self.tab_6)
        self.right___1.setObjectName(u"right___1")

        self.horizontalLayout_17.addWidget(self.right___1)


        self.verticalLayout_6.addLayout(self.horizontalLayout_17)

        self.verticalLayout_5 = QVBoxLayout()
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.down_1 = QPushButton(self.tab_6)
        self.down_1.setObjectName(u"down_1")

        self.verticalLayout_5.addWidget(self.down_1)

        self.down__1 = QPushButton(self.tab_6)
        self.down__1.setObjectName(u"down__1")

        self.verticalLayout_5.addWidget(self.down__1)

        self.down___1 = QPushButton(self.tab_6)
        self.down___1.setObjectName(u"down___1")

        self.verticalLayout_5.addWidget(self.down___1)

        self.down____1 = QPushButton(self.tab_6)
        self.down____1.setObjectName(u"down____1")

        self.verticalLayout_5.addWidget(self.down____1)


        self.verticalLayout_6.addLayout(self.verticalLayout_5)


        self.horizontalLayout_18.addLayout(self.verticalLayout_6)

        self.right____1 = QPushButton(self.tab_6)
        self.right____1.setObjectName(u"right____1")

        self.horizontalLayout_18.addWidget(self.right____1)


        self.horizontalLayout_19.addLayout(self.horizontalLayout_18)


        self.horizontalLayout_20.addLayout(self.horizontalLayout_19)

        self.tabWidget_3.addTab(self.tab_6, "")

        self.verticalLayout.addWidget(self.tabWidget_3)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.tabWidget_2 = QTabWidget(self.tab_3)
        self.tabWidget_2.setObjectName(u"tabWidget_2")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        self.horizontalLayout_13 = QHBoxLayout(self.tab_5)
        self.horizontalLayout_13.setObjectName(u"horizontalLayout_13")
        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.CPU1 = QPushButton(self.tab_5)
        self.CPU1.setObjectName(u"CPU1")

        self.horizontalLayout_6.addWidget(self.CPU1)

        self.LON = QPushButton(self.tab_5)
        self.LON.setObjectName(u"LON")

        self.horizontalLayout_6.addWidget(self.LON)

        self.startTracking = QPushButton(self.tab_5)
        self.startTracking.setObjectName(u"startTracking")

        self.horizontalLayout_6.addWidget(self.startTracking)

        self.stopTracking = QPushButton(self.tab_5)
        self.stopTracking.setObjectName(u"stopTracking")

        self.horizontalLayout_6.addWidget(self.stopTracking)

        self.start_stream = QPushButton(self.tab_5)
        self.start_stream.setObjectName(u"start_stream")

        self.horizontalLayout_6.addWidget(self.start_stream)

        self.close_stream = QPushButton(self.tab_5)
        self.close_stream.setObjectName(u"close_stream")

        self.horizontalLayout_6.addWidget(self.close_stream)

        self.LFF = QPushButton(self.tab_5)
        self.LFF.setObjectName(u"LFF")

        self.horizontalLayout_6.addWidget(self.LFF)


        self.verticalLayout_2.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.fsDistancePb = QPushButton(self.tab_5)
        self.fsDistancePb.setObjectName(u"fsDistancePb")

        self.horizontalLayout_12.addWidget(self.fsDistancePb)

        self.fsDistanceCbb = QComboBox(self.tab_5)
        self.fsDistanceCbb.setObjectName(u"fsDistanceCbb")

        self.horizontalLayout_12.addWidget(self.fsDistanceCbb)

        self.SFC = QPushButton(self.tab_5)
        self.SFC.setObjectName(u"SFC")

        self.horizontalLayout_12.addWidget(self.SFC)

        self.SFC_SB = QSpinBox(self.tab_5)
        self.SFC_SB.setObjectName(u"SFC_SB")
        self.SFC_SB.setMinimum(0)
        self.SFC_SB.setMaximum(500)

        self.horizontalLayout_12.addWidget(self.SFC_SB)


        self.verticalLayout_2.addLayout(self.horizontalLayout_12)

        self.horizontalLayout_29 = QHBoxLayout()
        self.horizontalLayout_29.setObjectName(u"horizontalLayout_29")
        self.LFC = QPushButton(self.tab_5)
        self.LFC.setObjectName(u"LFC")

        self.horizontalLayout_29.addWidget(self.LFC)

        self.LFC_SB = QSpinBox(self.tab_5)
        self.LFC_SB.setObjectName(u"LFC_SB")
        self.LFC_SB.setMinimum(0)
        self.LFC_SB.setMaximum(1000)

        self.horizontalLayout_29.addWidget(self.LFC_SB)


        self.verticalLayout_2.addLayout(self.horizontalLayout_29)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.TAR = QPushButton(self.tab_5)
        self.TAR.setObjectName(u"TAR")

        self.horizontalLayout_11.addWidget(self.TAR)

        self.TAR_SB = QSpinBox(self.tab_5)
        self.TAR_SB.setObjectName(u"TAR_SB")
        self.TAR_SB.setMinimum(1)
        self.TAR_SB.setMaximum(3)

        self.horizontalLayout_11.addWidget(self.TAR_SB)


        self.verticalLayout_2.addLayout(self.horizontalLayout_11)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.TH1 = QPushButton(self.tab_5)
        self.TH1.setObjectName(u"TH1")

        self.horizontalLayout_7.addWidget(self.TH1)

        self.TH1_SB = QSpinBox(self.tab_5)
        self.TH1_SB.setObjectName(u"TH1_SB")
        self.TH1_SB.setMinimum(0)
        self.TH1_SB.setMaximum(1000)

        self.horizontalLayout_7.addWidget(self.TH1_SB)


        self.verticalLayout_2.addLayout(self.horizontalLayout_7)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.TH2 = QPushButton(self.tab_5)
        self.TH2.setObjectName(u"TH2")

        self.horizontalLayout_8.addWidget(self.TH2)

        self.TH2_SB = QSpinBox(self.tab_5)
        self.TH2_SB.setObjectName(u"TH2_SB")
        self.TH2_SB.setMinimum(0)
        self.TH2_SB.setMaximum(1000)

        self.horizontalLayout_8.addWidget(self.TH2_SB)


        self.verticalLayout_2.addLayout(self.horizontalLayout_8)

        self.horizontalLayout_26 = QHBoxLayout()
        self.horizontalLayout_26.setObjectName(u"horizontalLayout_26")
        self.horizontalLayout_27 = QHBoxLayout()
        self.horizontalLayout_27.setObjectName(u"horizontalLayout_27")
        self.SIG = QPushButton(self.tab_5)
        self.SIG.setObjectName(u"SIG")

        self.horizontalLayout_27.addWidget(self.SIG)

        self.SIG_SB = QSpinBox(self.tab_5)
        self.SIG_SB.setObjectName(u"SIG_SB")
        self.SIG_SB.setMinimum(0)
        self.SIG_SB.setMaximum(1000)

        self.horizontalLayout_27.addWidget(self.SIG_SB)


        self.horizontalLayout_26.addLayout(self.horizontalLayout_27)


        self.verticalLayout_2.addLayout(self.horizontalLayout_26)

        self.horizontalLayout_30 = QHBoxLayout()
        self.horizontalLayout_30.setObjectName(u"horizontalLayout_30")
        self.AJT = QPushButton(self.tab_5)
        self.AJT.setObjectName(u"AJT")

        self.horizontalLayout_30.addWidget(self.AJT)

        self.TEM = QPushButton(self.tab_5)
        self.TEM.setObjectName(u"TEM")

        self.horizontalLayout_30.addWidget(self.TEM)


        self.verticalLayout_2.addLayout(self.horizontalLayout_30)

        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.fs_start_measurement = QPushButton(self.tab_5)
        self.fs_start_measurement.setObjectName(u"fs_start_measurement")

        self.horizontalLayout_9.addWidget(self.fs_start_measurement)

        self.fs_stop_measurement = QPushButton(self.tab_5)
        self.fs_stop_measurement.setObjectName(u"fs_stop_measurement")

        self.horizontalLayout_9.addWidget(self.fs_stop_measurement)


        self.verticalLayout_2.addLayout(self.horizontalLayout_9)

        self.LPP = QPushButton(self.tab_5)
        self.LPP.setObjectName(u"LPP")

        self.verticalLayout_2.addWidget(self.LPP)

        self.horizontalLayout_28 = QHBoxLayout()
        self.horizontalLayout_28.setObjectName(u"horizontalLayout_28")
        self.label_7 = QLabel(self.tab_5)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_28.addWidget(self.label_7)

        self.rude_distance1 = QLineEdit(self.tab_5)
        self.rude_distance1.setObjectName(u"rude_distance1")

        self.horizontalLayout_28.addWidget(self.rude_distance1)


        self.verticalLayout_2.addLayout(self.horizontalLayout_28)

        self.get_coordinate = QPushButton(self.tab_5)
        self.get_coordinate.setObjectName(u"get_coordinate")

        self.verticalLayout_2.addWidget(self.get_coordinate)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.label = QLabel(self.tab_5)
        self.label.setObjectName(u"label")

        self.horizontalLayout_10.addWidget(self.label)

        self.measurement_result = QLineEdit(self.tab_5)
        self.measurement_result.setObjectName(u"measurement_result")

        self.horizontalLayout_10.addWidget(self.measurement_result)


        self.verticalLayout_2.addLayout(self.horizontalLayout_10)


        self.horizontalLayout_13.addLayout(self.verticalLayout_2)

        self.tabWidget_2.addTab(self.tab_5, "")

        self.horizontalLayout_2.addWidget(self.tabWidget_2)

        self.tabWidget_4 = QTabWidget(self.tab_3)
        self.tabWidget_4.setObjectName(u"tabWidget_4")
        self.tab_7 = QWidget()
        self.tab_7.setObjectName(u"tab_7")
        self.verticalLayout_7 = QVBoxLayout(self.tab_7)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.horizontalLayout_21 = QHBoxLayout()
        self.horizontalLayout_21.setObjectName(u"horizontalLayout_21")
        self.label_8 = QLabel(self.tab_7)
        self.label_8.setObjectName(u"label_8")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy1)
        self.label_8.setStyleSheet(u"font-size: 30pt;  /* \u4f7f\u7528\u70b9\u5355\u4f4d */")
        self.label_8.setTextFormat(Qt.TextFormat.AutoText)
        self.label_8.setWordWrap(False)

        self.horizontalLayout_21.addWidget(self.label_8)

        self.coordinate_result = QLineEdit(self.tab_7)
        self.coordinate_result.setObjectName(u"coordinate_result")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.coordinate_result.sizePolicy().hasHeightForWidth())
        self.coordinate_result.setSizePolicy(sizePolicy2)
        self.coordinate_result.setStyleSheet(u"font-size: 30pt;  /* \u4f7f\u7528\u70b9\u5355\u4f4d */")
        self.coordinate_result.setMaxLength(20000)

        self.horizontalLayout_21.addWidget(self.coordinate_result)

        self.label_9 = QLabel(self.tab_7)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setStyleSheet(u"font-size: 30pt;  /* \u4f7f\u7528\u70b9\u5355\u4f4d */")

        self.horizontalLayout_21.addWidget(self.label_9)


        self.verticalLayout_7.addLayout(self.horizontalLayout_21)

        self.tabWidget_4.addTab(self.tab_7, "")

        self.horizontalLayout_2.addWidget(self.tabWidget_4)


        self.verticalLayout.addLayout(self.horizontalLayout_2)


        self.horizontalLayout_3.addLayout(self.verticalLayout)

        self.tabWidget.addTab(self.tab_3, "")
        self.tab_8 = QWidget()
        self.tab_8.setObjectName(u"tab_8")
        self.tabWidget.addTab(self.tab_8, "")
        self.tab_4 = QWidget()
        self.tab_4.setObjectName(u"tab_4")
        self.horizontalLayout_23 = QHBoxLayout(self.tab_4)
        self.horizontalLayout_23.setObjectName(u"horizontalLayout_23")
        self.horizontalLayout_22 = QHBoxLayout()
        self.horizontalLayout_22.setObjectName(u"horizontalLayout_22")
        self.tabWidget_5 = QTabWidget(self.tab_4)
        self.tabWidget_5.setObjectName(u"tabWidget_5")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.horizontalLayout_25 = QHBoxLayout(self.tab)
        self.horizontalLayout_25.setObjectName(u"horizontalLayout_25")
        self.gridLayout_3 = QGridLayout()
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.start_accurate_measurement = QPushButton(self.tab)
        self.start_accurate_measurement.setObjectName(u"start_accurate_measurement")

        self.gridLayout_3.addWidget(self.start_accurate_measurement, 0, 0, 1, 1)

        self.stop_accurate_measurement = QPushButton(self.tab)
        self.stop_accurate_measurement.setObjectName(u"stop_accurate_measurement")

        self.gridLayout_3.addWidget(self.stop_accurate_measurement, 0, 1, 1, 1)

        self.save_measurement_data = QPushButton(self.tab)
        self.save_measurement_data.setObjectName(u"save_measurement_data")

        self.gridLayout_3.addWidget(self.save_measurement_data, 0, 2, 1, 1)

        self.measurement_table = QTableWidget(self.tab)
        self.measurement_table.setObjectName(u"measurement_table")

        self.gridLayout_3.addWidget(self.measurement_table, 1, 0, 1, 3)

        self.label_5 = QLabel(self.tab)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.label_5, 2, 0, 1, 1)

        self.accurate_measurement = QLineEdit(self.tab)
        self.accurate_measurement.setObjectName(u"accurate_measurement")

        self.gridLayout_3.addWidget(self.accurate_measurement, 2, 1, 1, 2)

        self.label_6 = QLabel(self.tab)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.label_6, 3, 0, 1, 1)

        self.fusion_measurement = QLineEdit(self.tab)
        self.fusion_measurement.setObjectName(u"fusion_measurement")

        self.gridLayout_3.addWidget(self.fusion_measurement, 3, 1, 1, 2)


        self.horizontalLayout_25.addLayout(self.gridLayout_3)

        self.tabWidget_5.addTab(self.tab, "")

        self.horizontalLayout_22.addWidget(self.tabWidget_5)

        self.tabWidget_6 = QTabWidget(self.tab_4)
        self.tabWidget_6.setObjectName(u"tabWidget_6")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.horizontalLayout_24 = QHBoxLayout(self.tab_2)
        self.horizontalLayout_24.setObjectName(u"horizontalLayout_24")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.open_rude_camera = QPushButton(self.tab_2)
        self.open_rude_camera.setObjectName(u"open_rude_camera")

        self.gridLayout_2.addWidget(self.open_rude_camera, 0, 0, 1, 1)

        self.close_rude_camera = QPushButton(self.tab_2)
        self.close_rude_camera.setObjectName(u"close_rude_camera")

        self.gridLayout_2.addWidget(self.close_rude_camera, 0, 1, 1, 1)

        self.detect = QPushButton(self.tab_2)
        self.detect.setObjectName(u"detect")

        self.gridLayout_2.addWidget(self.detect, 0, 2, 1, 1)

        self.widget_2 = QWidget(self.tab_2)
        self.widget_2.setObjectName(u"widget_2")

        self.gridLayout_2.addWidget(self.widget_2, 1, 0, 1, 3)

        self.label_4 = QLabel(self.tab_2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_2.addWidget(self.label_4, 2, 0, 1, 1)

        self.rude_measurement = QLineEdit(self.tab_2)
        self.rude_measurement.setObjectName(u"rude_measurement")

        self.gridLayout_2.addWidget(self.rude_measurement, 2, 1, 1, 2)


        self.horizontalLayout_24.addLayout(self.gridLayout_2)

        self.tabWidget_6.addTab(self.tab_2, "")

        self.horizontalLayout_22.addWidget(self.tabWidget_6)


        self.horizontalLayout_23.addLayout(self.horizontalLayout_22)

        self.tabWidget.addTab(self.tab_4, "")
        self.tab_10 = QWidget()
        self.tab_10.setObjectName(u"tab_10")
        self.horizontalLayout_31 = QHBoxLayout(self.tab_10)
        self.horizontalLayout_31.setObjectName(u"horizontalLayout_31")
        self.wMPS_widget = QWidget(self.tab_10)
        self.wMPS_widget.setObjectName(u"wMPS_widget")

        self.horizontalLayout_31.addWidget(self.wMPS_widget)

        self.tabWidget.addTab(self.tab_10, "")

        self.horizontalLayout.addWidget(self.tabWidget)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1127, 33))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(0)
        self.tabWidget_3.setCurrentIndex(0)
        self.tabWidget_2.setCurrentIndex(0)
        self.tabWidget_4.setCurrentIndex(0)
        self.tabWidget_5.setCurrentIndex(0)
        self.tabWidget_6.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.socket_connection_pushButton.setText(QCoreApplication.translate("MainWindow", u"\u8fde\u63a5\u63a7\u5236\u5668", None))
        self.socket_connection.setText(QCoreApplication.translate("MainWindow", u"192.168.137.130", None))
        self.socket_disconnection_pushButton.setText(QCoreApplication.translate("MainWindow", u"\u65ad\u5f00\u63a7\u5236\u5668", None))
        self.add_motors.setText(QCoreApplication.translate("MainWindow", u"\u589e\u52a0\u7535\u673a", None))
        self.open_xy.setText(QCoreApplication.translate("MainWindow", u"\u6253\u5f00\u6447\u6746", None))
        self.close_xy.setText(QCoreApplication.translate("MainWindow", u"\u5173\u95ed\u6447\u6746", None))
        self.send_motor_message.setText(QCoreApplication.translate("MainWindow", u"\u53d1\u9001", None))
        self.motor_position_mode.setText(QCoreApplication.translate("MainWindow", u"\u4f4d\u7f6e\u6a21\u5f0f", None))
        self.set_zero_position.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u96f6\u70b9", None))
        self.motor_velocity_mode.setText(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u6a21\u5f0f", None))
        self.enable_motor.setText(QCoreApplication.translate("MainWindow", u"\u7535\u673a\u4f7f\u80fd", None))
        self.disable_motor.setText(QCoreApplication.translate("MainWindow", u"\u7535\u673a\u5931\u80fd", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u4fef\u4ef0\u89d2", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u504f\u822a\u89d2", None))
        self.start_tracking.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u542f\u8ddf\u8e2a", None))
        self.stop_tracking.setText(QCoreApplication.translate("MainWindow", u"\u5173\u95ed\u8ddf\u8e2a", None))
        self.read_currunt_position.setText(QCoreApplication.translate("MainWindow", u"\u8bb0\u5f55\u5f53\u524d\u4f4d\u7f6e", None))
        self.delete_last_line.setText(QCoreApplication.translate("MainWindow", u"\u5220\u9664\u4e00\u884c", None))
        self.left____1.setText(QCoreApplication.translate("MainWindow", u"\u21900.0003", None))
        self.up____1.setText(QCoreApplication.translate("MainWindow", u"\u21910.0003", None))
        self.up___1.setText(QCoreApplication.translate("MainWindow", u"\u21910.01", None))
        self.up__1.setText(QCoreApplication.translate("MainWindow", u"\u21910.1", None))
        self.up_1.setText(QCoreApplication.translate("MainWindow", u"\u21911", None))
        self.left___1.setText(QCoreApplication.translate("MainWindow", u"\u21900.01", None))
        self.left__1.setText(QCoreApplication.translate("MainWindow", u"\u21900.1", None))
        self.left_1.setText(QCoreApplication.translate("MainWindow", u"\u21901", None))
        self.right_1.setText(QCoreApplication.translate("MainWindow", u"\u21921", None))
        self.right__1.setText(QCoreApplication.translate("MainWindow", u"\u21920.1", None))
        self.right___1.setText(QCoreApplication.translate("MainWindow", u"\u21920.01", None))
        self.down_1.setText(QCoreApplication.translate("MainWindow", u"\u21931", None))
        self.down__1.setText(QCoreApplication.translate("MainWindow", u"\u21930.1", None))
        self.down___1.setText(QCoreApplication.translate("MainWindow", u"\u21930.01", None))
        self.down____1.setText(QCoreApplication.translate("MainWindow", u"\u21930.0003", None))
        self.right____1.setText(QCoreApplication.translate("MainWindow", u"\u21920.0003", None))
        self.tabWidget_3.setTabText(self.tabWidget_3.indexOf(self.tab_6), QCoreApplication.translate("MainWindow", u"\u7535\u673a\u63a7\u5236", None))
        self.CPU1.setText(QCoreApplication.translate("MainWindow", u"CPU1", None))
        self.LON.setText(QCoreApplication.translate("MainWindow", u"LON", None))
        self.startTracking.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u59cb", None))
        self.stopTracking.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62", None))
        self.start_stream.setText(QCoreApplication.translate("MainWindow", u"\u8c03\u8bd5", None))
        self.close_stream.setText(QCoreApplication.translate("MainWindow", u"\u5173\u95ed\u8c03\u8bd5", None))
        self.LFF.setText(QCoreApplication.translate("MainWindow", u"LFF", None))
        self.fsDistancePb.setText(QCoreApplication.translate("MainWindow", u"\u6d4b\u8ddd\u4eea\u4e32\u53e3\u8fde\u63a5", None))
        self.SFC.setText(QCoreApplication.translate("MainWindow", u"SFC", None))
        self.LFC.setText(QCoreApplication.translate("MainWindow", u"LFC", None))
        self.TAR.setText(QCoreApplication.translate("MainWindow", u"TAR", None))
        self.TH1.setText(QCoreApplication.translate("MainWindow", u"TH1", None))
        self.TH2.setText(QCoreApplication.translate("MainWindow", u"TH2", None))
        self.SIG.setText(QCoreApplication.translate("MainWindow", u"SIG", None))
        self.AJT.setText(QCoreApplication.translate("MainWindow", u"AJT", None))
        self.TEM.setText(QCoreApplication.translate("MainWindow", u"TEM", None))
        self.fs_start_measurement.setText(QCoreApplication.translate("MainWindow", u"\u6d4b\u91cf", None))
        self.fs_stop_measurement.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62\u6d4b\u91cf", None))
        self.LPP.setText(QCoreApplication.translate("MainWindow", u"LPP", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u7c97\u6d4b\u503c", None))
        self.get_coordinate.setText(QCoreApplication.translate("MainWindow", u"\u5750\u6807\u89e3\u7b97", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u878d\u5408\u6d4b\u8ddd\u503c", None))
        self.measurement_result.setText(QCoreApplication.translate("MainWindow", u"0mm", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_5), QCoreApplication.translate("MainWindow", u"\u98de\u79d2\u6d4b\u8ddd", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u5750\u6807\u4e3a", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"mm", None))
        self.tabWidget_4.setTabText(self.tabWidget_4.indexOf(self.tab_7), QCoreApplication.translate("MainWindow", u"\u5f53\u524d\u5750\u6807\u663e\u793a", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"\u57fa\u7840\u529f\u80fd", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_8), QCoreApplication.translate("MainWindow", u"3D\u663e\u793a", None))
        self.start_accurate_measurement.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u542f\u7cbe\u6d4b", None))
        self.stop_accurate_measurement.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62\u7cbe\u6d4b", None))
        self.save_measurement_data.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58\u6570\u636e", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"\u7cbe\u6d4b\u503c", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"\u7c97\u7cbe\u878d\u5408\u503c", None))
        self.tabWidget_5.setTabText(self.tabWidget_5.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"\u98de\u79d2\u7cbe\u6d4b", None))
        self.open_rude_camera.setText(QCoreApplication.translate("MainWindow", u"\u6253\u5f00\u76f8\u673a", None))
        self.close_rude_camera.setText(QCoreApplication.translate("MainWindow", u"\u5173\u95ed\u76f8\u673a", None))
        self.detect.setText(QCoreApplication.translate("MainWindow", u"\u8bc6\u522b", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u7c97\u6d4b\u503c", None))
        self.tabWidget_6.setTabText(self.tabWidget_6.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"\u53cc\u76ee\u76f8\u673a\u7c97\u6d4b", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), QCoreApplication.translate("MainWindow", u"\u6d4b\u91cf", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_10), QCoreApplication.translate("MainWindow", u"wMPS", None))
    # retranslateUi

