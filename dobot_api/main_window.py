# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(791, 476)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.setupGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.setupGroupBox.setGeometry(QtCore.QRect(10, 100, 261, 81))
        self.setupGroupBox.setObjectName("setupGroupBox")
        self.label_3 = QtWidgets.QLabel(self.setupGroupBox)
        self.label_3.setGeometry(QtCore.QRect(10, 20, 61, 16))
        self.label_3.setObjectName("label_3")
        self.velocity_line = QtWidgets.QLineEdit(self.setupGroupBox)
        self.velocity_line.setGeometry(QtCore.QRect(90, 20, 71, 20))
        self.velocity_line.setObjectName("velocity_line")
        self.acceleration_line = QtWidgets.QLineEdit(self.setupGroupBox)
        self.acceleration_line.setGeometry(QtCore.QRect(90, 50, 71, 20))
        self.acceleration_line.setObjectName("acceleration_line")
        self.label_4 = QtWidgets.QLabel(self.setupGroupBox)
        self.label_4.setGeometry(QtCore.QRect(10, 50, 61, 16))
        self.label_4.setObjectName("label_4")
        self.home_button = QtWidgets.QPushButton(self.setupGroupBox)
        self.home_button.setGeometry(QtCore.QRect(180, 30, 61, 23))
        self.home_button.setObjectName("home_button")
        self.connectionGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.connectionGroupBox.setGeometry(QtCore.QRect(10, 10, 261, 80))
        self.connectionGroupBox.setObjectName("connectionGroupBox")
        self.port_line = QtWidgets.QLineEdit(self.connectionGroupBox)
        self.port_line.setGeometry(QtCore.QRect(80, 20, 71, 20))
        self.port_line.setObjectName("port_line")
        self.label = QtWidgets.QLabel(self.connectionGroupBox)
        self.label.setGeometry(QtCore.QRect(10, 20, 47, 13))
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(self.connectionGroupBox)
        self.label_2.setGeometry(QtCore.QRect(10, 50, 61, 16))
        self.label_2.setObjectName("label_2")
        self.baudrate_line = QtWidgets.QLineEdit(self.connectionGroupBox)
        self.baudrate_line.setGeometry(QtCore.QRect(80, 50, 71, 20))
        self.baudrate_line.setObjectName("baudrate_line")
        self.connect_button = QtWidgets.QPushButton(self.connectionGroupBox)
        self.connect_button.setGeometry(QtCore.QRect(180, 30, 61, 23))
        self.connect_button.setObjectName("connect_button")
        self.trajectoryGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.trajectoryGroupBox.setGeometry(QtCore.QRect(10, 190, 261, 161))
        self.trajectoryGroupBox.setObjectName("trajectoryGroupBox")
        self.shapeComboBox = QtWidgets.QComboBox(self.trajectoryGroupBox)
        self.shapeComboBox.setGeometry(QtCore.QRect(90, 20, 69, 22))
        self.shapeComboBox.setObjectName("shapeComboBox")
        self.shapeComboBox.addItem("")
        self.shapeComboBox.addItem("")
        self.shapeComboBox.addItem("")
        self.label_5 = QtWidgets.QLabel(self.trajectoryGroupBox)
        self.label_5.setGeometry(QtCore.QRect(10, 20, 61, 16))
        self.label_5.setObjectName("label_5")
        self.label_6 = QtWidgets.QLabel(self.trajectoryGroupBox)
        self.label_6.setGeometry(QtCore.QRect(10, 50, 61, 16))
        self.label_6.setObjectName("label_6")
        self.zstep_line = QtWidgets.QLineEdit(self.trajectoryGroupBox)
        self.zstep_line.setGeometry(QtCore.QRect(90, 50, 71, 20))
        self.zstep_line.setObjectName("zstep_line")
        self.xystep_line = QtWidgets.QLineEdit(self.trajectoryGroupBox)
        self.xystep_line.setGeometry(QtCore.QRect(90, 80, 71, 20))
        self.xystep_line.setObjectName("xystep_line")
        self.label_7 = QtWidgets.QLabel(self.trajectoryGroupBox)
        self.label_7.setGeometry(QtCore.QRect(10, 80, 71, 16))
        self.label_7.setObjectName("label_7")
        self.plan_button = QtWidgets.QPushButton(self.trajectoryGroupBox)
        self.plan_button.setGeometry(QtCore.QRect(180, 20, 61, 23))
        self.plan_button.setObjectName("plan_button")
        self.show_button = QtWidgets.QPushButton(self.trajectoryGroupBox)
        self.show_button.setGeometry(QtCore.QRect(180, 50, 61, 23))
        self.show_button.setObjectName("show_button")
        self.execute_button = QtWidgets.QPushButton(self.trajectoryGroupBox)
        self.execute_button.setGeometry(QtCore.QRect(180, 80, 61, 23))
        self.execute_button.setObjectName("execute_button")
        self.label_8 = QtWidgets.QLabel(self.trajectoryGroupBox)
        self.label_8.setGeometry(QtCore.QRect(10, 110, 81, 16))
        self.label_8.setObjectName("label_8")
        self.diameter_line = QtWidgets.QLineEdit(self.trajectoryGroupBox)
        self.diameter_line.setGeometry(QtCore.QRect(90, 110, 71, 20))
        self.diameter_line.setObjectName("diameter_line")
        self.graphicsView = QtWidgets.QGraphicsView(self.centralwidget)
        self.graphicsView.setGeometry(QtCore.QRect(310, 10, 431, 421))
        self.graphicsView.setObjectName("graphicsView")
        self.estop_button = QtWidgets.QPushButton(self.centralwidget)
        self.estop_button.setGeometry(QtCore.QRect(170, 370, 101, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.estop_button.setFont(font)
        self.estop_button.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.estop_button.setAutoFillBackground(False)
        self.estop_button.setStyleSheet("background-color:red;\n"
"color:white;\n"
"")
        self.estop_button.setObjectName("estop_button")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 791, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.setupGroupBox.setTitle(_translate("MainWindow", "2. Setup"))
        self.label_3.setText(_translate("MainWindow", "Velocity:"))
        self.velocity_line.setText(_translate("MainWindow", "200"))
        self.acceleration_line.setText(_translate("MainWindow", "200"))
        self.label_4.setText(_translate("MainWindow", "Acceleration:"))
        self.home_button.setText(_translate("MainWindow", "HOME"))
        self.connectionGroupBox.setTitle(_translate("MainWindow", "1. Connection"))
        self.port_line.setText(_translate("MainWindow", "COM3"))
        self.label.setText(_translate("MainWindow", "Port:"))
        self.label_2.setText(_translate("MainWindow", "Baudrate:"))
        self.baudrate_line.setText(_translate("MainWindow", "115200"))
        self.connect_button.setText(_translate("MainWindow", "CONNECT"))
        self.trajectoryGroupBox.setTitle(_translate("MainWindow", "3. Trajectory"))
        self.shapeComboBox.setItemText(0, _translate("MainWindow", "Spiral"))
        self.shapeComboBox.setItemText(1, _translate("MainWindow", "Triangle"))
        self.shapeComboBox.setItemText(2, _translate("MainWindow", "Square"))
        self.label_5.setText(_translate("MainWindow", "Shape:"))
        self.label_6.setText(_translate("MainWindow", "Z step [mm]:"))
        self.zstep_line.setText(_translate("MainWindow", "0.03"))
        self.xystep_line.setText(_translate("MainWindow", "0.03"))
        self.label_7.setText(_translate("MainWindow", "XY step [mm]:"))
        self.plan_button.setText(_translate("MainWindow", "PLAN"))
        self.show_button.setText(_translate("MainWindow", "SHOW"))
        self.execute_button.setText(_translate("MainWindow", "EXECUTE"))
        self.label_8.setText(_translate("MainWindow", "Diameter [mm]:"))
        self.diameter_line.setText(_translate("MainWindow", "60"))
        self.estop_button.setText(_translate("MainWindow", "E-STOP"))
