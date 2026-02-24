#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # Main window setup
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1056, 901)
        
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # Main grid layout
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(10, 80, 1041, 691))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setVerticalSpacing(6)
        self.gridLayout_2.setObjectName("gridLayout_2")
        
        # Left panel - Navigation controls
        self._setup_left_panel()
        
        # Right panel - Map and controls
        self._setup_right_panel()
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        # Menu bar and status bar
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1056, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def _setup_left_panel(self):
        """Setup the left panel with navigation buttons"""
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout.setSpacing(3)
        self.verticalLayout.setObjectName("verticalLayout")
        
        # Title label
        self.labelTitle = QtWidgets.QLabel(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.labelTitle.setFont(font)
        self.labelTitle.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.labelTitle.setAutoFillBackground(True)
        self.labelTitle.setObjectName("labelTitle")
        self.verticalLayout.addWidget(self.labelTitle, 0, QtCore.Qt.AlignHCenter|QtCore.Qt.AlignVCenter)
        
        # Navigation buttons with meaningful names
        button_configs = [
            ('buttonMenu', 'Menu'),
            ('buttonCreateMap', 'Create Map'),
            ('buttonLoadMap', 'Load Map'),
            ('buttonSetMap', 'Set Map'),
            ('buttonCreateWaypoint', 'Create Waypoint'),
            ('buttonSetWaypoint', 'Set Waypoint'),
            ('buttonLoadWaypoint', 'Load Waypoint'),
            ('buttonWaypointList', 'Waypoint List'),
            ('buttonSelectPoint', 'Select Point To Go'),
            ('buttonPointList', 'Point List'),
        ]
        
        for attr_name, text in button_configs:
            button = self._create_nav_button(attr_name, text)
            self.verticalLayout.addWidget(button)
        
        self.gridLayout_2.addLayout(self.verticalLayout, 0, 0, 1, 1)

    def _setup_right_panel(self):
        """Setup the right panel with map display and controls"""
        self.gridLayout_4 = QtWidgets.QGridLayout()
        self.gridLayout_4.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.gridLayout_4.setObjectName("gridLayout_4")
        
        # Map label (title)
        self.labelMapTitle = QtWidgets.QLabel(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelMapTitle.sizePolicy().hasHeightForWidth())
        self.labelMapTitle.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.labelMapTitle.setFont(font)
        self.labelMapTitle.setObjectName("labelMapTitle")
        self.gridLayout_4.addWidget(self.labelMapTitle, 0, 0, 1, 1, QtCore.Qt.AlignHCenter)
        
        # Map display area
        self.verticalLayout_6 = QtWidgets.QVBoxLayout()
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        
        self.labelMapDisplay = QtWidgets.QLabel(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelMapDisplay.sizePolicy().hasHeightForWidth())
        self.labelMapDisplay.setSizePolicy(sizePolicy)
        self.labelMapDisplay.setStyleSheet(
            "background-color: transparent;\n"
            "color: black;\n"
            "border: 1px solid black;\n"
            "border-radius: 4px;\n"
            "padding: 6px 12px;"
        )
        self.labelMapDisplay.setObjectName("labelMapDisplay")
        self.verticalLayout_6.addWidget(self.labelMapDisplay)
        
        self.gridLayout_4.addLayout(self.verticalLayout_6, 1, 0, 1, 1)
        
        # Control panel (manual/auto + keyboard + emergency)
        self._setup_control_panel()
        
        self.gridLayout_2.addLayout(self.gridLayout_4, 0, 1, 1, 1)

    def _setup_control_panel(self):
        """Setup the control panel with keyboard controls and mode selection"""
        self.gridLayout_7 = QtWidgets.QGridLayout()
        self.gridLayout_7.setObjectName("gridLayout_7")
        
        # Mode selection radio buttons
        self.radioButtonManual = QtWidgets.QRadioButton(self.gridLayoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(True)
        font.setWeight(75)
        self.radioButtonManual.setFont(font)
        self.radioButtonManual.setObjectName("radioButtonManual")
        self.gridLayout_7.addWidget(self.radioButtonManual, 0, 0, 1, 1)
        
        self.radioButtonAuto = QtWidgets.QRadioButton(self.gridLayoutWidget_2)
        self.radioButtonAuto.setFont(font)
        self.radioButtonAuto.setObjectName("radioButtonAuto")
        self.gridLayout_7.addWidget(self.radioButtonAuto, 0, 1, 1, 1)
        
        # Keyboard controls (WASD layout)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        
        # W (forward)
        self.buttonKeyW = self._create_keyboard_button("buttonKeyW", "W")
        self.gridLayout.addWidget(self.buttonKeyW, 0, 1, 1, 1)
        
        # A (left)
        self.buttonKeyA = self._create_keyboard_button("buttonKeyA", "A")
        self.gridLayout.addWidget(self.buttonKeyA, 1, 0, 1, 1)
        
        # S (backward)
        self.buttonKeyS = self._create_keyboard_button("buttonKeyS", "S")
        self.gridLayout.addWidget(self.buttonKeyS, 1, 1, 1, 1)
        
        # D (right)
        self.buttonKeyD = self._create_keyboard_button("buttonKeyD", "D")
        self.gridLayout.addWidget(self.buttonKeyD, 1, 2, 1, 1)
        
        self.gridLayout_7.addLayout(self.gridLayout, 1, 0, 1, 1)
        
        # Emergency stop button
        self.buttonEmergencyStop = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.buttonEmergencyStop.sizePolicy().hasHeightForWidth())
        self.buttonEmergencyStop.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(16)
        self.buttonEmergencyStop.setFont(font)
        self.buttonEmergencyStop.setStyleSheet(
            "QPushButton {\n"
            "    background-color: #2d2d2d;\n"
            "    color: #ff3b3b;\n"
            "    border: 2px solid #ff3b3b;\n"
            "    border-radius: 6px;\n"
            "}\n"
            "\n"
            "QPushButton:hover {\n"
            "    background-color: #ff3b3b;\n"
            "    color: #2d2d2d;\n"
            "}\n"
            "\n"
            "QPushButton:pressed {\n"
            "    background-color: #cc2f2f;\n"
            "}\n"
        )
        self.buttonEmergencyStop.setObjectName("buttonEmergencyStop")
        self.gridLayout_7.addWidget(self.buttonEmergencyStop, 1, 1, 1, 1)
        
        self.gridLayout_4.addLayout(self.gridLayout_7, 2, 0, 1, 1)

    def _create_nav_button(self, object_name, text):
        """Helper method to create navigation buttons with consistent styling"""
        button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
        button.setSizePolicy(sizePolicy)
        
        font = QtGui.QFont()
        font.setPointSize(16)
        button.setFont(font)
        
        button.setStyleSheet(
            "QPushButton {\n"
            "    background-color: #2d2d2d;\n"
            "    color: #ffffff;\n"
            "    border: 2px solid #ffffff;\n"
            "    border-radius: 6px;\n"
            "}\n"
            "\n"
            "QPushButton:hover {\n"
            "    background-color: #ffffff;\n"
            "    color: #2d2d2d;\n"
            "}\n"
            "\n"
            "QPushButton:pressed {\n"
            "    background-color: #cccccc;\n"
            "}\n"
        )
        
        button.setObjectName(object_name)
        setattr(self, object_name, button)
        return button

    def _create_keyboard_button(self, object_name, text):
        """Helper method to create keyboard control buttons"""
        button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(button.sizePolicy().hasHeightForWidth())
        button.setSizePolicy(sizePolicy)
        
        button.setStyleSheet(
            "QPushButton {\n"
            "    background-color: transparent;\n"
            "    color: black;\n"
            "    border: 1px solid black;\n"
            "    border-radius: 4px;\n"
            "    padding: 6px 12px;\n"
            "}\n"
            "\n"
            "QPushButton:hover {\n"
            "    background-color: #f2f2f2;\n"
            "}\n"
            "\n"
            "QPushButton:pressed {\n"
            "    background-color: #d6d6d6;\n"
            "}\n"
            "\n"
            "QPushButton:disabled {\n"
            "    color: #888;\n"
            "    border: 1px solid #888;\n"
            "}\n"
        )
        
        button.setObjectName(object_name)
        button.setText(text)
        return button

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Robot Navigation System"))
        
        # Left panel
        self.labelTitle.setText(_translate("MainWindow", "Robot Navigasi"))
        self.buttonMenu.setText(_translate("MainWindow", "Menu"))
        self.buttonCreateMap.setText(_translate("MainWindow", "Create Map"))
        self.buttonLoadMap.setText(_translate("MainWindow", "Load Map"))
        self.buttonSetMap.setText(_translate("MainWindow", "Set Map"))
        self.buttonCreateWaypoint.setText(_translate("MainWindow", "Create Waypoint"))
        self.buttonSetWaypoint.setText(_translate("MainWindow", "Set Waypoint"))
        self.buttonLoadWaypoint.setText(_translate("MainWindow", "Load Waypoint"))
        self.buttonWaypointList.setText(_translate("MainWindow", "Waypoint List"))
        self.buttonSelectPoint.setText(_translate("MainWindow", "Select Point To Go"))
        self.buttonPointList.setText(_translate("MainWindow", "Point List"))
        
        # Right panel
        self.labelMapTitle.setText(_translate("MainWindow", "Map"))
        self.labelMapDisplay.setText(_translate("MainWindow", "Map"))
        self.radioButtonManual.setText(_translate("MainWindow", "Manual"))
        self.radioButtonAuto.setText(_translate("MainWindow", "Auto"))
        self.buttonEmergencyStop.setText(_translate("MainWindow", "EMERGENCY BUTTON"))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())