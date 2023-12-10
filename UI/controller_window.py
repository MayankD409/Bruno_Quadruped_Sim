# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controller_window.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(346, 158)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        Form.setMinimumSize(QtCore.QSize(346, 158))
        Form.setMaximumSize(QtCore.QSize(346, 158))
        self.graphicsView_2 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_2.setGeometry(QtCore.QRect(5, 5, 150, 150))
        self.graphicsView_2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.graphicsView_2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.standup_pushButton = QtWidgets.QPushButton(Form)
        self.standup_pushButton.setGeometry(QtCore.QRect(160, 10, 81, 25))
        self.standup_pushButton.setObjectName("standup_pushButton")
        self.mode_groupBox = QtWidgets.QGroupBox(Form)
        self.mode_groupBox.setGeometry(QtCore.QRect(160, 50, 81, 101))
        self.mode_groupBox.setObjectName("mode_groupBox")
        self.stand_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.stand_radioButton.setGeometry(QtCore.QRect(10, 30, 61, 23))
        self.stand_radioButton.setChecked(True)
        self.stand_radioButton.setObjectName("stand_radioButton")
        self.crawl_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.crawl_radioButton.setGeometry(QtCore.QRect(10, 50, 61, 23))
        self.crawl_radioButton.setObjectName("crawl_radioButton")
        self.trot_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.trot_radioButton.setGeometry(QtCore.QRect(10, 70, 61, 23))
        self.trot_radioButton.setObjectName("trot_radioButton")
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(250, 50, 91, 81))
        self.groupBox.setObjectName("groupBox")
        self.normal_radioButton = QtWidgets.QRadioButton(self.groupBox)
        self.normal_radioButton.setGeometry(QtCore.QRect(10, 30, 71, 23))
        self.normal_radioButton.setChecked(True)
        self.normal_radioButton.setObjectName("normal_radioButton")
        self.crab_radioButton = QtWidgets.QRadioButton(self.groupBox)
        self.crab_radioButton.setGeometry(QtCore.QRect(10, 50, 71, 23))
        self.crab_radioButton.setObjectName("crab_radioButton")
        self.Use_IMU_pushButton = QtWidgets.QPushButton(Form)
        self.Use_IMU_pushButton.setGeometry(QtCore.QRect(250, 10, 89, 25))
        self.Use_IMU_pushButton.setObjectName("Use_IMU_pushButton")

        self.retranslateUi(Form)
        self.standup_pushButton.clicked.connect(Form.stand_up)
        self.stand_radioButton.clicked.connect(Form.stand)
        self.crawl_radioButton.clicked.connect(Form.crawl)
        self.trot_radioButton.clicked.connect(Form.trot)
        self.Use_IMU_pushButton.pressed.connect(Form.IMU_ON)
        self.Use_IMU_pushButton.released.connect(Form.IMU_OFF)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "controller"))
        self.standup_pushButton.setText(_translate("Form", "stand up"))
        self.mode_groupBox.setTitle(_translate("Form", "mode"))
        self.stand_radioButton.setText(_translate("Form", "stand"))
        self.crawl_radioButton.setText(_translate("Form", "crawl"))
        self.trot_radioButton.setText(_translate("Form", "trot"))
        self.groupBox.setTitle(_translate("Form", "gait mode"))
        self.normal_radioButton.setText(_translate("Form", "normal"))
        self.crab_radioButton.setText(_translate("Form", "crab"))
        self.Use_IMU_pushButton.setText(_translate("Form", "IMU"))
