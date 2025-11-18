# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controller_window.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!



from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *

class Ui_Form(object):
    def setupUi(self, Form:QDialog):
        Form.setObjectName("Form")
        Form.resize(110, 220)
        sizePolicy = QSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)



        self.layout = QGridLayout()
        Form.setLayout(self.layout)

        self.standup_pushButton = QPushButton(Form)
        self.standup_pushButton.setGeometry(QRect(10, 10, 81, 25))
        self.standup_pushButton.setObjectName("standup_pushButton")
        self.laydown_pushButton = QPushButton(Form)
        self.laydown_pushButton.setGeometry(QRect(10, 50, 81, 25))
        self.laydown_pushButton.setObjectName("laydown_pushButton")
        self.walk_pushButton = QPushButton(Form)
        self.walk_pushButton.setGeometry(QRect(10, 90, 81, 25))
        self.walk_pushButton.setObjectName("walk_pushButton")
        self.turn_pushButton = QPushButton(Form)
        self.turn_pushButton.setGeometry(QRect(10, 130, 81, 25))
        self.turn_pushButton.setObjectName("turn_pushButton")
        self.quit_pushButton = QPushButton(Form)
        self.quit_pushButton.setGeometry(QRect(10, 170, 81, 25))
        self.quit_pushButton.setObjectName("quit_pushButton")

        self.layout.addWidget(self.standup_pushButton)
        self.layout.addWidget(self.laydown_pushButton)
        self.layout.addWidget(self.walk_pushButton)
        self.layout.addWidget(self.turn_pushButton)
        self.layout.addWidget(self.quit_pushButton)


        self.retranslateUi(Form)
        self.standup_pushButton.clicked.connect(Form.stand_up)
        self.laydown_pushButton.clicked.connect(Form.lay_down)
        self.walk_pushButton.clicked.connect(Form.walk)
        self.turn_pushButton.clicked.connect(Form.turn)
        self.quit_pushButton.clicked.connect(Form.quit)

        QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "controller"))
        self.standup_pushButton.setText(_translate("Form", "stand up"))
        self.laydown_pushButton.setText(_translate("Form", "lay down"))
        self.walk_pushButton.setText(_translate("Form", "walk"))
        self.turn_pushButton.setText(_translate("Form", "turn"))
        self.quit_pushButton.setText(_translate("Form", "quit"))


