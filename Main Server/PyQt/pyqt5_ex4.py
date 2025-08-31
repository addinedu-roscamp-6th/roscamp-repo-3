import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QDesktopWidget, QMainWindow, QLabel, QVBoxLayout
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication, QDate, Qt

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.data = QDate.currentDate()
        self.initUI()


    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())


    def initUI(self):
        ######################### Button ######################### 
        btn = QPushButton("나가기", self)
        btn.move(110, 280)
        btn.resize(btn.sizeHint())
        btn.clicked.connect(QCoreApplication.instance().quit)

        ######################### Date ######################### 
        ######################### QMainWindow일 때 가능 ######################### 
        # self.statusBar().showMessage(self.data.toString(Qt.DefaultLocaleLongDate))

        ######################### Label ######################### 
        lbl_red = QLabel('Red')
        lbl_green = QLabel('Green')
        lbl_blue = QLabel('Blue')
        lbl_red.setStyleSheet("color: red;"
                             "border-style: solid;"
                             "border-width: 2px;"
                             "border-color: #FA8072;"
                             "border-radius: 3px")
        lbl_green.setStyleSheet("color: green;"
                               "background-color: #7FFFD4")
        lbl_blue.setStyleSheet("color: blue;"
                              "background-color: #87CEFA;"
                              "border-style: dashed;"
                              "border-width: 3px;"
                              "border-color: #1E90FF")
        vbox = QVBoxLayout()
        vbox.addWidget(lbl_red)
        vbox.addWidget(lbl_green)
        vbox.addWidget(lbl_blue)
        self.setLayout(vbox)

        ######################### Show ######################### 
        self.setWindowTitle("User GUI")
        self.setWindowIcon(QIcon("/home/jun/kang/src/project/project/mart.png"))
        self.resize(300,350)
        self.center()
        self.show()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())