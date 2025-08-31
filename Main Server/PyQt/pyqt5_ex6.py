import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QDesktopWidget, QMainWindow
from PyQt5.QtWidgets import QLabel, QVBoxLayout, QGridLayout, QLabel, QLineEdit, QTextEdit
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication, QDate, Qt
class MyApp(QWidget):

    def __init__(self):
        super().__init__()
        self.initUI()

    def center(self):
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def initUI(self):
        grid = QGridLayout()
        self.setLayout(grid)

        grid.addWidget(QLabel('ID   :'), 0, 0)
        grid.addWidget(QLabel('Item :'), 1, 0)
        grid.addWidget(QLabel('count:'), 2, 0)

        grid.addWidget(QLineEdit(), 0, 1)
        grid.addWidget(QLineEdit(), 1, 1)
        grid.addWidget(QTextEdit(), 2, 1)

        self.setWindowTitle("User GUI")
        self.resize(300,350)
        self.center()
        self.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())