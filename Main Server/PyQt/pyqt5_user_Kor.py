from PyQt5.QtWidgets import (
    QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout,
    QApplication, QMessageBox
)
import sys
import socket
import json

class TCPGuiSender(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("유저 정보 전송")
        self.setGeometry(100, 100, 300, 200)

        self.id_input = QLineEdit()
        self.item_input = QLineEdit()
        self.count_input = QLineEdit()
        self.status_label = QLabel("서버에 연결되지 않음")

        send_button = QPushButton("주문 완료")
        send_button.clicked.connect(self.send_data)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("ID"))
        layout.addWidget(self.id_input)
        layout.addWidget(QLabel("진열 물품"))
        layout.addWidget(self.item_input)
        layout.addWidget(QLabel("개수"))
        layout.addWidget(self.count_input)
        layout.addWidget(send_button)
        layout.addWidget(self.status_label)

        self.setLayout(layout)

        # 서버에 연결
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect(("192.168.1.20", 9999)) 
            self.status_label.setText("서버 연결 성공")
        except Exception as e:
            self.status_label.setText(f"연결 실패: {e}")
            self.client_socket = None

    def send_data(self):
        if not self.client_socket:
            self.status_label.setText("서버에 연결되지 않았습니다.")
            return

        try:
            user_id = int(self.id_input.text())
            item = self.item_input.text()
            count = int(self.count_input.text())

            # 사용자 입력을 확인 메시지에 넣기
            confirm_msg = f"정말로 다음 내용을 주문하시겠습니까?\n\nID: {user_id}\n물품: {item}\n개수: {count}"
            reply = QMessageBox.question(self, '주문 확인', confirm_msg,
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)

            if reply == QMessageBox.Yes:
                payload = {
                    "id": user_id,
                    "product": item,
                    "quantity": count
                }
                message = json.dumps(payload)
                self.client_socket.sendall(message.encode('utf-8'))
                self.status_label.setText("주문이 전송되었습니다.")
            else:
                self.status_label.setText("주문이 취소되었습니다.")

        except Exception as e:
            self.status_label.setText(f"전송 실패: {e}")


    def closeEvent(self, event):
        if self.client_socket:
            self.client_socket.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TCPGuiSender()
    window.show()
    sys.exit(app.exec_())
