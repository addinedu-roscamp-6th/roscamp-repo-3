from PyQt5.QtWidgets import (
    QWidget, QLabel, QSpinBox, QComboBox, QPushButton,
    QVBoxLayout, QHBoxLayout, QApplication, QMessageBox
)
import sys
import socket
import json

class TCPGuiSender(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("유저 정보 전송")
        self.setGeometry(100, 100, 300, 200)

        # ID, 물품, 개수 입력 위젯
        self.id_input = QSpinBox()
        self.id_input.setRange(0, 9999)

        self.item_input = QComboBox()
        self.item_input.addItems(["콜라", "사이다", "오렌지주스", "커피", "물"])  # 원하는 물품들 추가

        self.count_input = QSpinBox()
        self.count_input.setRange(1, 100)

        self.status_label = QLabel("서버에 연결되지 않음")

        send_button = QPushButton("저장")
        send_button.clicked.connect(self.send_data)

        cancel_button = QPushButton("취소")
        cancel_button.clicked.connect(self.close)  # 창 닫기

        # 버튼들을 가로로 배치
        button_layout = QHBoxLayout()
        button_layout.addWidget(send_button)
        button_layout.addWidget(cancel_button)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("ID"))
        layout.addWidget(self.id_input)
        layout.addWidget(QLabel("진열 물품"))
        layout.addWidget(self.item_input)
        layout.addWidget(QLabel("개수"))
        layout.addWidget(self.count_input)
        layout.addLayout(button_layout)  # 저장/취소 버튼 추가
        layout.addWidget(self.status_label)

        self.setLayout(layout)

        # 서버에 연결
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.client_socket.connect(("192.168.1.20", 9999))  # <-- 서버 IP 수정 가능
            self.status_label.setText("서버 연결 성공")
        except Exception as e:
            self.status_label.setText(f"연결 실패: {e}")
            self.client_socket = None

    def send_data(self):
        if not self.client_socket:
            self.status_label.setText("서버에 연결되지 않았습니다.")
            return

        user_id = self.id_input.value()
        item = self.item_input.currentText()
        count = self.count_input.value()

        confirm_msg = f"다음 내용을 주문하시겠습니까?\n\nID: {user_id}\n물품: {item}\n개수: {count}"

        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("주문 확인")
        msg_box.setText(confirm_msg)
        order_button = msg_box.addButton("주문", QMessageBox.AcceptRole)
        modify_button = msg_box.addButton("수정", QMessageBox.RejectRole)

        msg_box.exec_()

        clicked_button = msg_box.clickedButton()
        if clicked_button == order_button:
            try:
                payload = {
                    "id": user_id,
                    "product": item,
                    "quantity": count
                }
                message = json.dumps(payload)
                self.client_socket.sendall(message.encode('utf-8'))
                self.status_label.setText("주문이 전송되었습니다.")
            except Exception as e:
                self.status_label.setText(f"전송 실패: {e}")

        elif clicked_button == modify_button:
            self.status_label.setText("주문이 수정되었습니다.")


    def closeEvent(self, event):
        if self.client_socket:
            self.client_socket.close()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = TCPGuiSender()
    window.show()
    sys.exit(app.exec_())
