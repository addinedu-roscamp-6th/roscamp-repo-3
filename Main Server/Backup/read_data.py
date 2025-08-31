import os
import rclpy
from rclpy.node import Node
import pandas as pd

class CsvReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')

        # 현재 이 Python 파일 위치 기준
        base_path = os.path.dirname(os.path.realpath('project'))

        # GUI 서버 폴더 내의 CSV 파일 경로 설정
        self.csv_file = os.path.join(base_path, 'src','project', 'project', 'gui_server', 'received_data.csv')
        self.csv_file = os.path.abspath(self.csv_file)  # 경로 정리

        self.get_logger().info(f'CSV 경로: {self.csv_file}')

        # 주기적으로 읽기
        self.timer = self.create_timer(5.0, self.read_csv_file)

    def read_csv_file(self):
        try:
            df = pd.read_csv(self.csv_file)

            if df.empty:
                self.get_logger().warn("CSV 파일이 비어있음")
                return

            # datetime 형식으로 변환 후 정렬
            df['datetime'] = pd.to_datetime(df['datetime'])
            df = df.sort_values(by='datetime')

            self.get_logger().info(f"총 주문 수: {len(df)}건 (시간순 정렬됨)")

            # 상태 값 해석
            status_map = {
                0: "완료",
                1: "진행중",
                2: "대기"
            }

            for _, row in df.iterrows():
                status_str = status_map.get(row['Status'], "알 수 없음")
                self.get_logger().info(
                    f"[{row['datetime']}] ID: {row['ID']}, Item: {row['Item']}, "
                    f"Count: {row['Count']}, 상태: {status_str}"
                )

        except FileNotFoundError:
            self.get_logger().error(f"CSV 파일을 찾을 수 없습니다: {self.csv_file}")
        except Exception as e:
            self.get_logger().error(f"CSV 읽기 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CsvReaderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
