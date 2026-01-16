import threading
import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LoadCellPublisher(Node):
    def __init__(self):
        super().__init__('load_cell_publisher')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)

        # launch에서 전달된 파라미터 읽기
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        self.pub = self.create_publisher(Float32, 'loadcell', 10)

        # timeout 짧게: readline 블로킹 최소화
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
            self.get_logger().info(f'Opened serial: {port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial {port} @ {baud}: {e}')
            rclpy.shutdown()
            raise SystemExit(1)

        self._stop = False
        self._th = threading.Thread(target=self._reader_loop, daemon=True)
        self._th.start()

    def _reader_loop(self):
        while rclpy.ok() and not self._stop:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                val = float(line)  # Arduino가 한 줄에 숫자만 보내는 가정
                msg = Float32()
                msg.data = val
                self.pub.publish(msg)
            except ValueError:
                # 숫자 아닌 줄(부팅 메시지 등) 무시
                continue
            except (serial.SerialException, OSError) as e:
                # 일시적 I/O 오류는 경고 후 재시도
                self.get_logger().warn(f'Serial I/O error, retrying: {e}')
                time.sleep(0.005)
                continue
            except Exception as e:
                self.get_logger().error(f'Unexpected reader error, stopping: {e}')
                break

    def destroy_node(self):
        self._stop = True
        try:
            self.ser.close()
        except Exception:
            pass
        try:
            # 종료 시 스레드가 남지 않도록 합류 시도
            self._th.join(timeout=1.0)
            if self._th.is_alive():
                self.get_logger().warn('Reader thread did not stop within timeout')
        except Exception as e:
            self.get_logger().warn(f'Failed to join reader thread: {e}')
        super().destroy_node()


def main():
    rclpy.init()
    node = LoadCellPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()