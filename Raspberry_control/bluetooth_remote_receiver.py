#!/usr/bin/env python3
"""
블루투스 리모컨 입력 수신기
BT-410을 통해 블루투스 리모컨의 버튼 입력을 실시간으로 받아서 출력합니다.
"""

import serial
import time
import sys
from datetime import datetime

# 설정
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 38400  # 테스트에서 응답이 있었던 속도 사용
TIMEOUT = 0.1  # 빠른 응답을 위해 짧은 타임아웃

class BluetoothRemoteReceiver:
    def __init__(self):
        self.ser = None
        self.running = False
        
    def connect(self):
        """시리얼 포트에 연결합니다."""
        try:
            self.ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            
            print(f"✓ 블루투스 수신기 연결 성공 ({SERIAL_PORT}, {BAUD_RATE} bps)")
            
            # 포트 초기화
            self.ser.flushInput()
            self.ser.flushOutput()
            time.sleep(0.5)
            
            # 기존 데이터 제거
            if self.ser.in_waiting > 0:
                old_data = self.ser.read(self.ser.in_waiting)
                print(f"기존 데이터 제거: {len(old_data)} 바이트")
            
            return True
            
        except serial.SerialException as e:
            print(f"✗ 시리얼 포트 연결 실패: {e}")
            return False
        except Exception as e:
            print(f"✗ 예상치 못한 오류: {e}")
            return False
    
    def disconnect(self):
        """시리얼 포트 연결을 해제합니다."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("시리얼 포트 연결을 해제했습니다.")
    
    def format_data(self, data):
        """수신된 데이터를 여러 형식으로 포맷합니다."""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # 원시 바이트 표시
        hex_str = ' '.join([f"{b:02X}" for b in data])
        
        # ASCII 문자 표시 (출력 가능한 문자만)
        ascii_str = ''.join([chr(b) if 32 <= b <= 126 else f'[{b:02X}]' for b in data])
        
        # 정수 배열로 표시
        int_str = ', '.join([str(b) for b in data])
        
        return {
            'timestamp': timestamp,
            'hex': hex_str,
            'ascii': ascii_str,
            'integers': int_str,
            'length': len(data)
        }
    
    def listen_for_input(self):
        """블루투스 리모컨 입력을 지속적으로 수신합니다."""
        if not self.ser:
            print("✗ 시리얼 포트가 연결되지 않았습니다.")
            return
        
        print("\n=== 블루투스 리모컨 입력 수신 시작 ===")
        print("리모컨 버튼을 눌러보세요. Ctrl+C로 종료할 수 있습니다.")
        print("=" * 60)
        print()
        
        self.running = True
        input_count = 0
        
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    # 데이터 수신
                    data = self.ser.read(self.ser.in_waiting)
                    input_count += 1
                    
                    # 데이터 포맷팅
                    formatted = self.format_data(data)
                    
                    # 출력
                    print(f"[{input_count:03d}] {formatted['timestamp']}")
                    print(f"    길이: {formatted['length']} 바이트")
                    print(f"    HEX:  {formatted['hex']}")
                    print(f"    ASCII: {formatted['ascii']}")
                    print(f"    정수:  {formatted['integers']}")
                    print()
                
                # CPU 사용률을 줄이기 위한 짧은 대기
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n사용자에 의해 중단되었습니다.")
            self.running = False
    
    def test_connection(self):
        """연결 상태를 테스트합니다."""
        if not self.ser:
            return False
        
        print("연결 테스트 중...")
        
        # 몇 초간 데이터가 오는지 확인
        start_time = time.time()
        data_received = False
        
        while time.time() - start_time < 3:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                if data:
                    data_received = True
                    print(f"테스트 데이터 수신: {len(data)} 바이트")
                    break
            time.sleep(0.1)
        
        if not data_received:
            print("3초간 데이터가 수신되지 않았습니다.")
            print("리모컨 버튼을 눌러보거나 BT-410 연결 상태를 확인하세요.")
        
        return data_received

def main():
    print("블루투스 리모컨 입력 수신기")
    print("=" * 40)
    
    receiver = BluetoothRemoteReceiver()
    
    try:
        # 연결
        if not receiver.connect():
            print("연결에 실패했습니다. 프로그램을 종료합니다.")
            return
        
        # 연결 테스트
        print("\n연결 테스트를 진행합니다...")
        if receiver.test_connection():
            print("✓ 데이터 수신이 확인되었습니다.")
        else:
            print("⚠ 데이터가 수신되지 않지만 수신 대기를 계속합니다.")
        
        # 입력 수신 시작
        receiver.listen_for_input()
        
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        receiver.disconnect()

if __name__ == "__main__":
    main()
