#!/usr/bin/env python3
"""
블루투스 리모컨 입력 수신기 (115200 bps 버전)
"""

import serial
import time
import sys
from datetime import datetime

# 설정 - 115200 bps로 설정 (더 빠른 응답을 위해)
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 115200
TIMEOUT = 0.05

def main():
    print(f"블루투스 리모컨 입력 수신기 ({BAUD_RATE} bps)")
    print("=" * 50)
    
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT
        )
        
        print(f"✓ 연결 성공: {SERIAL_PORT} @ {BAUD_RATE} bps")
        print("리모컨 버튼을 눌러보세요. Ctrl+C로 종료.")
        print("=" * 50)
        
        ser.flushInput()
        ser.flushOutput()
        
        button_count = 0
        
        while True:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                button_count += 1
                
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                hex_data = ' '.join([f"{b:02X}" for b in data])
                
                print(f"[{button_count:03d}] {timestamp} | {len(data)} 바이트 | {hex_data}")
                
                # 의미있는 패턴이 있다면 추가 분석
                if len(data) > 0:
                    # 첫 번째 바이트가 특정 값인지 확인
                    if data[0] in range(32, 127):  # 출력 가능한 ASCII
                        ascii_char = chr(data[0])
                        print(f"     → ASCII: '{ascii_char}'")
                    
                    # 반복되는 패턴 확인
                    if len(set(data)) == 1:
                        print(f"     → 반복 패턴: {data[0]:02X} x {len(data)}")
            
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\n\n프로그램이 중단되었습니다.")
    except Exception as e:
        print(f"오류: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("시리얼 포트를 닫았습니다.")

if __name__ == "__main__":
    main()



