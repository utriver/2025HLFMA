#!/usr/bin/env python3
"""
BT-410 블루투스 동글 테스트 스크립트
Raspberry Pi에서 BT-410과 UART 통신을 테스트합니다.
"""

import serial
import time
import sys

# 설정
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 9600  # BT-410 기본 속도
TIMEOUT = 3

def test_bt410_connection():
    """BT-410 동글과의 연결을 테스트합니다."""
    
    print("=== BT-410 블루투스 동글 테스트 ===")
    print(f"포트: {SERIAL_PORT}")
    print(f"속도: {BAUD_RATE} bps")
    print()
    
    try:
        # 시리얼 포트 열기
        ser = serial.Serial(
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
        
        print(f"✓ 시리얼 포트 '{SERIAL_PORT}' 연결 성공")
        
        # 포트 초기화
        ser.flushInput()
        ser.flushOutput()
        time.sleep(0.5)
        
        # 기존 데이터 제거
        if ser.in_waiting > 0:
            old_data = ser.read(ser.in_waiting)
            print(f"기존 데이터 제거: {old_data}")
        
        # BT-410 AT 명령어 테스트
        print("\n1. AT 명령어 응답 테스트...")
        test_at_command(ser, "AT", "OK")
        
        print("\n2. 버전 정보 확인...")
        test_at_command(ser, "AT+VERSION", None)
        
        print("\n3. 이름 확인...")
        test_at_command(ser, "AT+NAME", None)
        
        print("\n4. UUID 확인...")
        test_at_command(ser, "AT+UUID", None)
        
        print("\n5. 역할 확인 (마스터/슬레이브)...")
        test_at_command(ser, "AT+ROLE", None)
        
        print("\n6. 연결 상태 확인...")
        test_at_command(ser, "AT+STATE", None)
        
        return True
        
    except serial.SerialException as e:
        print(f"✗ 시리얼 포트 연결 실패: {e}")
        return False
    except Exception as e:
        print(f"✗ 예상치 못한 오류: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("\n시리얼 포트를 닫았습니다.")

def test_at_command(ser, command, expected_response=None):
    """AT 명령어를 전송하고 응답을 확인합니다."""
    
    try:
        # 명령어 전송
        cmd_bytes = (command + '\r\n').encode('utf-8')
        ser.write(cmd_bytes)
        ser.flush()
        print(f"명령어 전송: {command}")
        
        # 응답 대기
        time.sleep(1)
        
        response = ""
        start_time = time.time()
        
        while time.time() - start_time < TIMEOUT:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                try:
                    response += data.decode('utf-8', errors='ignore')
                except:
                    response += str(data)
                
                # 응답이 완료되었는지 확인
                if '\r' in response or '\n' in response:
                    break
            time.sleep(0.1)
        
        if response:
            response = response.strip()
            print(f"응답: {response}")
            
            if expected_response and expected_response in response:
                print(f"✓ 예상 응답 '{expected_response}' 확인됨")
                return True
            elif expected_response is None:
                print("✓ 응답 수신 완료")
                return True
            else:
                print(f"✗ 예상 응답 '{expected_response}'와 다름")
                return False
        else:
            print("✗ 응답 없음 - BT-410이 연결되지 않았거나 전원이 꺼져있을 수 있습니다")
            return False
            
    except Exception as e:
        print(f"✗ 명령어 실행 중 오류: {e}")
        return False

def print_troubleshooting_guide():
    """문제 해결 가이드를 출력합니다."""
    
    print("\n=== 문제 해결 가이드 ===")
    print()
    print("BT-410 동글이 응답하지 않는 경우:")
    print("1. 물리적 연결 확인:")
    print("   - BT-410의 VCC → 라즈베리파이 3.3V 또는 5V")
    print("   - BT-410의 GND → 라즈베리파이 GND")
    print("   - BT-410의 RXD → 라즈베리파이 GPIO14 (TXD)")
    print("   - BT-410의 TXD → 라즈베리파이 GPIO15 (RXD)")
    print()
    print("2. 전원 확인:")
    print("   - BT-410의 LED가 깜박이는지 확인")
    print("   - 전원 전압이 적절한지 확인 (3.3V 또는 5V)")
    print()
    print("3. 설정 확인:")
    print("   - sudo raspi-config → Interface Options → Serial Port")
    print("   - 시리얼 로그인 셸: 비활성화")
    print("   - 시리얼 포트 하드웨어: 활성화")
    print()
    print("4. 다른 속도 시도:")
    print("   - BT-410은 9600, 38400, 57600, 115200 bps를 지원할 수 있습니다")
    print("   - 이 스크립트를 다른 속도로 실행해보세요")

if __name__ == "__main__":
    print("BT-410 블루투스 동글 연결 테스트를 시작합니다...")
    print("Ctrl+C를 눌러 언제든지 중단할 수 있습니다.")
    print()
    
    try:
        success = test_bt410_connection()
        
        if not success:
            print_troubleshooting_guide()
            
    except KeyboardInterrupt:
        print("\n\n사용자에 의해 테스트가 중단되었습니다.")
    except Exception as e:
        print(f"\n예상치 못한 오류가 발생했습니다: {e}")
    
    print("\n테스트 완료.")
