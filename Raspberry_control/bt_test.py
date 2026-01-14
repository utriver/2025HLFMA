# -*- coding: utf-8 -*-
import serial
import time

# BT-410은 라즈베리파이의 '/dev/serial0'에 연결됩니다.
# 기본 통신 속도(Baud rate)는 57600입니다.
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 57600

# 로보티즈 리모컨 통신 패킷의 고유한 시작 바이트
START_BYTE_1 = 0xFF
START_BYTE_2 = 0x55

def get_rc100_data(ser):
    """
    시리얼 포트에서 RC-100 프로토콜에 맞는 유효한 패킷을 찾아 버튼 데이터를 반환합니다.
    패킷 형식: 0xFF, 0x55, ~DATA, DATA
    """
    while True:
        # 첫 번째 시작 바이트(0xFF)를 찾습니다.
        if ser.read(1) == bytes([START_BYTE_1]):
            # 두 번째 시작 바이트(0x55)를 찾습니다.
            if ser.read(1) == bytes([START_BYTE_2]):
                # 다음 2바이트(~DATA, DATA)를 읽어옵니다.
                data_packet = ser.read(2)
                
                # 패킷 길이가 2바이트인지 확인합니다.
                if len(data_packet) == 2:
                    not_data, data = data_packet
                    
                    # 데이터 유효성 검사: DATA == ~NOT_DATA
                    # 8비트 연산을 위해 & 0xFF를 해줍니다.
                    if data == (~not_data & 0xFF):
                        # 유효한 데이터이므로 값을 반환하고 함수를 종료합니다.
                        return data
        # 유효한 패킷을 찾지 못하면 None을 반환하지 않고 계속 루프를 돕니다.

try:
    # 시리얼 포트 설정
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"시리얼 포트 '{SERIAL_PORT}'가 열렸습니다. (Baud rate: {BAUD_RATE})")
    print("리모컨 신호 수신 대기 중... (종료하려면 Ctrl+C)")
    print("--- 유효한 RC-100 데이터 패킷만 출력합니다. ---")

    while True:
        button_data = get_rc100_data(ser)
        
        if button_data is not None:
            # 수신된 버튼 데이터를 16진수와 2진수 형태로 출력
            print(f"버튼 데이터 수신: {hex(button_data)} | 2진수: {bin(button_data)}")
        
        # CPU 부하를 줄이기 위해 짧은 대기 시간 추가
        time.sleep(0.01)

except serial.SerialException as e:
    print(f"오류: 시리얼 포트를 열 수 없습니다. - {e}")
    print("1. BT-410 동글이 올바르게 연결되었는지 확인하세요.")
    print("2. 'sudo raspi-config' 또는 '/boot/config.txt'에서 시리얼 포트가 활성화되었는지 확인하세요.")
    print("3. 'sudo usermod -a -G dialout $USER' 명령어로 사용자 권한을 추가했는지 확인하세요.")

except KeyboardInterrupt:
    print("\n프로그램을 종료합니다.")

finally:
    # 프로그램 종료 시 시리얼 포트를 닫음
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print(f"시리얼 포트 '{SERIAL_PORT}'가 닫혔습니다.")
