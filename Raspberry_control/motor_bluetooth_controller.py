#!/usr/bin/env python3
"""
블루투스 리모컨과 모터 제어기 연동
"""

import asyncio
import serial
import sys
import time
from motor_control.motorcontrol import MD10CController

# 블루투스 설정
SERIAL_PORT = '/dev/serial0'
BAUD_RATE = 38400
TIMEOUT = 0.1

class BluetoothMotorController:
    def __init__(self):
        self.motor_controller = None
        self.bluetooth_serial = None
        self.running = False
        self.last_command = None
        self.last_command_time = 0
        self.current_motor_state = 0.0  # 현재 모터 상태 추적
        
    async def initialize(self):
        """초기화"""
        try:
            # 모터 컨트롤러 초기화
            self.motor_controller = MD10CController()
            print("✅ 모터 컨트롤러 초기화 완료")
            
            # 블루투스 시리얼 연결
            self.bluetooth_serial = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=TIMEOUT
            )
            self.bluetooth_serial.flushInput()
            self.bluetooth_serial.flushOutput()
            print("✅ 블루투스 연결 완료")
            
            return True
            
        except Exception as e:
            print(f"❌ 초기화 실패: {e}")
            return False
    
    def parse_bluetooth_command(self, data):
        """블루투스 데이터를 모터 명령으로 변환"""
        try:
            if len(data) < 3:
                return None
            
            # 신호 패턴 확인
            if data[0] != 0x7F:  # 시작 바이트 검증
                print(f"⚠️ 잘못된 시작 바이트: {data[0]:02X} (예상: 7F)")
                return None
            
            # 명령 바이트 분석 (두 번째 바이트)
            command_byte = data[1]
            
            if command_byte == 0x06:
                # 전진 신호: 7F 06 E0
                if len(data) >= 3 and data[2] == 0xE0:
                    current_time = time.time()
                    
                    # 연속된 같은 신호인지 확인 (정지 판별)
                    if (self.last_command == "forward" and 
                        current_time - self.last_command_time < 0.2 and
                        self.current_motor_state > 0):
                        # 연속된 전진 신호 = 정지 명령으로 해석
                        print("⏹️ 정지 신호 감지 (연속 전진)")
                        self.last_command = "stop"
                        self.last_command_time = current_time
                        return 0.0
                    else:
                        # 새로운 전진 명령
                        print("🟢 전진 신호 감지")
                        self.last_command = "forward"
                        self.last_command_time = current_time
                        return 50.0  # 50% 전진 속도
                else:
                    print(f"⚠️ 전진 신호 불완전: {' '.join([f'{b:02X}' for b in data])}")
                    return None
                    
            elif command_byte == 0x0A:
                # 후진: 7F 0A 0F FF
                if len(data) >= 4 and data[2] == 0x0F and data[3] == 0xFF:
                    print("🔴 후진 신호 감지")
                    self.last_command = "backward"
                    self.last_command_time = time.time()
                    return -50.0  # 50% 후진 속도
                else:
                    print(f"⚠️ 후진 신호 불완전: {' '.join([f'{b:02X}' for b in data])}")
                    return None
                    
            elif command_byte == 0x02:
                # 중립/정지: 7F 02 E0 (아무것도 누르지 않을 때)
                if len(data) >= 3 and data[2] == 0xE0:
                    print("⏸️ 중립 신호 감지 (버튼 해제)")
                    self.last_command = "neutral"
                    self.last_command_time = time.time()
                    return 0.0  # 정지
                else:
                    print(f"⚠️ 중립 신호 불완전: {' '.join([f'{b:02X}' for b in data])}")
                    return None
            
            else:
                print(f"❓ 알 수 없는 명령 바이트: {command_byte:02X}")
                return None
            
        except Exception as e:
            print(f"❌ 명령 파싱 오류: {e}")
            return None
    
    async def listen_bluetooth(self):
        """블루투스 입력 수신 루프"""
        print("🎮 블루투스 리모컨 입력 대기 중...")
        print("리모컨 버튼을 눌러보세요!")
        
        while self.running:
            try:
                if self.bluetooth_serial.in_waiting > 0:
                    data = self.bluetooth_serial.read(self.bluetooth_serial.in_waiting)
                    
                    # 데이터 출력
                    hex_data = ' '.join([f"{b:02X}" for b in data])
                    print(f"📱 수신: {hex_data}")
                    
                    # 명령 파싱
                    motor_command = self.parse_bluetooth_command(data)
                    
                    if motor_command is not None:
                        print(f"🎯 모터 명령: {motor_command:+.1f}%")
                        # 모터 제어
                        success = self.motor_controller.set_motor_duty(motor_command)
                        if success:
                            self.current_motor_state = motor_command
                    else:
                        print("❓ 알 수 없는 명령")
                
                await asyncio.sleep(0.01)  # 10ms 대기
                
            except Exception as e:
                print(f"❌ 블루투스 수신 오류: {e}")
                await asyncio.sleep(0.1)
    
    async def run(self):
        """메인 실행 루프"""
        if not await self.initialize():
            return
        
        self.running = True
        
        try:
            await self.listen_bluetooth()
        except KeyboardInterrupt:
            print("\n🛑 사용자에 의해 중단됨")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """정리"""
        self.running = False
        
        if self.motor_controller:
            self.motor_controller.cleanup()
            print("🧹 모터 컨트롤러 정리 완료")
        
        if self.bluetooth_serial and self.bluetooth_serial.is_open:
            self.bluetooth_serial.close()
            print("🧹 블루투스 연결 해제 완료")

async def main():
    print("=" * 60)
    print("🎮 블루투스 리모컨 모터 제어기")
    print("=" * 60)
    
    controller = BluetoothMotorController()
    await controller.run()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 프로그램 종료")
    except Exception as e:
        print(f"❌ 시스템 오류: {e}")
        sys.exit(1)
