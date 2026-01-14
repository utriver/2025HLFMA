#!/usr/bin/env python3
from dynamixel_sdk import *
import time

class ServoControl:
    """Dynamixel XM540 제어 클래스"""
    
    def __init__(self):
        # Dynamixel 설정
        self.DEVICENAME = '/dev/ttyUSB0'  # 포트 이름 (환경에 맞게 수정)
        self.BAUDRATE = 3000000   
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = 1  # Dynamixel ID (환경에 맞게 수정)
        
        # Control table address (XM540용)
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116  # Goal Position 주소
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0
        
        # 위치 값 (0도 = 2048)
        self.CENTER_POSITION = 2048  # 0도 위치
        
        # PortHandler와 PacketHandler 초기화
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        # 포트 열기
        if self.portHandler.openPort():
            print(f"포트 {self.DEVICENAME} 열기 성공")
        else:
            print("포트 열기 실패")
            return
        
        # 통신 속도 설정
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print(f"통신 속도 {self.BAUDRATE} 설정 성공")
        else:
            print("통신 속도 설정 실패")
            return
        
        # 토크 활성화
        self.enable_torque()
        
        # 목표 위치를 0도로 설정
        self.set_goal_position(self.CENTER_POSITION)
    
    def enable_torque(self):
        """토크 활성화"""
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, 
            self.DXL_ID, 
            self.ADDR_TORQUE_ENABLE, 
            self.TORQUE_ENABLE
        )
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"토크 활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"토크 활성화 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Dynamixel ID {self.DXL_ID} 토크 활성화 완료")
    
    def disable_torque(self):
        """토크 비활성화"""
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, 
            self.DXL_ID, 
            self.ADDR_TORQUE_ENABLE, 
            self.TORQUE_DISABLE
        )
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"토크 비활성화 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"토크 비활성화 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Dynamixel ID {self.DXL_ID} 토크 비활성화 완료")
    
    def set_goal_position(self, goal_position):
        """목표 위치 설정"""
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, 
            self.DXL_ID, 
            self.ADDR_GOAL_POSITION, 
            goal_position
        )
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"위치 설정 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"위치 설정 에러: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            print(f"Dynamixel ID {self.DXL_ID} 목표 위치 {goal_position} 설정 완료")
    
    def cleanup(self):
        """정리 작업"""
        self.disable_torque()
        self.portHandler.closePort()
        print("포트 닫기 및 정리 완료")


def main():
    servo = ServoControl()
    
    print("Servo Control 시작됨 - 목표 위치 0도로 설정됨")
    print("종료하려면 Ctrl+C를 누르세요.")
    
    try:
        # 무한 루프 (토크 유지)
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n종료 신호 감지됨")
    finally:
        # 정리 작업
        servo.cleanup()


if __name__ == '__main__':
    main()
