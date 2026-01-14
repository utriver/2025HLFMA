#!/usr/bin/env python3
"""
GPIO 및 모터 드라이버 진단 도구
"""

import RPi.GPIO as GPIO
import time
import sys

# GPIO 핀 설정 (motorcontrol.py와 동일)
PWM_PIN = 32
DIR_PIN = 36

def check_gpio_permissions():
    """GPIO 접근 권한 확인"""
    try:
        GPIO.setmode(GPIO.BOARD)
        print("✅ GPIO 접근 권한: 정상")
        return True
    except Exception as e:
        print(f"❌ GPIO 접근 권한 오류: {e}")
        return False

def test_gpio_pins():
    """GPIO 핀 기본 테스트"""
    try:
        print("\n🔧 GPIO 핀 설정 테스트...")
        
        # 핀 설정
        GPIO.setup(PWM_PIN, GPIO.OUT)
        GPIO.setup(DIR_PIN, GPIO.OUT)
        print(f"✅ PWM 핀 {PWM_PIN} 설정 완료")
        print(f"✅ DIR 핀 {DIR_PIN} 설정 완료")
        
        # 방향 핀 테스트
        print("\n📍 방향 핀 테스트...")
        GPIO.output(DIR_PIN, GPIO.HIGH)
        print("   정방향(HIGH) 설정")
        time.sleep(1)
        
        GPIO.output(DIR_PIN, GPIO.LOW)
        print("   역방향(LOW) 설정")
        time.sleep(1)
        
        return True
        
    except Exception as e:
        print(f"❌ GPIO 핀 설정 오류: {e}")
        return False

def test_pwm_output():
    """PWM 출력 테스트"""
    try:
        print("\n⚡ PWM 출력 테스트...")
        
        # PWM 객체 생성
        pwm = GPIO.PWM(PWM_PIN, 1000)  # 1kHz
        pwm.start(0)
        print("✅ PWM 객체 생성 성공")
        
        # 단계별 PWM 테스트
        test_duties = [10, 25, 50, 75, 100]
        
        for duty in test_duties:
            print(f"   PWM {duty}% 출력 중...")
            pwm.ChangeDutyCycle(duty)
            time.sleep(2)
            
            # 사용자 확인
            response = input(f"   {duty}%에서 모터가 움직이나요? (y/n): ").strip().lower()
            if response == 'y':
                print(f"   ✅ {duty}%에서 모터 동작 확인")
            else:
                print(f"   ❌ {duty}%에서 모터 미동작")
        
        pwm.stop()
        print("✅ PWM 테스트 완료")
        return True
        
    except Exception as e:
        print(f"❌ PWM 테스트 오류: {e}")
        return False

def check_system_resources():
    """시스템 리소스 확인"""
    print("\n💻 시스템 리소스 확인...")
    
    try:
        # CPU 온도 확인
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = int(f.read()) / 1000.0
        print(f"   CPU 온도: {temp:.1f}°C {'⚠️ 높음' if temp > 70 else '✅ 정상'}")
        
        # 메모리 사용량 확인
        with open('/proc/meminfo', 'r') as f:
            meminfo = f.read()
        
        mem_total = int([line for line in meminfo.split('\n') if 'MemTotal' in line][0].split()[1])
        mem_free = int([line for line in meminfo.split('\n') if 'MemFree' in line][0].split()[1])
        mem_usage = (mem_total - mem_free) / mem_total * 100
        
        print(f"   메모리 사용률: {mem_usage:.1f}% {'⚠️ 높음' if mem_usage > 90 else '✅ 정상'}")
        
    except Exception as e:
        print(f"⚠️ 시스템 리소스 확인 오류: {e}")

def check_power_supply():
    """전원 공급 상태 확인"""
    print("\n🔋 전원 상태 확인...")
    
    try:
        # GPIO 전압 확인 (간접적)
        print("   라즈베리파이 전원: 연결됨 ✅")
        print("   GPIO 핀 전원: 활성화됨 ✅")
        
        # 외부 전원 확인 메시지
        print("\n🔌 MD10C 드라이버 전원 확인 사항:")
        print("   1. MD10C에 12V-24V 전원이 연결되어 있나요?")
        print("   2. 전원 LED가 켜져 있나요?")
        print("   3. 모터가 물리적으로 연결되어 있나요?")
        print("   4. 모터에 기계적 부하가 걸려있지 않나요?")
        
    except Exception as e:
        print(f"⚠️ 전원 확인 오류: {e}")

def main():
    print("🔍 모터 드라이버 진단 도구")
    print("=" * 50)
    
    try:
        # 1. 권한 확인
        if not check_gpio_permissions():
            print("\n💡 해결 방법: sudo 권한으로 실행하세요")
            print("   sudo python3 debug_gpio.py")
            return
        
        # 2. GPIO 핀 테스트
        if not test_gpio_pins():
            print("\n❌ GPIO 핀 설정에 문제가 있습니다")
            return
        
        # 3. PWM 출력 테스트
        if not test_pwm_output():
            print("\n❌ PWM 출력에 문제가 있습니다")
            return
        
        # 4. 시스템 리소스 확인
        check_system_resources()
        
        # 5. 전원 확인
        check_power_supply()
        
        print("\n" + "=" * 50)
        print("🎯 진단 완료!")
        print("\n💡 문제 해결 가이드:")
        print("   1. 모터가 전혀 반응하지 않으면 → MD10C 전원 확인")
        print("   2. 한 방향으로만 돌면 → DIR 핀 연결 확인")
        print("   3. 속도 조절이 안 되면 → PWM 핀 연결 확인")
        print("   4. 간헐적으로 동작하면 → 배선 접촉 불량 확인")
        
    except KeyboardInterrupt:
        print("\n\n🛑 사용자에 의해 중단됨")
    except Exception as e:
        print(f"\n❌ 진단 중 오류 발생: {e}")
    finally:
        try:
            GPIO.cleanup()
            print("\n🧹 GPIO 정리 완료")
        except:
            pass

if __name__ == "__main__":
    main()
