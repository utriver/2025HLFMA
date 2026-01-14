#!/usr/bin/env python3
"""
리모컨 신호 테스트 시뮬레이터
실제 리모컨 없이 신호를 테스트할 수 있습니다.
"""

import time
import serial

# 테스트할 신호들
TEST_SIGNALS = {
    "forward": bytes([0x7F, 0x06, 0xE0]),      # 전진
    "backward": bytes([0x7F, 0x0A, 0x0F, 0xFF]), # 후진
    "neutral": bytes([0x7F, 0x02, 0xE0]),      # 중립/정지
}

def simulate_remote_signals():
    """리모컨 신호를 시뮬레이션합니다."""
    
    print("🎮 리모컨 신호 테스트 시뮬레이터")
    print("=" * 40)
    
    while True:
        print("\n테스트할 신호를 선택하세요:")
        print("1. 전진 (7F 06 E0)")
        print("2. 후진 (7F 0A 0F FF)")
        print("3. 중립/정지 (7F 02 E0)")
        print("4. 연속 테스트")
        print("q. 종료")
        
        choice = input("\n선택: ").strip().lower()
        
        if choice == 'q':
            break
        elif choice == '1':
            test_signal("forward", TEST_SIGNALS["forward"])
        elif choice == '2':
            test_signal("backward", TEST_SIGNALS["backward"])
        elif choice == '3':
            test_signal("neutral", TEST_SIGNALS["neutral"])
        elif choice == '4':
            continuous_test()
        else:
            print("❌ 잘못된 선택입니다.")

def test_signal(name, signal_bytes):
    """단일 신호를 테스트합니다."""
    hex_str = ' '.join([f"{b:02X}" for b in signal_bytes])
    print(f"\n📡 테스트 신호: {name}")
    print(f"   바이트: {hex_str}")
    print(f"   길이: {len(signal_bytes)} 바이트")
    
    # 신호 분석
    if len(signal_bytes) >= 2:
        print(f"   헤더: {signal_bytes[0]:02X}")
        print(f"   명령: {signal_bytes[1]:02X}")
        if len(signal_bytes) >= 3:
            print(f"   데이터: {' '.join([f'{b:02X}' for b in signal_bytes[2:]])}")

def continuous_test():
    """연속 테스트 시나리오"""
    print("\n🚀 연속 테스트 시작...")
    
    test_sequence = [
        ("전진", TEST_SIGNALS["forward"]),
        ("정지", TEST_SIGNALS["neutral"]),
        ("후진", TEST_SIGNALS["backward"]),
        ("정지", TEST_SIGNALS["neutral"]),
        ("전진", TEST_SIGNALS["forward"]),
        ("정지", TEST_SIGNALS["neutral"]),
    ]
    
    for i, (name, signal) in enumerate(test_sequence, 1):
        hex_str = ' '.join([f"{b:02X}" for b in signal])
        print(f"\n[{i}/{len(test_sequence)}] {name}: {hex_str}")
        time.sleep(1)
    
    print("\n✅ 연속 테스트 완료")

def analyze_signal_pattern():
    """신호 패턴을 분석합니다."""
    print("\n🔍 신호 패턴 분석")
    print("=" * 30)
    
    for name, signal in TEST_SIGNALS.items():
        hex_str = ' '.join([f"{b:02X}" for b in signal])
        print(f"\n{name:8}: {hex_str}")
        print(f"         길이: {len(signal)} 바이트")
        print(f"         헤더: {signal[0]:02X} ({'OK' if signal[0] == 0x7F else 'ERROR'})")
        print(f"         명령: {signal[1]:02X}")
        if len(signal) > 2:
            print(f"         추가: {' '.join([f'{b:02X}' for b in signal[2:]])}")

if __name__ == "__main__":
    try:
        print("리모컨 신호 분석:")
        analyze_signal_pattern()
        print("\n" + "=" * 50)
        simulate_remote_signals()
    except KeyboardInterrupt:
        print("\n👋 테스트 종료")
    except Exception as e:
        print(f"❌ 오류: {e}")

