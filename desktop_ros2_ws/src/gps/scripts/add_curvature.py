import csv
import numpy as np
import argparse
import os

# --- [새로운 설정] ---
# 곡률 계산 시 사용할 점의 간격(Offset)입니다.
# 이 값을 키울수록 더 완만한 곡선을 잘 감지하지만, 너무 크면 날카로운 코너를 놓칠 수 있습니다.
# 5 ~ 15 사이의 값으로 시작하여 테스트해보는 것을 추천합니다.
CURVATURE_CALC_OFFSET = 10

def calculate_curvature(p1, p2, p3):
    """
    연속된 세 점(p1, p2, p3)을 이용하여 p2에서의 곡률을 계산합니다.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    # 세 점이 거의 일직선 상에 있는 경우, 곡률은 0
    if abs((y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1)) < 1e-9:
        return 0.0

    area = 0.5 * abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    side1 = np.hypot(x1 - x2, y1 - y2)
    side2 = np.hypot(x2 - x3, y2 - y3)
    side3 = np.hypot(x3 - x1, y3 - y1)

    if side1 * side2 * side3 == 0:
        return 0.0

    radius = (side1 * side2 * side3) / (4 * area)
    
    if radius > 1000:
        return 0.0

    return 1.0 / radius

def main():
    parser = argparse.ArgumentParser(description="CSV 파일에 곡률(curvature) 열을 추가합니다.")
    parser.add_argument('--input', required=True, help="입력 CSV 파일 경로")
    parser.add_argument('--output', required=True, help="출력 CSV 파일 경로")
    args = parser.parse_args()

    points = []
    try:
        with open(args.input, 'r', newline='') as infile:
            reader = csv.reader(infile)
            for row in reader:
                if len(row) >= 2:
                    points.append((float(row[0]), float(row[1])))
    except FileNotFoundError:
        print(f"[에러] 파일을 찾을 수 없습니다: {args.input}")
        return
        
    if len(points) < (2 * CURVATURE_CALC_OFFSET + 1):
        print(f"[에러] 곡률을 계산하기에 포인트 수가 너무 적습니다. (최소 {2 * CURVATURE_CALC_OFFSET + 1}개 필요)")
        return

    curvatures = [0.0] * len(points) # 모든 곡률을 0으로 초기화

    # --- [수정된 로직] ---
    # 간격을 둔 점들을 사용하여 곡률 계산
    for i in range(CURVATURE_CALC_OFFSET, len(points) - CURVATURE_CALC_OFFSET):
        p1 = points[i - CURVATURE_CALC_OFFSET]
        p2 = points[i]
        p3 = points[i + CURVATURE_CALC_OFFSET]
        curvatures[i] = calculate_curvature(p1, p2, p3)

    with open(args.output, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(['x', 'y', 'curvature'])
        for i, point in enumerate(points):
            writer.writerow([point[0], point[1], curvatures[i]])

    print(f"✅ 성공: '{args.output}' 파일에 {len(points)}개의 포인트와 곡률이 저장되었습니다.")

if __name__ == '__main__':
    main()
