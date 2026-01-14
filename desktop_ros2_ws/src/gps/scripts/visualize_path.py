import matplotlib.pyplot as plt
import csv
import os

# --- [중요] ---
# 시각화할 CSV 파일의 경로와 이름을 정확하게 입력해주세요.
# 사용자께서는 'final_edited_path.csv' 파일을 생성하셨습니다.
CSV_PATH = '~/ros2_ws/src/gps/paths/yongf1.csv'

def visualize():
    """CSV 경로 파일을 읽어 Matplotlib으로 시각화합니다."""
    
    # '~' 경로를 실제 전체 경로로 변환
    full_path = os.path.expanduser(CSV_PATH)

    if not os.path.exists(full_path):
        print(f"[에러] 파일을 찾을 수 없습니다: {full_path}")
        print("CSV_PATH 변수의 파일 경로와 이름이 올바른지 확인해주세요.")
        return

    # 데이터 읽기
    x, y, curvature = [], [], []
    with open(full_path, 'r') as f:
        reader = csv.reader(f)
        try:
            next(reader) # 헤더 건너뛰기
            for row in reader:
                x.append(float(row[0]))
                y.append(float(row[1]))
                curvature.append(float(row[2]))
        except (StopIteration, IndexError, ValueError) as e:
            print(f"[에러] CSV 파일을 읽는 중 문제가 발생했습니다: {e}")
            print("파일이 비어있거나 형식이 올바르지 않은지 확인해주세요.")
            return

    print("데이터를 성공적으로 읽었습니다. 그래프를 생성합니다...")

    # 시각화
    plt.figure(figsize=(10, 10))
    # 곡률 값을 색깔(c)과 점의 크기(s)로 표현
    scatter = plt.scatter(x, y, c=curvature, cmap='viridis', s=15, vmin=0, vmax=max(curvature))
    
    plt.colorbar(scatter, label='Curvature (1/m)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Path with Curvature Visualization')
    plt.axis('equal') # X, Y 축의 비율을 동일하게 설정
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    visualize()
