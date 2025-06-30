import numpy as np
from scipy.spatial.transform import Rotation

# テスト用クォータニオンリスト（様々な回転パターンを網羅）
test_quaternions = [
    # 基本回転（単位・180度回転）
    [0, 0, 0, 1],          # 単位クォータニオン（回転なし）
    [1, 0, 0, 0],          # 180度回転 (X軸)
    [0, 1, 0, 0],          # 180度回転 (Y軸)
    [0, 0, 1, 0],          # 180度回転 (Z軸)
    
    # 90度回転（各軸）
    [0.7071, 0, 0, 0.7071],      # 90度回転 (X軸)
    [0, 0.7071, 0, 0.7071],      # 90度回転 (Y軸)
    [0, 0, 0.7071, 0.7071],      # 90度回転 (Z軸)
    [-0.7071, 0, 0, 0.7071],     # -90度回転 (X軸)
    [0, -0.7071, 0, 0.7071],     # -90度回転 (Y軸)
    [0, 0, -0.7071, 0.7071],     # -90度回転 (Z軸)
    
    # 45度回転（各軸）
    [0.3827, 0, 0, 0.9239],      # 45度回転 (X軸)
    [0, 0.3827, 0, 0.9239],      # 45度回転 (Y軸)
    [0, 0, 0.3827, 0.9239],      # 45度回転 (Z軸)
    [-0.3827, 0, 0, 0.9239],     # -45度回転 (X軸)
    [0, -0.3827, 0, 0.9239],     # -45度回転 (Y軸)
    [0, 0, -0.3827, 0.9239],     # -45度回転 (Z軸)
    
    # 30度回転（各軸）
    [0.2588, 0, 0, 0.9659],      # 30度回転 (X軸)
    [0, 0.2588, 0, 0.9659],      # 30度回転 (Y軸)
    [0, 0, 0.2588, 0.9659],      # 30度回転 (Z軸)
    
    # 複合回転（2軸組み合わせ）
    [-0.7071, 0.7071, 0, 0],     # 180度回転 (XY軸)
    [0.7071, 0, 0.7071, 0],      # 180度回転 (XZ軸)
    [0, 0.7071, 0.7071, 0],      # 180度回転 (YZ軸)
    [0.5, 0.5, 0, 0.7071],       # X90度+Y90度
    [0.5, 0, 0.5, 0.7071],       # X90度+Z90度
    [0, 0.5, 0.5, 0.7071],       # Y90度+Z90度
    
    # 複合回転（3軸組み合わせ）
    [0.5, 0.5, 0.5, 0.5],        # 120度回転 (XYZ等方)
    [0.4619, 0.1913, 0.1913, 0.8536],  # X30度+Y30度+Z30度
    [0.6830, 0.1830, 0.1830, 0.6830],  # X45度+Y45度+Z45度
    [0.3536, 0.3536, 0.3536, 0.7454],  # X45度+Y45度+Z30度
    [0.2706, 0.2706, 0.6533, 0.6533],  # X30度+Y30度+Z90度
    
    # 小角度回転（高精度テスト用）
    [0.0872, 0, 0, 0.9962],      # 10度回転 (X軸)
    [0, 0.0872, 0, 0.9962],      # 10度回転 (Y軸)
    [0, 0, 0.0872, 0.9962],      # 10度回転 (Z軸)
    [0.0436, 0.0436, 0, 0.9981], # X5度+Y5度
    
    # 大角度回転
    [0.9659, 0, 0, 0.2588],      # 150度回転 (X軸)
    [0, 0.9659, 0, 0.2588],      # 150度回転 (Y軸)
    [0, 0, 0.9659, 0.2588],      # 150度回転 (Z軸)
    
    # ランダム的な複合回転
    [0.3015, 0.4520, 0.6030, 0.6030],  # 複雑な組み合わせ1
    [0.1826, 0.3651, 0.5477, 0.7303],  # 複雑な組み合わせ2
    [0.4082, 0.4082, 0.4082, 0.7071],  # 対称的な複合回転
]

def quaternion_equivalence(q1, q2, tolerance=1e-10):
    """
    クォータニオンの等価性をチェック
    qと-qは同じ回転を表すので、両方をチェック
    """
    q1 = np.array(q1)
    q2 = np.array(q2)
    
    # 正の符号での一致をチェック
    diff1 = np.linalg.norm(q1 - q2)
    
    # 負の符号での一致をチェック（クォータニオンの符号の曖昧性）
    diff2 = np.linalg.norm(q1 + q2)
    
    return min(diff1, diff2) < tolerance

def test_quaternion_conversion():
    """
    クォータニオン → Rotation → クォータニオンの変換テスト
    """
    print("クォータニオン変換テスト")
    print("=" * 80)
    print(f"{'No.':<3} {'Original Quaternion':<25} {'Converted Quaternion':<25} {'Match':<8} {'Error':<12}")
    print("-" * 80)
    
    total_tests = len(test_quaternions)
    passed_tests = 0
    tolerance = 1e-10
    
    for i, original_quat in enumerate(test_quaternions):
        try:
            # 元のクォータニオンを正規化
            original_quat = np.array(original_quat)
            original_quat = original_quat / np.linalg.norm(original_quat)
            
            # scipyのRotationは(x, y, z, w)の順序を期待する
            # 入力データが(x, y, z, w)の順序であることを前提とする
            rotation = Rotation.from_quat(original_quat)
            
            # Rotationオブジェクトからクォータニオンに戻す
            converted_quat = rotation.as_quat()
            
            # 等価性をチェック
            is_match = quaternion_equivalence(original_quat, converted_quat, tolerance)
            
            # エラーを計算（最小差分）
            diff1 = np.linalg.norm(original_quat - converted_quat)
            diff2 = np.linalg.norm(original_quat + converted_quat)
            error = min(diff1, diff2)
            
            # 結果を表示
            original_str = f"[{original_quat[0]:.4f}, {original_quat[1]:.4f}, {original_quat[2]:.4f}, {original_quat[3]:.4f}]"
            converted_str = f"[{converted_quat[0]:.4f}, {converted_quat[1]:.4f}, {converted_quat[2]:.4f}, {converted_quat[3]:.4f}]"
            match_str = "OK" if is_match else "NG"
            
            print(f"{i+1:<3} {original_str:<25} {converted_str:<25} {match_str:<8} {error:.2e}")
            
            if is_match:
                passed_tests += 1
                
        except Exception as e:
            print(f"{i+1:<3} {'Error':<25} {'Error':<25} {'ERROR':<8} {str(e)}")
    
    print("-" * 80)
    print(f"テスト結果: {passed_tests}/{total_tests} 通過")
    print(f"成功率: {passed_tests/total_tests*100:.2f}%")
    
    # 詳細な統計情報
    print(f"\n許容誤差: {tolerance:.0e}")
    print("注意: クォータニオンには符号の曖昧性があります（qと-qは同じ回転を表す）")

def test_specific_cases():
    """
    特定のケースの詳細テスト
    """
    print("\n\n詳細テスト（代表的なケース）")
    print("=" * 80)
    
    test_cases = [
        ([0, 0, 0, 1], "単位クォータニオン"),
        ([1, 0, 0, 0], "X軸180度回転"),
        ([0.7071, 0, 0, 0.7071], "X軸90度回転"),
        ([0.5, 0.5, 0.5, 0.5], "XYZ等方120度回転"),
    ]
    
    for quat, description in test_cases:
        print(f"\n{description}:")
        quat = np.array(quat)
        quat = quat / np.linalg.norm(quat)  # 正規化
        
        rotation = Rotation.from_quat(quat)
        converted_quat = rotation.as_quat()
        
        print(f"  元のクォータニオン: {quat}")
        print(f"  変換後クォータニオン: {converted_quat}")
        print(f"  差分: {np.abs(quat - converted_quat)}")
        print(f"  符号反転差分: {np.abs(quat + converted_quat)}")
        
        # 回転行列での確認
        matrix1 = rotation.as_matrix()
        rotation2 = Rotation.from_quat(converted_quat)
        matrix2 = rotation2.as_matrix()
        matrix_diff = np.linalg.norm(matrix1 - matrix2)
        print(f"  回転行列の差分: {matrix_diff:.2e}")

if __name__ == "__main__":
    test_quaternion_conversion()
    test_specific_cases()