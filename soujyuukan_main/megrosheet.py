import numpy as np
import matplotlib.pyplot as plt
import os
import sys

def fit_polynomial(x, y, degree):
    """
    与えられた (x, y) のデータセットから n次多項式で近似し、その係数を計算します。
    """
    # np.polyfitを使用して最小二乗法で多項式近似の係数を取得
    # 係数は降べきの順 (a*x^n + b*x^(n-1) + ... + c)
    coefficients = np.polyfit(x, y, degree)
    
    # 係数から多項式オブジェクトを作成 (評価に便利)
    poly_func = np.poly1d(coefficients)
    
    return coefficients, poly_func


def polynomial_to_pow_expr(coefficients, var_name="x"):
    """polyfit 係数（高次から）を coef*pow(var, n)+... 形式の1行に整形（Arduino 等へコピペ用）。"""
    ncoef = len(coefficients)
    degree = ncoef - 1
    parts = []
    for i, c in enumerate(coefficients):
        p = degree - i
        cs = f"{c:.12g}"
        if p == 0:
            parts.append(cs)
        else:
            parts.append(f"{cs}*pow({var_name}, {p})")
    expr = parts[0]
    for s in parts[1:]:
        if s.startswith("-"):
            expr += " - " + s[1:].lstrip()
        else:
            expr += " + " + s
    return expr


def main():
    # 1. CSV ファイルの指定（第1引数で指定、省略時は input.csv）
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'input.csv'

    if not os.path.exists(csv_file):
        print(f"エラー: {csv_file} が見つかりません。")
        print(f"使い方: python {os.path.basename(sys.argv[0])} [csv_file]")
        return
    
    x_data = []
    y_data = []
    
    try:
        with open(csv_file, 'r', encoding='utf-8') as f:
            # 1行目を読み込んでヘッダーを取得
            header_line = f.readline().strip()
            # カンマまたはタブ・スペースで区切られている場合に対応
            header_line = header_line.replace(',', ' ')
            headers = header_line.split()
            
            # xとyの列が存在するか確認し、インデックスを取得
            if 'x' in headers and 'y' in headers:
                x_idx = headers.index('x')
                y_idx = headers.index('y')
                print(f"ヘッダーから 'x' (列インデックス: {x_idx}) と 'y' (列インデックス: {y_idx}) の列を正常に発見しました。")
            else:
                print("エラー: ヘッダーに 'x' または 'y' が見つかりません。")
                return
            
            # データ行の読み込み
            for line_num, line in enumerate(f, start=2):
                line = line.replace(',', ' ')
                parts = line.split()
                if not parts:
                    continue # 空行はスキップ
                
                # 指定したインデックス以上の要素がない行はスキップ
                if len(parts) <= max(x_idx, y_idx):
                    continue
                
                try:
                    x_val = float(parts[x_idx])
                    y_val = float(parts[y_idx])
                    x_data.append(x_val)
                    y_data.append(y_val)
                except ValueError:
                    print(f"警告: {line_num}行目のデータを数値に変換できませんでした。スキップします。")
                    
        x_data = np.array(x_data)
        y_data = np.array(y_data)
        print(f"{csv_file} から {len(x_data)} 件の有効なデータを読み込みました。")
        
    except Exception as e:
        print(f"データの読み込み中にエラーが発生しました: {e}")
        return
    
    # 2. 多項式近似の実行（x=舵角 → y=サーボ角）
    degree = 5
    
    print(f"--- {degree}次多項式でデータを近似 ---")
    fit_input = x_data   # 多項式の入力（横軸）
    fit_output = y_data  # 多項式の出力（縦軸）
    input_label = 'x'
    output_label = 'y'

    coefficients, poly_func = fit_polynomial(fit_input, fit_output, degree)

    print("近似された係数 (高次から順):")
    for i, coef in enumerate(coefficients):
        print(f"  {input_label}^{degree - i} の係数: {coef:.4f}")

    print(f"\n生成された多項式 ({input_label} → {output_label}):")
    print(poly_func)
    print("\nコピペ用 (pow 形式):")
    print(polynomial_to_pow_expr(coefficients, input_label))

    # 3. 結果の可視化 (Matplotlib)
    # 元データを散布図として描画
    plt.scatter(fit_input, fit_output, label=f'Original Data ({os.path.basename(csv_file)})', color='blue', alpha=0.5, s=10)

    # 近似曲線をプロットするための滑らかな座標の配列を作成
    input_fit = np.linspace(min(fit_input), max(fit_input), 500)
    output_fit = poly_func(input_fit)

    # 近似曲線を描画
    plt.plot(input_fit, output_fit, label=f'Fitted Polynomial (degree={degree})', color='red', linewidth=2)

    plt.xlabel(input_label)
    plt.ylabel(output_label)
    plt.title(f'Polynomial Regression (n={degree}, {input_label} → {output_label})')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
