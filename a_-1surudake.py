import csv
import os

def detect_delimiter(filepath):
    with open(filepath, 'r', newline='') as f:
        line = f.readline()
        return '\t' if '\t' in line else ','

def main():
    # ===== 実行ディレクトリ基準 =====
    current_dir = os.getcwd()              # ICM_pathplanning
    base_dir = os.path.dirname(current_dir)  # plan
    path_dir = os.path.join(base_dir, "ICM_Log", "path")

    # ===== 入力（拡張子なし） =====
    name = input("変換するCSVファイル名（拡張子なし）を入力してください: ").strip()

    input_path = os.path.join(path_dir, f"{name}.csv")

    if not os.path.isfile(input_path):
        print("ファイルが見つかりません:", input_path)
        return

    output_path = os.path.join(path_dir, f"{name}-1.csv")

    delimiter = detect_delimiter(input_path)

    # 1,3,4,6列（0始まり）
    invert_cols = {0, 2, 3, 5}

    with open(input_path, 'r', newline='') as fin, \
         open(output_path, 'w', newline='') as fout:

        reader = csv.reader(fin, delimiter=delimiter)
        writer = csv.writer(fout, delimiter=delimiter)

        for row in reader:
            for i in invert_cols:
                if i < len(row):
                    row[i] = str(-float(row[i]))
            writer.writerow(row)

    print("変換完了")
    print("保存先:", output_path)

if __name__ == "__main__":
    main()
