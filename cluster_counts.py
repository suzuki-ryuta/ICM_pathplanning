import os
import re
import pandas as pd

# ---- フォルダ名を実行時に入力 ----
folder_name = input("クラスタCSVが入ったフォルダ名を入力してください: ")

# 相対パスでクラスタフォルダを指定
cluster_folder = os.path.join("..", "ICM_Log", "path", folder_name)

# フォルダ確認
if not os.path.isdir(cluster_folder):
    print(f"エラー: フォルダが存在しません → {cluster_folder}")
    exit(1)

# ---- 処理開始 ----

# clusters_000.csv などの CSV ファイルをすべて取得
csv_files = [f for f in os.listdir(cluster_folder) if f.endswith(".csv")]

results = []

for filename in sorted(csv_files):
    match = re.search(r"clusters_(\d+)\.csv", filename)
    if not match:
        continue

    cluster_id = match.group(1)
    filepath = os.path.join(cluster_folder, filename)

    with open(filepath, "r") as f:
        lines = f.readlines()

    # 1行目はヘッダなので無視
    num_points = sum(1 for line in lines[1:] if re.search(r"\d", line))

    results.append([cluster_id, num_points])

# ---- DataFrame 作成 ----
df = pd.DataFrame(results, columns=["cluster_id", "num_points"])

# 3列目：空白列
df[""] = ""

# 4列目：差分列 = 前の num_points − 現在の num_points
diff_list = []

for i in range(len(df)):
    if i == 0:
        diff_list.append("")   # 一行目は比較対象が無いので空欄
    else:
        prev_val = df.loc[i - 1, "num_points"]
        curr_val = df.loc[i, "num_points"]
        diff_list.append(prev_val - curr_val)

df["diff"] = diff_list

# ---- Excel をフォルダ内に保存 ----
output_path = os.path.join(cluster_folder, "cluster_counts.xlsx")
df.to_excel(output_path, index=False)

print(f"完了: {output_path} に cluster_counts.xlsx を作成しました")


# import os
# import re
# import pandas as pd

# # ---- フォルダ名を実行時に入力 ----
# folder_name = input("クラスタCSVが入ったフォルダ名を入力してください（例: 2025_11_28_14_51_03）: ")

# # 相対パスでクラスタフォルダを指定
# cluster_folder = os.path.join("..", "ICM_Log", "path", folder_name)

# # フォルダ確認
# if not os.path.isdir(cluster_folder):
#     print(f"エラー: フォルダが存在しません → {cluster_folder}")
#     exit(1)

# # ---- 処理開始 ----

# # clusters_000.csv などの CSV ファイルをすべて取得
# csv_files = [f for f in os.listdir(cluster_folder) if f.endswith(".csv")]

# results = []

# for filename in sorted(csv_files):
#     match = re.search(r"clusters_(\d+)\.csv", filename)
#     if not match:
#         continue

#     cluster_id = match.group(1)
#     filepath = os.path.join(cluster_folder, filename)

#     with open(filepath, "r") as f:
#         lines = f.readlines()

#     # 1行目はヘッダなので無視
#     num_points = sum(1 for line in lines[1:] if re.search(r"\d", line))

#     results.append([cluster_id, num_points])

# # ---- Excelをフォルダ内に保存 ----
# output_path = os.path.join(cluster_folder, "cluster_counts.xlsx")
# df = pd.DataFrame(results, columns=["cluster_id", "num_points"])
# df.to_excel(output_path, index=False)

# print(f"完了: {output_path} に cluster_counts.xlsx を作成しました")
