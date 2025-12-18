# select_csv_and_extract_first_column.py
import csv
import glob

csv_files = sorted(glob.glob("/data/*.csv"))

if not csv_files:
    raise RuntimeError("No CSV files found.")

for i, path in enumerate(csv_files):
    print(f"[{i}] {path}")

idx = int(input("Select file number: "))
if idx < 0 or idx >= len(csv_files):
    raise ValueError("Invalid selection.")

csv_path = csv_files[idx]

with open(csv_path, newline="", encoding="utf-8") as f:
    reader = csv.reader(f)
    first_column = [row[0] for row in reader if row]

with open("first_column.txt", "w", encoding="utf-8") as f:
    f.write("\n".join(first_column))
print(f"Extracted first column from {csv_path} to first_column.txt")