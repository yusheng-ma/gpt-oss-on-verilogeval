import os
from pathlib import Path
import json

# 請根據您的實際路徑修改這兩個變數
BUILD_DIR = "build"  # 模型生成程式碼的目錄
DATASET_DIR = "dataset_spec-to-rtl"  # 原始資料集目錄
OUTPUT_JSONL = "verilog_eval_summary.jsonl"
OUTPUT_MD = "verilog_eval_detailed.md"

def read_file_safely(file_path):
    """安全地讀取檔案，如果檔案不存在則返回一個提示。"""
    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as f:
            return f.read().strip()
    return "[檔案未找到]"

def main():
    # 準備輸出檔案
    jsonl_file = open(OUTPUT_JSONL, 'w', encoding='utf-8')
    md_lines = ["# Verilog Evaluation Detailed Summary", ""]

    # 獲取 dataset_spec-to-rtl 目錄下所有 _prompt.txt 檔案，並從中提取 ID 進行排序
    # 這樣可以確保遍歷的順序是從 Prob001 到 Prob156
    dataset_path = Path(DATASET_DIR)
    prompt_files = sorted(dataset_path.glob("*_prompt.txt"))
    
    processed_count = 0

    for prompt_file in prompt_files:
        # 從 prompt 檔名提取問題 ID (例如 Prob001_zero_prompt.txt -> Prob001_zero)
        problem_id = prompt_file.name.split('_prompt.txt')[0]
        
        # 構建對應檔案的路徑
        ref_path = Path(DATASET_DIR) / f"{problem_id}_ref.sv"
        test_path = Path(DATASET_DIR) / f"{problem_id}_test.sv"
        # 在 build 目錄中，生成的檔案位於 {id}/ 目錄下
        generated_path = Path(BUILD_DIR) / problem_id / f"{problem_id}_sample01.sv"

        # 讀取所有檔案內容
        prompt_content = read_file_safely(prompt_file)
        ref_content = read_file_safely(ref_path)
        test_content = read_file_safely(test_path)
        generated_content = read_file_safely(generated_path)

        # 建立 JSON 物件 (包含 test)
        record = {
            "id": problem_id,
            "prompt": prompt_content,
            "ref": ref_content,
            "test": test_content,
            "generated": generated_content
        }
        # 寫入 JSONL 檔案
        jsonl_file.write(json.dumps(record, ensure_ascii=False) + '\n')

        # 添加到 Markdown 文件 (按您要求的格式，test 不在 MD 中)
        md_lines.append(f"### {problem_id}")
        
        # 使用 HTML <details> 標籤建立可折疊的 prompt
        md_lines.append("<details>")
        md_lines.append(f"<summary>prompt</summary>")
        md_lines.append("")
        md_lines.append("```text")
        md_lines.append(prompt_content)
        md_lines.append("```")
        md_lines.append("</details>")
        md_lines.append("")

        # Reference
        md_lines.append("#### reference")
        md_lines.append("```verilog")
        md_lines.append(ref_content)
        md_lines.append("```")
        md_lines.append("")

        # Model Output
        md_lines.append("#### model output")
        md_lines.append("```verilog")
        md_lines.append(generated_content)
        md_lines.append("```")
        md_lines.append("---")  # 用分隔線區分不同問題
        md_lines.append("")

        processed_count += 1

    # 關閉檔案
    jsonl_file.close()

    # 寫入 Markdown 檔案
    with open(OUTPUT_MD, 'w', encoding='utf-8') as md_file:
        md_file.write('\n'.join(md_lines))

    print(f"處理完畢！")
    print(f"總共處理了 {processed_count} 個問題，並已按 ID 數字排序。")
    print(f"JSONL 檔案已生成: {OUTPUT_JSONL}")
    print(f"Markdown 檔案已生成: {OUTPUT_MD}")

if __name__ == "__main__":
    main()