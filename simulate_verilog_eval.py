import json
import os
import tempfile
import subprocess
from pathlib import Path

# 請根據您的實際情況修改
INPUT_JSONL = "verilog_eval_summary.jsonl"  # 您提供的 JSONL 檔案
OUTPUT_REPORT = "simulation_report.md"

# iverilog 命令
IVERILOG_CMD = ["iverilog", "-Wall", "-Winfloop", "-Wno-timescale", "-g2012", "-s", "tb"]
VVP_CMD = ["vvp"]

def run_simulation(generated_code, ref_code, testbench_code, problem_id):
    """
    執行模擬，返回 (success, output)
    success: True 表示 PASS, False 表示 FAIL
    output: 模擬的完整輸出
    """
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_dir = Path(temp_dir)
        combined_file = temp_dir / f"{problem_id}.v"
        vvp_output = temp_dir / "a.out"
        
        # 將 generated, ref, 和 testbench 三者合併
        # 重點：將 ref_code 中的 'RefModule' 定義包含進來，供 testbench 使用
        combined_content = ""
        
        # 1. 添加 generated code (作為 TopModule)
        #    有些 generated code 可能已經有 module TopModule...endmodule，有些可能只有內部邏輯
        #    為了保險，我們先檢查並確保它是一個完整的模組
        if not generated_code.strip().startswith("module TopModule"):
            # 如果不是以 module TopModule 開頭，我們手動包裝
            combined_content += "module TopModule ();\n"
            combined_content += generated_code
            combined_content += "\nendmodule\n"
        else:
            combined_content += generated_code
            # 確保以 endmodule 結尾
            if not generated_code.strip().endswith("endmodule"):
                combined_content += "\nendmodule\n"
        
        combined_content += "\n\n"
        
        # 2. 添加 ref code (作為 RefModule)
        #    同樣地，確保它是一個完整的模組定義
        if not ref_code.strip().startswith("module RefModule"):
            # 如果不是以 module RefModule 開頭，我們手動包裝
            combined_content += "module RefModule ();\n"
            combined_content += ref_code
            combined_content += "\nendmodule\n"
        else:
            combined_content += ref_code
            # 確保以 endmodule 結尾
            if not ref_code.strip().endswith("endmodule"):
                combined_content += "\nendmodule\n"
        
        combined_content += "\n\n"
        
        # 3. 添加 testbench code
        combined_content += testbench_code
        
        # 寫入合併後的檔案
        with open(combined_file, 'w') as f:
            f.write(combined_content)
        
        # Step 1: 執行 iverilog 編譯
        try:
            compile_result = subprocess.run(
                IVERILOG_CMD + ["-o", str(vvp_output), str(combined_file)],
                capture_output=True,
                text=True,
                cwd=temp_dir
            )
        except FileNotFoundError:
            return False, "Error: iverilog command not found. Please ensure iverilog is installed and in your PATH."
        
        if compile_result.returncode != 0:
            return False, f"Compilation Failed:\n{compile_result.stderr}"
        
        # Step 2: 執行 vvp 模擬
        try:
            sim_result = subprocess.run(
                VVP_CMD + [str(vvp_output)],
                capture_output=True,
                text=True,
                cwd=temp_dir
            )
        except FileNotFoundError:
            return False, "Error: vvp command not found. Please ensure iverilog is installed and in your PATH."
        
        full_output = sim_result.stdout + "\n" + sim_result.stderr
        
        # 判斷結果
        # 根據您提供的 testbench，它會在失敗時輸出 `INCORRECT`，成功時輸出 `OK`
        if "INCORRECT" in full_output:
            return False, full_output
        elif "OK" in full_output:
            return True, full_output
        elif "TIMEOUT" in full_output:
            return False, full_output
        else:
            # 如果沒有任何標誌，但編譯成功，可以視為通過（或根據需要調整）
            return True, full_output

def main():
    # 讀取 JSONL 檔案
    records = []
    with open(INPUT_JSONL, 'r') as f:
        for line in f:
            if line.strip():  # 忽略空行
                records.append(json.loads(line))
    
    # 按 ID 排序
    records.sort(key=lambda x: x['id'])
    
    # 準備報告
    report_lines = ["# Verilog Evaluation Simulation Report", ""]
    report_lines.append("| ID | Result |")
    report_lines.append("|----|--------|")
    
    pass_count = 0
    total_count = len(records)
    
    print(f"開始模擬 {total_count} 個問題...")
    
    for record in records:
        problem_id = record['id']
        generated_code = record['generated']
        ref_code = record['ref']  # 新增：讀取 ref code
        testbench_code = record['test']
        
        print(f"正在模擬 {problem_id}...")
        success, output = run_simulation(generated_code, ref_code, testbench_code, problem_id)
        
        result = "✅PASS" if success else "❌FAIL"
        if success:
            pass_count += 1
        report_lines.append(f"| {problem_id} | {result} |")
        
        # （可選）將詳細的模擬輸出保存到單獨的日誌文件中
        # log_file = f"{problem_id}_sim.log"
        # with open(log_file, 'w') as f:
        #     f.write(output)
        
    # 添加統計
    report_lines.append("")
    report_lines.append(f"## 統計")
    report_lines.append(f"總共 {total_count} 題，通過 {pass_count} 題，失敗 {total_count - pass_count} 題。")
    report_lines.append(f"成功率: {pass_count/total_count*100:.2f}%")
    
    # 寫入報告
    with open(OUTPUT_REPORT, 'w') as f:
        f.write('\n'.join(report_lines))
    
    print(f"模擬完成！")
    print(f"報告已生成: {OUTPUT_REPORT}")

if __name__ == "__main__":
    main()