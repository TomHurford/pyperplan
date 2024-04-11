import os
from socket import timeout
import subprocess
import argparse
import time
import math

# Define argument parser
parser = argparse.ArgumentParser(description='Run Pyperplan benchmarks.')
# Heuristic, search algorithm, and benchmark folder arguments
parser.add_argument('-H', '--heuristic', type=str, default='hff', help='Heuristic to use')
parser.add_argument('-s', '--search', type=str, default='ehs2', help='Search algorithm to use')
parser.add_argument('-b', '--benchmark_folder', type=str, default='blocks', help='Folder with task files')
parser.add_argument('-l', '--loglevel', type=str, default='info', help='Log level')
parser.add_argument('-o', '--output_file', type=str, default="", help='Output file')
parser.add_argument('-t', '--timeout', type=int, default=60, help='Timeout in seconds')
# Parse arguments
args = parser.parse_args()

# Use parsed arguments in the pyperplan command
pyperplan_command = f"pyperplan -H {args.heuristic} -s {args.search} -l {args.loglevel} -o {args.output_file} "
domain_file = f"./benchmarks/{args.benchmark_folder}/domain.pddl"  # Your domain file
gripper_folder = f"./benchmarks/{args.benchmark_folder}/"  # Folder with task files
timeout_seconds = args.timeout  # 1 minutes in seconds

task_files = [f for f in os.listdir(gripper_folder) if f.endswith('.pddl') and f != 'domain.pddl']

task_files.sort()

for task_file in task_files:
    task_path = os.path.join(gripper_folder, task_file)
    command = f"{pyperplan_command} {domain_file} {task_path}"

    start_time = time.time()

    if timeout_seconds == 0:
        subprocess.run(command, shell=True)
    else:
        try:
            subprocess.run(command, shell=True, timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - Search for task {task_file} timed out after {math.floor(timeout_seconds / 60)} minute(s).")
