import subprocess
import concurrent.futures

# Example command
# python benchmark.py -H hff -s {search} -l benchmarks -b {benchmark_folder} -o {output_file} -t {timeout}

searches = ["ehc", "eehc", "gehc", "aehc", "db_aehc", "ehc+"]
benchmarks = ["rovers", "depot", "miconic", "satellite"]

commands = list()

for search in searches:
    for benchmark in benchmarks:
        command = f"python benchmark.py -H hff -s {search} -l benchmarks -b {benchmark} -o {benchmark}_{search} -t 0"
        commands.append(command)


def run_command(command):
    print(f"Running command: {command}")
    result = subprocess.run(command, shell=True, capture_output=True)
    print(f"Executed command: {command}")
    if result.returncode != 0:
        print(f"Error running command: {command}")
        print(result.stderr)

with concurrent.futures.ThreadPoolExecutor() as executor:
    executor.map(run_command, commands)