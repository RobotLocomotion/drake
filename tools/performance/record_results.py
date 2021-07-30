"""Run a google benchmark program, with restricted processor affinity to
mitigate variance of measurements. Collect environment information and store it
alongside the benchmark results.
"""

# TODO(rpoyner-tri) find a robust way of recording source code version
# information.

# TODO(rpoyner-tri) make affinity configurable if some new use case needs to
# run on multiple cores.

import os
import subprocess
import sys


def main():
    benchmark = sys.argv[1]
    command_tail = sys.argv[2:]

    results_dir = os.environ['TEST_UNDECLARED_OUTPUTS_DIR']
    src_dir = os.environ['TEST_SRCDIR']

    # Debug-configured test runs are nice for coverage, but not very useful
    # otherwise. Don't waste too much time on them.
    if '-dbg/' in results_dir:
        command_tail.append("--benchmark_repetitions=1")

    os_data = os.uname()

    # Record kernel.
    kernel_results = os.path.join(results_dir, "kernel.txt")
    with open(kernel_results, 'w', encoding='utf-8') as kernel_file:
        print(' '.join(list(os_data)), file=kernel_file)

    # Record OS.
    os_results = os.path.join(results_dir, "os.txt")
    if os_data.sysname == 'Linux':
        os_txt = subprocess.run(
            ["lsb_release", "-idrc"],
            check=True, stdout=subprocess.PIPE, encoding='utf-8').stdout
    elif os_data.sysname == 'Darwin':
        os_txt = subprocess.run(
            ["sw_vers"],
            check=True, stdout=subprocess.PIPE, encoding='utf-8').stdout
    else:
        os_txt = "unknown"
    with open(os_results, 'w', encoding='utf-8') as os_file:
        print(os_txt, file=os_file)

    # Record compiler.
    id_compiler = os.path.join(
        src_dir, 'drake/tools/workspace/cc/identify_compiler')
    compiler_results = os.path.join(results_dir, "compiler.txt")
    with open(compiler_results, 'w', encoding='utf-8') as compiler_file:
        os_txt = subprocess.run(
            [id_compiler], check=True, stdout=compiler_file)

    # Set affinity command, based on platform.
    affinity_command = []
    if os_data.sysname == 'Linux':
        # Choosing processor #0 is arbitrary. It is up to experimenters
        # to ensure it is reliably idle during experiments.
        affinity_command = ["taskset", "0x1"]

    # Run benchmark.
    benchmark_results = os.path.join(results_dir, "summary.txt")
    benchmark_json_results = os.path.join(results_dir, "results.json")
    benchmark_exe = os.path.join(src_dir, benchmark)
    benchmark_command = [
        benchmark_exe,
        '--benchmark_display_aggregates_only=true',
        '--benchmark_repetitions=9',
        '--benchmark_out_format=json',
        f'--benchmark_out={benchmark_json_results}',
    ]
    with open(benchmark_results, 'w', encoding='utf-8') as benchmark_file:
        os_txt = subprocess.run(
            affinity_command + benchmark_command + command_tail,
            check=True, stdout=benchmark_file, stderr=subprocess.STDOUT)

    # Announce.
    print("Full results are in:")
    print(results_dir + "/")


if __name__ == '__main__':
    main()
