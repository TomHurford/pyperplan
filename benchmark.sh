#!/bin/bash

# Iterate over all subdirectories of the benchmarks directory
for problem_set_dir in benchmarks/*; do
  # Ensure it's a directory
  if [ -d "$problem_set_dir" ]; then
    # Extract the problem name from the directory path
    problem_name=$(basename $problem_set_dir)
    echo "Problem name: $problem_name"

    # Find the domain files, there are several domain files have the word "domain" in their name
    domain_files=$(find $problem_set_dir -name "*domain*")
    # Order the domain files alphabetically
    domain_files=$(echo $domain_files | tr " " "\n" | sort | tr "\n" " ")
    # echo "Domain files: $domain_files"

    # Find the task files, there are several task files have the word "task" in their name, we also need to check that they dont have .soln in their name
    task_files=$(find $problem_set_dir -name "*task*" | grep -v ".soln")
    # Order the task files alphabetically
    task_files=$(echo $task_files | tr " " "\n" | sort | tr "\n" " ")

    # If the length of the domain files is the same as the length of the task files then we can assume that there is a task file for each domain file
    if [ $(echo $domain_files | wc -w) -eq $(echo $task_files | wc -w) ]; then
      # For each domain file run pyperplan on the corresponding task file
      # Convert the space-separated strings of file paths into arrays
      domain_files_array=($domain_files)
      task_files_array=($task_files)

      # Get the number of files
      num_files=${#domain_files_array[@]}

      # For each domain file run pyperplan on the corresponding task file
      for (( i=0; i<$num_files; i++ )); do
        domain_file=${domain_files_array[$i]}
        task_file=${task_files_array[$i]}
        python3 pyperplan -H hff -s ehs -l info "$domain_file" "$task_file"
        wait
      done
    else
      # There is one domain file and multiple task files that need to be run with it
      # For each task file run pyperplan on the domain file
      for task_file in $task_files; do
        python3 pyperplan -H hff -s ehs -l info "$domain_files" "$task_file"
        wait
      done
    fi
  fi
  # wait for the user to press enter before continuing
  read -p "Press enter to continue"
  # Wait for background processes to finish
  wait
done
