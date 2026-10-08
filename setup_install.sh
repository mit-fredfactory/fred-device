# Shell Script to Install FrED Device Package Requirements
# Versions pinned to match the device build

#!/bin/bash

# stop script on error
set -e

# Ensure pip is up to date
python3 -m pip install --upgrade pip

# Function to install a package at a specific pinned version.
# Checks the version actually installed (via pip), not just whether the
# module can be imported, so a mismatched/newer version gets corrected.
check_and_install() {
  package_name=$1
  friendly_name=$2
  version=$3

  installed_version=$(python3 -m pip show "$package_name" 2>/dev/null | awk -F': ' '/^Version:/{print $2}')

  if [ "$installed_version" == "$version" ]; then
    printf "\n%s %s is already installed.\n" "$friendly_name" "$version"
  else
    if [ -z "$installed_version" ]; then
      printf "\nInstalling %s==%s...\n" "$friendly_name" "$version"
    else
      printf "\n%s is installed at %s, changing to %s...\n" "$friendly_name" "$installed_version" "$version"
    fi
    python3 -m pip install "$package_name==$version"
    result=$?
    if [ $result -ne 0 ]; then
      printf "\nERROR: Failed to install %s==%s.\n" "$friendly_name" "$version"
      exit $result
    fi
  fi
}

# Package checks — versions pinned to the device's package list
check_and_install "opencv-python" "OpenCV" "4.12.0.88"
check_and_install "PyYAML" "PyYAML" "6.0.2"
check_and_install "adafruit-blinka" "Adafruit Blinka" "8.67.0"
check_and_install "adafruit-circuitpython-mcp3xxx" "Adafruit CircuitPython MCP3xxx" "1.4.22"
check_and_install "RPi.GPIO" "RPi.GPIO" "0.7.1"
check_and_install "numpy" "NumPy" "2.2.4"
check_and_install "matplotlib" "Matplotlib" "3.10.7"
check_and_install "PyQt5" "PyQt5" "5.15.11"
check_and_install "gpiozero" "GPIO Zero" "2.0.1"

printf "\nAll packages are checked and pinned to the device's versions.\n"
