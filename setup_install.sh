# Shell Script to Install FrED Device Package Requirements

#!/bin/bash

# stop script on error
set -e

# Ensure pip is up to date
python3 -m pip install --upgrade pip

# Function to install a package if it's not already available
check_and_install() {
  import_name=$1
  package_name=$2
  friendly_name=$3

  if ! python3 -c "import $import_name" &> /dev/null; then
    printf "\nInstalling %s...\n" "$friendly_name"
    python3 -m pip install "$package_name"
    result=$?
    if [ $result -ne 0 ]; then
      printf "\nERROR: Failed to install %s.\n" "$friendly_name"
      exit $result
    fi
  else
    printf "\n%s is already installed.\n" "$friendly_name"
  fi
}

# Package checks
check_and_install "cv2" "opencv-python" "OpenCV"
check_and_install "yaml" "PyYAML" "PyYAML"
check_and_install "adafruit_blinka" "adafruit-blinka" "Adafruit Blinka"
check_and_install "adafruit_mcp3xxx" "adafruit-circuitpython-mcp3xxx" "Adafruit CircuitPython MCP3xxx"
check_and_install "RPi.GPIO" "RPi.GPIO" "RPi.GPIO"
check_and_install "numpy" "numpy" "NumPy"
check_and_install "matplotlib" "matplotlib" "Matplotlib"
check_and_install "PyQt5" "PyQt5" "PyQt5"
check_and_install "gpiozero" "gpiozero" "GPIO Zero"

printf "\nAll packages are checked and installed if necessary.\n"
