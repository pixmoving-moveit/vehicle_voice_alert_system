#!/usr/bin/env bash
# Install Python libraries: simpleaudio, gtts, pyudev, pyaudio, pygame, pydub.
# Make sure you have Python, pip, and portaudio19-dev installed before running the script.
# Note: -y option is only for CI.

set -e

# Parse arguments
option_yes=false
option_verbose=false

while [ "$1" != "" ]; do
    case "$1" in
    -y)
        option_yes=true
        ;;
    -v)
        option_verbose=true
        ;;
    *)
        echo "Unknown option: $1"
        exit 1
        ;;
    esac
    shift
done

# Confirm to start installation
if [ "$option_yes" = "true" ]; then
    echo -e "\e[36mInstalling Python libraries in non-interactive mode.\e[m"
else
    echo -e "\e[33mThis script will install Python libraries: simpleaudio, gtts, pyudev, pyaudio, alsaaudio, pulsectl, pygame, pydub.\e[m"
    read -rp ">  Are you sure you want to run setup? [y/N] " answer

    if ! [[ ${answer:0:1} =~ y|Y ]]; then
        echo -e "\e[33mCancelled.\e[0m"
        exit 1
    fi
fi

# Check verbose option
if [ "$option_verbose" = "true" ]; then
    pip_install_args="--upgrade --verbose"
else
    pip_install_args="--upgrade"
fi

# Install portaudio19-dev
if ! dpkg -s portaudio19-dev >/dev/null 2>&1; then
    echo -e "\e[36mInstalling portaudio19-dev.\e[m"
    sudo apt-get install -y portaudio19-dev
fi

# Install pavucontrol
if ! dpkg -s pavucontrol >/dev/null 2>&1; then
    echo -e "\e[36mInstalling pavucontrol.\e[m"
    sudo apt-get install -y pavucontrol
fi

# Install Python libraries
python3 -m pip install $pip_install_args simpleaudio gtts pyudev pyaudio pygame pydub

# Check if installation was successful
if [ $? -eq 0 ]; then
    echo -e "\e[32mPython libraries installed successfully.\e[0m"
    exit 0
else
    echo -e "\e[31mFailed to install Python libraries.\e[0m"
    exit 1
fi
