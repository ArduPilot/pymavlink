#!/bin/bash

set -e
# set -x

echo "Checking dev environment ... "
if ! command -v pre-commit &> /dev/null
then
    echo "pre-commit not found, we need it for all test"
    echo "please install it using pip : "
    echo "python3 -m pip install -U --user pre-commit"
    echo "You can found more information about it on https://pre-commit.com"
    exit 1
fi

# list of packages to check and install
PACKAGES=("flake8" "pytest" "pytest-mock" "numpy" "types-PyYAML" "types-requests" "types-setuptools" "types-protobuf" "pre-commit")
pip_list=$(pip list)
echo -e "\033[32mChecking required packages \033[0m"
for package in "${PACKAGES[@]}"; do
    if ! echo "$pip_list" | grep -q "^${package} "; then
        echo -e "\033[31m Package '$package' is not installed. Installing... \033[0m"
        python3 -m pip install -U --disable-pip-version-check --user "$package"
    fi
done
echo ""

echo "Running pre-commit on changed files..."
pre-commit run --color=always --all-files
