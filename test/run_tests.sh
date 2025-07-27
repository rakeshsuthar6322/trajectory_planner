#!/bin/bash

# Enhanced Testing Script for TheElite Trajectory Planner
# File: comprehensive_test.sh

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
PACKAGE_NAME="TheElite_trajectory_planner"
PYTHON_VERSION="3.10"
ROS_DISTRO="humble"  # Adjust if using different ROS2 distribution

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to create test report directory
create_test_report_dir() {
    local report_dir="test/report"
    if [ ! -d "$report_dir" ]; then
        mkdir -p "$report_dir"
        print_status "Created test report directory: $report_dir"
    fi
}

# Function to backup existing test results
backup_previous_results() {
    local timestamp=$(date +"%Y%m%d_%H%M%S")
    if [ -d "test/htmlcov" ]; then
        mv test/htmlcov "test/htmlcov_backup_$timestamp"
        print_status "Backed up previous coverage results"
    fi
}

# Function to check environment
check_environment() {
    print_status "Checking environment setup..."
    
    # Check ROS2 environment
    if [ -z "$ROS_DISTRO" ] || [ -z "$AMENT_PREFIX_PATH" ]; then
        print_error "ROS2 environment not sourced. Please run: source /opt/ros/$ROS_DISTRO/setup.bash"
        exit 1
    fi
    
    # Check Python version
    python_version=$(python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1-2)
    if [ "$python_version" != "3.10" ]; then
        print_warning "Expected Python 3.10, found $python_version"
    fi
    
    # Check required tools
    local missing_tools=()
    
    if ! command_exists pytest; then
        missing_tools+=("pytest")
    fi
    
    if ! command_exists coverage; then
        missing_tools+=("coverage")
    fi
    
    if ! command_exists colcon; then
        missing_tools+=("colcon")
    fi
    
    if [ ${#missing_tools[@]} -ne 0 ]; then
        print_error "Missing required tools: ${missing_tools[*]}"
        print_status "Install with: pip3 install pytest coverage pytest-cov"
        print_status "Install colcon with: sudo apt install python3-colcon-common-extensions"
        exit 1
    fi
    
    print_success "Environment check passed"
}

# Function to install dependencies
install_dependencies() {
    print_status "Installing/checking dependencies..."
    
    # Check if requirements.txt exists (note: there's a typo in your structure - rquirements.txt)
    if [ -f "test/rquirements.txt" ]; then
        print_status "Installing dependencies from test/rquirements.txt..."
        pip3 install -r test/rquirements.txt
    elif [ -f "requirements.txt" ]; then
        print_status "Installing dependencies from requirements.txt..."
        pip3 install -r requirements.txt
    else
        print_status "Installing common ROS2 testing dependencies..."
        pip3 install pytest pytest-cov coverage rclpy
    fi
    
    print_success "Dependencies installed"
}

# Function to build the package
build_package() {
    print_status "Building ROS2 package..."
    
    # Navigate to workspace root (assuming we're in src/package_name)
    cd ../../
    
    # Build the specific package
    colcon build --packages-select $PACKAGE_NAME --cmake-args -DCMAKE_BUILD_TYPE=Debug
    
    if [ $? -eq 0 ]; then
        print_success "Package built successfully"
    else
        print_error "Package build failed"
        exit 1
    fi
    
    # Source the workspace
    source install/setup.bash
    
    # Return to package directory
    cd src/$PACKAGE_NAME
}

# Function to run unit tests
run_unit_tests() {
    print_status "Running unit tests..."
    
    if [ -d "test/unit" ]; then
        cd test/unit
        
        # Run pytest with coverage
        pytest test_tp_planner.py -v --cov=../../tp_package --cov-report=html:../htmlcov --cov-report=term-missing --cov-report=xml:../report/coverage.xml
        
        local exit_code=$?
        cd ../../
        
        if [ $exit_code -eq 0 ]; then
            print_success "Unit tests passed"
        else
            print_error "Unit tests failed"
            return 1
        fi
    else
        print_warning "No unit tests directory found"
    fi
}

# Function to run integration tests
run_integration_tests() {
    print_status "Running integration tests..."
    
    if [ -d "test/intgration" ] && [ "$(ls -A test/intgration)" ]; then
        cd test/intgration
        
        # Run any Python test files found
        for test_file in test_*.py; do
            if [ -f "$test_file" ]; then
                print_status "Running integration test: $test_file"
                pytest "$test_file" -v
            fi
        done
        
        cd ../../
        print_success "Integration tests completed"
    else
        print_warning "No integration tests found in test/intgration directory"
    fi
}

# Function to run linting and code quality checks
run_code_quality_checks() {
    print_status "Running code quality checks..."
    
    if command_exists flake8; then
        print_status "Running flake8 linting..."
        flake8 tp_package/ --max-line-length=100 --exclude=__pycache__ > test/report/flake8_report.txt 2>&1 || true
    fi
    
    if command_exists pylint; then
        print_status "Running pylint analysis..."
        pylint tp_package/ > test/report/pylint_report.txt 2>&1 || true
    fi
    
    print_success "Code quality checks completed"
}

# Function to run ROS2 specific tests
run_ros2_tests() {
    print_status "Running ROS2 specific tests..."
    
    # Navigate to workspace root
    cd ../../
    
    # Run ROS2 package tests
    colcon test --packages-select $PACKAGE_NAME --event-handlers console_direct+
    
    # Show test results
    colcon test-result --verbose
    
    # Return to package directory
    cd src/$PACKAGE_NAME
    
    print_success "ROS2 tests completed"
}

# Function to generate comprehensive report
generate_report() {
    print_status "Generating test report..."
    
    local report_file="test/report/test_summary_$(date +%Y%m%d_%H%M%S).txt"
    
    cat > "$report_file" << EOF
Trajectory Planner Test Report
Generated: $(date)
Package: $PACKAGE_NAME
Python Version: $(python3 --version)
ROS Distribution: $ROS_DISTRO

=== Test Summary ===
$(if [ -f "test/report/coverage.xml" ]; then echo "Coverage report generated"; else echo "No coverage report"; fi)
$(if [ -f "test/htmlcov/index.html" ]; then echo "HTML coverage report available"; else echo "No HTML coverage report"; fi)
$(if [ -f "test/report/flake8_report.txt" ]; then echo "Flake8 report generated"; else echo "No flake8 report"; fi)
$(if [ -f "test/report/pylint_report.txt" ]; then echo "Pylint report generated"; else echo "No pylint report"; fi)

=== Package Structure ===
$(tree . 2>/dev/null || ls -la)

=== Test Files ===
$(find test/ -name "*.py" -type f 2>/dev/null || echo "No Python test files found")

EOF
    
    print_success "Test report saved to: $report_file"
}

# Function to open coverage report
open_coverage_report() {
    if [ -f "test/htmlcov/index.html" ]; then
        print_status "Coverage report available at: test/htmlcov/index.html"
        
        # Try to open in browser (Linux)
        if command_exists xdg-open; then
            read -p "Open coverage report in browser? (y/n): " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                xdg-open test/htmlcov/index.html
            fi
        fi
    fi
}

# Function to cleanup temporary files
cleanup() {
    print_status "Cleaning up temporary files..."
    
    # Remove Python cache files
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
    find . -name "*.pyc" -delete 2>/dev/null || true
    
    print_success "Cleanup completed"
}

# Main execution function
main() {
    print_status "Starting comprehensive test suite for $PACKAGE_NAME"
    print_status "Current directory: $(pwd)"
    
    # Create necessary directories
    create_test_report_dir
    
    # Backup previous results
    backup_previous_results
    
    # Run all test phases
    check_environment
    install_dependencies
    build_package
    
    # Run tests
    local test_failed=false
    
    if ! run_unit_tests; then
        test_failed=true
    fi
    
    run_integration_tests
    run_code_quality_checks
    run_ros2_tests
    
    # Generate reports
    generate_report
    open_coverage_report
    
    # Final status
    if [ "$test_failed" = true ]; then
        print_error "Some tests failed. Check the reports for details."
        exit 1
    else
        print_success "All tests completed successfully!"
    fi
    
    # Cleanup
    cleanup
    
    print_success "Test suite completed!"
}

# Script execution
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
