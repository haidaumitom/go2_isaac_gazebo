#!/usr/bin/env bash

COLOR_ERROR='\033[0;31m'
COLOR_SUCCESS='\033[0;32m'
COLOR_WARNING='\033[1;33m'
COLOR_INFO='\033[0;34m'
COLOR_RESET='\033[0m'

print_success() {
    echo -e "${COLOR_SUCCESS}[SUCCESS]${COLOR_RESET} $1"
}

print_warning() {
    echo -e "${COLOR_WARNING}[WARNING]${COLOR_RESET} $1"
}

print_error() {
    echo -e "${COLOR_ERROR}[ERROR]${COLOR_RESET} $1"
}

print_info() {
    echo -e "${COLOR_INFO}[INFO]${COLOR_RESET} $1"
}

print_separator() {
    echo -e "${COLOR_INFO}-------------------------------------------------------------------${COLOR_RESET}"
}

print_header() {
    print_separator
    echo -e "${COLOR_INFO}$1${COLOR_RESET}"
}

ask_confirmation() {
    local message="$1"
    local response

    while true; do
        echo -e -n "${COLOR_WARNING}$message (y/n): ${COLOR_RESET}"
        read -r response
        case "$response" in
            [yY]) return 0 ;;
            [nN]) return 1 ;;
            *) print_error "Please enter 'y' or 'n'" ;;
        esac
    done
}

is_online() {
    local host="${1:-github.com}"
    local timeout="${2:-3}"

    if command -v curl &> /dev/null; then
        curl --connect-timeout "$timeout" --silent --head "$host" &> /dev/null
        return $?
    fi

    if command -v wget &> /dev/null; then
        wget --timeout="$timeout" --tries=1 --spider --quiet "$host" &> /dev/null
        return $?
    fi

    if command -v ping &> /dev/null; then
        ping -c 1 -W "$timeout" "$host" &> /dev/null
        return $?
    fi

    return 1
}

check_network_status() {
    local host="${1:-github.com}"

    if is_online "$host"; then
        print_info "Network available, checking for updates..."
        return 0
    else
        print_warning "Network unavailable or cannot access $host"
        print_info "Using local files (offline mode)"
        return 1
    fi
}
