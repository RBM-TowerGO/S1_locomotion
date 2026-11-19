#!/usr/bin/env bash
set -e
NAV_SCRIPTS=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
for arg in "$@"; do
  case "$arg" in -h|--help) PYTHONPATH="$NAV_SCRIPTS${PYTHONPATH:+:$PYTHONPATH}" exec /usr/bin/python3 -m real_navigation.web "$@" ;; esac
done
source "$NAV_SCRIPTS/navigation_env.sh" "$@"
exec /usr/bin/python3 -m real_navigation.web "$@"
