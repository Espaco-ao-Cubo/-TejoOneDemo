#!/usr/bin/env sh
# Open the TejoOne viewer on Linux.
#   ./start-linux.sh          open it in Chrome / Chromium / Firefox on this computer
#   ./start-linux.sh --serve  also share it on the Wi-Fi for iPhones and iPads (see serve.py)
cd "$(dirname "$0")" || exit 1
PAGE="$(pwd)/index.html"

if [ "$1" = "--serve" ]; then
  exec python3 serve.py "${2:-8000}"
fi

for b in google-chrome google-chrome-stable chromium chromium-browser microsoft-edge firefox; do
  if command -v "$b" >/dev/null 2>&1; then
    nohup "$b" "file://$PAGE" >/dev/null 2>&1 &
    exit 0
  fi
done
if command -v xdg-open >/dev/null 2>&1; then
  xdg-open "file://$PAGE" >/dev/null 2>&1 &
  exit 0
fi
echo "No browser found. Open this file in Chrome or Firefox: $PAGE"
exit 1
