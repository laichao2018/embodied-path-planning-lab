#!/bin/sh

set -eu

PROJECT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
DEMO_FILE="$PROJECT_DIR/standalone/index.html"

if [ ! -f "$DEMO_FILE" ]; then
  printf '启动失败：未找到 %s\n' "$DEMO_FILE" >&2
  exit 1
fi

if [ "${1:-}" = "--print" ]; then
  printf '%s\n' "$DEMO_FILE"
  exit 0
fi

case "$(uname -s 2>/dev/null || printf unknown)" in
  Darwin)
    open "$DEMO_FILE"
    ;;
  Linux)
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "$DEMO_FILE" >/dev/null 2>&1 &
    elif command -v sensible-browser >/dev/null 2>&1; then
      sensible-browser "$DEMO_FILE" >/dev/null 2>&1 &
    else
      printf '未找到图形浏览器启动命令，请手动打开：\n%s\n' "$DEMO_FILE"
    fi
    ;;
  MINGW*|MSYS*|CYGWIN*)
    cmd.exe /c start "" "$(cygpath -w "$DEMO_FILE")"
    ;;
  *)
    printf '请在浏览器中打开：\n%s\n' "$DEMO_FILE"
    ;;
esac
