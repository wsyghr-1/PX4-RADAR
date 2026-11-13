#!/bin/bash
# Inject REC_CMD/REC_ACK/REC_STATUS from record.xml into MAVLink common.xml
COMMON_XML="$1"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RECORD_XML="$SCRIPT_DIR/../../../record.xml"

if grep -q "REC_CMD_TYPE" "$COMMON_XML" 2>/dev/null; then
    echo "record.xml already injected, skipping"
    exit 0
fi

ENUMS_LINE=$(grep -n "</enums>" "$COMMON_XML" | head -1 | cut -d: -f1)
MSGS_LINE=$(grep -n "</messages>" "$COMMON_XML" | tail -1 | cut -d: -f1)
REC_ENUM_START=$(grep -n "<enum name=\"REC_CMD_TYPE\">" "$RECORD_XML" | head -1 | cut -d: -f1)
REC_MSG_START=$(grep -n "<message id=\"16002\"" "$RECORD_XML" | head -1 | cut -d: -f1)

TMPFILE=$(mktemp)
{
  head -n $((ENUMS_LINE - 1)) "$COMMON_XML"
  echo "<!-- RECORD enums -->"
  sed -n "${REC_ENUM_START},$((REC_MSG_START - 1))p" "$RECORD_XML"
  head -n $((MSGS_LINE - 1)) "$COMMON_XML" | tail -n +$ENUMS_LINE
  echo "<!-- RECORD messages -->"
  sed -n "${REC_MSG_START},\$p" "$RECORD_XML"
  tail -n +$MSGS_LINE "$COMMON_XML"
} > "$TMPFILE"
mv "$TMPFILE" "$COMMON_XML"
echo "record.xml injected into common.xml"
