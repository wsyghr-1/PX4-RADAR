#!/usr/bin/env python3
"""Inject REC_CMD/REC_ACK/REC_STATUS from record.xml into MAVLink common.xml."""
import sys, os

common_xml = sys.argv[1]
# Repo root is 3 levels up from this script
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
record_xml = os.path.join(repo_root, "record.xml")

with open(common_xml) as f: c = f.read()
if "REC_CMD_TYPE" in c:
    print("record.xml already injected, skipping")
    sys.exit(0)

with open(record_xml) as f: r = f.read()

es = c.find("</enums>")
ms = r.find("<message id=\"16002\">")
enums = r[r.find("<enum name=\"REC_CMD_TYPE\">"):ms]
c = c[:es] + "\n<!-- RECORD enums -->\n" + enums + c[es:]

ms2 = c.rfind("</messages>")
c = c[:ms2] + "\n<!-- RECORD messages -->\n" + r[ms:] + c[ms2:]

with open(common_xml, "w") as f: f.write(c)
print("record.xml injected into common.xml")
