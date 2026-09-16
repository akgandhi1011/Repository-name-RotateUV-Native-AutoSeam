#!/usr/bin/env python3
from __future__ import annotations
import pathlib, re, sys
root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else '.').resolve()
pat = re.compile(r"cmake_minimum_required\s*\(\s*VERSION\s+([0-9]+(?:\.[0-9]+){0,2})([^)]*)\)", re.I)
changed = scanned = 0

def vt(s):
    vals=[int(x) for x in s.split('.')]
    while len(vals)<3: vals.append(0)
    return tuple(vals[:3])

for p in root.rglob('*'):
    if not p.is_file() or not (p.name == 'CMakeLists.txt' or p.suffix.lower() == '.cmake'):
        continue
    scanned += 1
    try: text=p.read_text(encoding='utf-8')
    except UnicodeDecodeError:
        try: text=p.read_text(encoding='latin-1')
        except Exception: continue
    original=text
    def repl(m):
        return f"cmake_minimum_required(VERSION 3.5{m.group(2)})" if vt(m.group(1)) < (3,5,0) else m.group(0)
    text=pat.sub(repl,text)
    if text != original:
        p.write_text(text,encoding='utf-8',newline='\n')
        changed += 1
        print('patched:',p.relative_to(root))
print(f'CMake patch scan complete: scanned={scanned}, changed={changed}')
