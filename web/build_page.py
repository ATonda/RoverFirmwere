#!/usr/bin/env python3
# Vygeneruje src/webpage.h z web/index.html – stránka se vkládá do firmwaru zabalená gzipem
# (prohlížeč ji rozbalí sám, hlavička Content-Encoding: gzip)
import gzip
import os
here = os.path.dirname(os.path.abspath(__file__))
html = open(os.path.join(here, 'index.html'), 'rb').read()
gz = gzip.compress(html, 9, mtime=0)
lines = []
for i in range(0, len(gz), 20):
    lines.append('  ' + ','.join('0x%02x' % b for b in gz[i:i + 20]) + ',')
out = ('// Vygenerováno z web/index.html skriptem web/build_page.py – needitovat ručně\n'
       '#pragma once\n#include <Arduino.h>\n\n'
       'static const size_t PAGE_GZ_LEN = %d;  // nezabaleno %d B\n'
       'static const uint8_t PAGE_GZ[] PROGMEM = {\n%s\n};\n') % (len(gz), len(html), '\n'.join(lines))
open(os.path.join(here, '..', 'src', 'webpage.h'), 'w', encoding='utf-8').write(out)
print('webpage.h: %d B, gzip %d B' % (len(html), len(gz)))
