import sys
import os
from itertools import chain

from path import path

home_dir = path('~').expand()

ARDUINO_SEARCH_PATHS = [home_dir, ]
if os.name == 'nt':
    AVRDUDE_NAME = 'avrdude.exe'
    ARDUINO_SEARCH_PATHS += [home_dir / path('Documents'),
                            path('%SYSTEMDRIVE%/').expand(),
                            path('%PROGRAMFILES%').expand(), ]
else:
    AVRDUDE_NAME = 'avrdude'
    ARDUINO_SEARCH_PATHS += [home_dir / path('local/opt'), ]


def get_arduino_paths():
    fs = []
    for p in ARDUINO_SEARCH_PATHS:
        fs += get_avrdude_list(p)

    if not fs:
        print >> sys.stderr, '''\
    ERROR: arduino install directory not found!

    Searched:
        %s''' % '\n    '.join(ARDUINO_SEARCH_PATHS)
        sys.exit(1)
    fs.sort(key=lambda x: -x.ctime)

    avrdude = fs[0]
    p = avrdude.parent
    while p and p.name != 'hardware':
        p = p.parent
    if not p:
        print >> sys.stderr, '''Arduino install path not found.'''
        sys.exit(1)
    arduino_path = p.parent
    avrdude_conf = list(arduino_path.walkfiles('avrdude.conf'))
    if not avrdude_conf:
        print >> sys.stderr, '''avrdude configuration (avrdude.conf) path not found.'''
        sys.exit(1)
    else:
        avrdude_conf = avrdude_conf[0]
    return arduino_path, avrdude, avrdude_conf


def get_avrdude_list(p):
    return list(set(chain(*[d.walkfiles(AVRDUDE_NAME) for d in p.dirs('arduino*')])))


if __name__ == '__main__':
    arduino_path, avrdude, avrdude_conf = get_arduino_paths()
    print 'found arduino path:', arduino_path
    print 'using newest avrdude:', avrdude
    print 'using avrdude config:', avrdude_conf
