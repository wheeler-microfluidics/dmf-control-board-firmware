#!/usr/bin/env python
from subprocess import check_call, CalledProcessError
import re

from site_scons.path import path


get_target = lambda x: ['libboost_%(name)s%(extension)s', 'libboost_%(name)s-%(variation)s%(extension)s'][x['variation'] is not None] % x


def get_link_info(boost_lib_path):
    libs = path('%s' % boost_path)
    fs = libs.files('*')

    cre_link = re.compile(r'libboost_(?P<name>.*)-mgw\d+-((?P<variation>[^0-9]*)-)?\d+_\d+.*?(?P<extension>(\.\w+)+)')

    link_info = [(f, cre_link.search(f).groupdict()) for f in fs if cre_link.search(f)]
    link_info = [(li[0], get_target(li[1])) for li in link_info]
    return link_info


if __name__ == '__main__':
    import sys
    import os

    if not os.name == 'nt':
        print >> sys.stderr, 'Program must be run in Windows.'
        sys.exit(1)

    if not len(sys.argv) == 2:
        print >> sys.stderr, '''usage: %s <boost lib path> (e.g, 'C:\\boost\\stage\\lib')''' % sys.argv[0]
        sys.exit(1)
    boost_path = sys.argv[1].replace('\\', '/')

    li = get_link_info(boost_path)
    for w in li:
        parent = w[0].parent
        target = w[0]
        link = parent / path(w[1])
        if link.exists():
            link.remove()
        cmd = 'fsutil hardlink create "%s" "%s"' % (link, target)
        try:
            check_call(cmd, shell=True)
        except CalledProcessError, why:
            print 'Warning: link not created - %s' % why
