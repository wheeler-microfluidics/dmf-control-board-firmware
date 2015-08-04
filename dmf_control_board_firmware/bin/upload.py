import sys
from collections import OrderedDict

from arduino_rpc.upload import get_arg_parser, upload
from .. import get_firmwares


def parse_args(firmwares, argv=None):
    if argv is None:
        argv = sys.argv[1:]

    parser = get_arg_parser()
    parser.add_argument('board_version', nargs='?', choices=firmwares,
                        default=firmwares[-1])

    return parser.parse_args(argv)


if __name__ == '__main__':
    firmware_versions = [(p.parent.name.replace('_', '.'), p)
                         for p in get_firmwares()['mega2560']]
    firmware_versions.sort()
    firmwares = OrderedDict(firmware_versions)
    args = parse_args(firmwares.keys())

    upload(args.board_name, lambda b: firmwares[args.board_version], args.port,
           args.arduino_install_home, verify=not args.skip_verify)
