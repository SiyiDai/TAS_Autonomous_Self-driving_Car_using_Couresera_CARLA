def argparser_add_argument(argparser):
    """
    Args:
        -v, --verbose: print debug information
        --host: IP of the host server (default: localhost)
        -p, --port: TCP port to listen to (default: 2000)
        -a, --autopilot: enable autopilot
        -q, --quality-level: graphics quality level [Low or Epic]
        -i, --images-to-disk: save images to disk
        -c, --carla-settings: Path to CarlaSettings.ini file
    """
    argparser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="debug",
        help="print debug information",
    )
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host server (default: localhost)",
    )
    argparser.add_argument(
        "-p",
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port to listen to (default: 2000)",
    )
    argparser.add_argument(
        "-a", "--autopilot", action="store_true", help="enable autopilot"
    )
    argparser.add_argument(
        "-q",
        "--quality-level",
        choices=["Low", "Epic"],
        type=lambda s: s.title(),
        default="Low",
        help="graphics quality level.",
    )
    argparser.add_argument(
        "-c",
        "--carla-settings",
        metavar="PATH",
        dest="settings_filepath",
        default=None,
        help='Path to a "CarlaSettings.ini" file',
    )
