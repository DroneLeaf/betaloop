import argparse
import configparser
import os

from start import Betaloop, logger


ROCKET_WORLD_FILE = "rocket_drone.world"


def _resolve_existing_path(primary_path, fallback_relative_path):
    if os.path.exists(primary_path):
        return primary_path

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    fallback_path = os.path.join(repo_root, fallback_relative_path)
    if os.path.exists(fallback_path):
        return fallback_path

    return primary_path


def _load_defaults_from_config(config_path):
    config = configparser.ConfigParser()
    if os.path.exists(config_path):
        config.read(config_path)
    else:
        logger.warning(
            "Could not find configuration file %s, falling back to defaults and command line arguments",
            config_path,
        )

    section = config["Betaloop"] if "Betaloop" in config else {}

    gazebo_assets_home = section.get(
        "AeroloopGazeboHome",
        _resolve_existing_path("/opt/aeroloop_gazebo", "aeroloop_gazebo"),
    )
    world = os.path.join(gazebo_assets_home, "worlds", ROCKET_WORLD_FILE)
    elf = section.get(
        "BetaflightElf",
        _resolve_existing_path(
            "/opt/betaflight/obj/main/betaflight_SITL.elf",
            "betaflight/obj/main/betaflight_SITL.elf",
        ),
    )
    msp_virtual_radio_home = section.get(
        "MspVirtualRadioHome",
        _resolve_existing_path("/opt/msp_virtualradio", "msp_virtualradio"),
    )
    transmitter = os.path.join(msp_virtual_radio_home, "emu-dx6-msp.js")
    vidrecv = section.get("Vidrecv", "vidrecv")

    return gazebo_assets_home, world, elf, transmitter, vidrecv


if __name__ == "__main__":
    gazebo_assets_home, world, elf, transmitter, vidrecv = _load_defaults_from_config("config.txt")

    parser = argparse.ArgumentParser("Betaloop Rocket Drone Launcher")
    parser.add_argument("--gazebo-assets", type=str, default=gazebo_assets_home)
    parser.add_argument("--world", type=str, default=world)
    parser.add_argument("--elf", type=str, default=elf)
    parser.add_argument("--transmitter", type=str, default=transmitter)
    parser.add_argument("--vidrecv", type=str, default=vidrecv)
    parser.add_argument("--gazebo", help="Start in Gazebo, not FPV mode", action="store_true")
    parser.add_argument("-l", "--list", help="List available worlds", action="store_true")

    args = parser.parse_args()

    betaloop = Betaloop(
        args.gazebo_assets,
        args.world,
        args.elf,
        args.transmitter,
        args.vidrecv,
        args.gazebo,
    )
    if args.list:
        betaloop.list_worlds()
    else:
        betaloop.start()
