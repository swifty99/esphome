from esphome.loader import FileResource
from tests.testing_helpers import ComponentManifestOverride


def override_manifest(manifest: ComponentManifestOverride) -> None:
    # Only the hardware independent decoder is under test, the bus drivers need a real pin
    manifest.resources = [
        FileResource("esphome.components.gpio.one_wire", "one_wire_rmt_decode.h")
    ]
