#!/usr/bin/env python3
"""Apply generic ArduPilot-compatible APIs to generated MAVSDK sources."""

from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def read_source(path: Path) -> tuple[str, str]:
    """Read generated source and return normalized text plus its newline style."""
    raw = path.read_bytes().decode("utf-8")
    newline = "\r\n" if "\r\n" in raw else "\n"
    return raw.replace("\r\n", "\n"), newline


def write_source(path: Path, text: str, newline: str) -> None:
    """Write generated source using the original newline style."""
    if not text.endswith("\n"):
        text += "\n"
    path.write_bytes(text.replace("\n", newline).encode("utf-8"))


def replace_once(text: str, marker: str, replacement: str, path: Path) -> str:
    """Replace one generated marker and fail loudly when it is absent or ambiguous."""
    count = text.count(marker)
    if count == 0:
        raise RuntimeError(f"Missing generated marker in {path}: {marker!r}")
    if count > 1:
        raise RuntimeError(f"Ambiguous generated marker in {path}: {marker!r}")
    return text.replace(marker, replacement, 1)


def insert_before(path: Path, marker: str, addition: str) -> None:
    """Insert an addition before one generated marker, preserving line endings."""
    text, newline = read_source(path)
    if addition.strip() in text:
        return
    text = replace_once(text, marker, addition + marker, path)
    write_source(path, text, newline)


def insert_after(path: Path, marker: str, addition: str) -> None:
    """Insert an addition after one generated marker, preserving line endings."""
    text, newline = read_source(path)
    if addition.strip() in text:
        return
    text = replace_once(text, marker, marker + addition, path)
    write_source(path, text, newline)


def add_include(path: Path) -> None:
    """Add the operation options include when generated code does not have it."""
    text, newline = read_source(path)
    include = '#include "operation_options.hpp"\n'
    if include in text:
        return
    marker = '#include "mavsdk_export.h"\n'
    text = replace_once(text, marker, marker + include, path)
    write_source(path, text, newline)


def add_action_api() -> None:
    """Add relative-altitude goto forwarding methods and declarations."""
    source = REPO_ROOT / "cpp/src/mavsdk/plugins/action/action.cpp"
    text, _ = read_source(source)
    if "Action::Result Action::goto_location_relative(" not in text:
        insert_before(
            source,
            "void Action::goto_location_fixedwing_async(",
            """Action::Result Action::goto_location_relative(
    double latitude_deg, double longitude_deg, float relative_altitude_m, float yaw_deg) const
{
    return _impl->goto_location_relative(latitude_deg, longitude_deg, relative_altitude_m, yaw_deg);
}

Action::Result Action::goto_location_relative(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float yaw_deg,
    const OperationOptions& options) const
{
    return _impl->goto_location_relative(
        latitude_deg, longitude_deg, relative_altitude_m, yaw_deg, options);
}

""",
        )

    header = REPO_ROOT / "cpp/src/mavsdk/plugins/action/include/plugins/action/action.hpp"
    add_include(header)
    text, _ = read_source(header)
    if "Result goto_location_relative(" not in text:
        marker = "    Result goto_location(double latitude_deg, double longitude_deg, float absolute_altitude_m, float yaw_deg) const;\n"
        addition = """

    /**
     * @brief Move the vehicle to a global position using altitude above home.
     *
     * The latitude and longitude are WGS84 degrees. The altitude is in metres
     * relative to the home position. Yaw is in degrees, clockwise from North.
     *
     * @return Result of request.
     */
    Result goto_location_relative(
        double latitude_deg,
        double longitude_deg,
        float relative_altitude_m,
        float yaw_deg) const;

    /**
     * @brief Move the vehicle to a global position using altitude above home.
     *
     * The timeout bounds the complete operation, including a required flight
     * mode change and all command retries.
     *
     * @return Result of request.
     */
    Result goto_location_relative(
        double latitude_deg,
        double longitude_deg,
        float relative_altitude_m,
        float yaw_deg,
        const OperationOptions& options) const;
"""
        text = replace_once(text, marker, marker + addition, header)
        _, newline = read_source(header)
        write_source(header, text, newline)


def add_param_api() -> None:
    """Add operation-scoped parameter getter forwarding methods and declarations."""
    source = REPO_ROOT / "cpp/src/mavsdk/plugins/param/param.cpp"
    text, _ = read_source(source)
    if "Param::get_param_int(std::string name, const OperationOptions& options)" not in text:
        insert_before(
            source,
            "Param::Result Param::set_param_int(",
            """std::pair<Param::Result, int32_t>
Param::get_param_int(std::string name, const OperationOptions& options) const
{
    return _impl->get_param_int(name, options);
}

""",
        )
    text, _ = read_source(source)
    if "Param::get_param_float(std::string name, const OperationOptions& options)" not in text:
        insert_before(
            source,
            "Param::Result Param::set_param_float(",
            """std::pair<Param::Result, float>
Param::get_param_float(std::string name, const OperationOptions& options) const
{
    return _impl->get_param_float(name, options);
}

""",
        )

    header = REPO_ROOT / "cpp/src/mavsdk/plugins/param/include/plugins/param/param.hpp"
    add_include(header)
    text, _ = read_source(header)
    int_signature = "std::pair<Result, int32_t> get_param_int(\n        std::string name, const OperationOptions& options) const;\n"
    if int_signature not in text:
        marker = "    std::pair<Result, int32_t> get_param_int(std::string name) const;\n"
        addition = """

    /**
     * @brief Get an int parameter with an overall operation timeout.
     *
     * The timeout includes queueing and all internal retries.
     */
    std::pair<Result, int32_t> get_param_int(
        std::string name, const OperationOptions& options) const;
"""
        insert_after(header, marker, addition)

    text, _ = read_source(header)
    float_signature = "std::pair<Result, float> get_param_float(\n        std::string name, const OperationOptions& options) const;\n"
    if float_signature not in text:
        marker = "    std::pair<Result, float> get_param_float(std::string name) const;\n"
        addition = """

    /**
     * @brief Get a float parameter with an overall operation timeout.
     *
     * The timeout includes queueing and all internal retries.
     */
    std::pair<Result, float> get_param_float(
        std::string name, const OperationOptions& options) const;
"""
        insert_after(header, marker, addition)


def main() -> None:
    """Apply all generated compatibility additions."""
    add_action_api()
    add_param_api()


if __name__ == "__main__":
    main()
