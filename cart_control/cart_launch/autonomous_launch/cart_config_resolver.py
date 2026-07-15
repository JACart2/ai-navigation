import os

from ament_index_python.packages import get_package_share_directory


_DOMAIN_TO_CART = {
    0: "james",
    1: "madison",
}


def _cart_config_path(cart_name: str) -> str:
    return os.path.join(
        get_package_share_directory("cart_launch"),
        "config",
        f"cart_{cart_name}.yaml",
    )


def resolve_cart_config_path(explicit_path: str = "") -> str:
    """Return an explicit cart config path or infer one from ROS_DOMAIN_ID."""
    if explicit_path:
        return explicit_path

    domain_raw = os.environ.get("ROS_DOMAIN_ID", "0").strip()
    try:
        domain_id = int(domain_raw)
    except ValueError as exc:
        raise RuntimeError(
            f"ROS_DOMAIN_ID must be an integer, got '{domain_raw}'"
        ) from exc

    cart_name = _DOMAIN_TO_CART.get(domain_id)
    if cart_name is None:
        raise RuntimeError(
            "No cart config mapping is defined for ROS_DOMAIN_ID="
            f"{domain_id}. Supported mappings: 0 -> james, 1 -> madison."
        )

    return _cart_config_path(cart_name)


def default_cart_config_path() -> str:
    """Return the default cart config path inferred from ROS_DOMAIN_ID."""
    return resolve_cart_config_path()
