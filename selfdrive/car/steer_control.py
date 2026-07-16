def is_curvature_steer_control(steer_control_type) -> bool:
  """Support C4's frozen opendbc enum while downstream uses the formal curvature API."""
  return getattr(steer_control_type, "name", str(steer_control_type)) in ("curvature", "curvatureDEPRECATED")
